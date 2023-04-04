import rospy
import pickle
import numpy as np
from .dynamics import Bicycle4D
from .ref_path import RefPath
from .ros_utility import get_ros_param
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
from racecar_msgs.msg import OdometryArray
from tf.transformations import quaternion_about_axis
import threading
from dynamic_reconfigure.server import Server
from racecar_interface.cfg import simConfig
from racecar_interface.srv import ResetObstacle
import queue


class TrafficSimulator:
    def __init__(self, map_file):
        
        with open(map_file, 'rb') as f:
            self.lanelet_map = pickle.load(f)
        self.lanelet_map.build_graph(0.5)
        
        self.read_parameters()
        self.sigma = np.zeros(2)
        self.K = np.ones(2)
        self.latency = 0 
        self.reset_latency = False
        self.update_lock = threading.Lock()
        
        self.dyn = Bicycle4D(1.0/self.pub_rate)
        
        self.dyn_server = Server(simConfig, self.reconfigure_callback)
        self.reset_srv = rospy.Service('/simulation/reset_static_obstacle', ResetObstacle, self.reset_cb)
        
        self.setup_publisher()

        threading.Thread(target=self.simulation_thread).start()

    def reset_cb(self, req):
        self.update_lock.acquire()
        self.num_static_obj = req.n
        self.static_obs_msg = self.create_static_obs()
        rospy.loginfo(f"Static Obstacle Reset")
        self.update_lock.release()
        return True
        
    def read_parameters(self):
        # read parameters
        self.num_dyn_obj = get_ros_param('~num_dyn_obs', 1)
        self.num_static_obj = get_ros_param('~num_static_obs', 1)
        self.static_obs_size = get_ros_param('~static_obs_size', 0.2)
        self.static_obs_topic = get_ros_param('~static_obs_topic', '/Obstacles/Static')
        self.dyn_obs_topic = get_ros_param('~dyn_obs_topic', '/Obstacles/Dynamic')
        self.pub_rate = get_ros_param('~pub_rate', 30)
        self.static_obs_location = get_ros_param('~static_obs_location', None)
        if self.static_obs_location is not None:
            self.static_obs_msg = self.load_static_obs(self.static_obs_location)
        else:
            self.static_obs_msg = self.create_static_obs()
            
    def setup_publisher(self):
        self.static_obs_publisher = rospy.Publisher(self.static_obs_topic, MarkerArray, queue_size=1)
        self.dyn_obs_publisher = rospy.Publisher(self.dyn_obs_topic, OdometryArray, queue_size=1)
        
    def reconfigure_callback(self, config, level):
        self.update_lock.acquire()
        self.sigma[0] = config['throttle_noise_sigma']
        self.sigma[1] = config['steer_noise_sigma']
        
        latency_new = config['latency']
        
        self.reset_latency = (latency_new != self.latency)
        self.latency = latency_new
        
        rospy.loginfo(f"Simulation Noise Updated to {self.sigma}. Latency Updated to {self.latency} s")
        self.update_lock.release()
        return config
        
    def simulation_thread(self):
        print("enter simulation thread")
        dyn_obs_pose = {}
        for i in range(self.num_dyn_obj):
            pose= self.lanelet_map.get_random_waypoint()
            x,y,psi = pose
            state = np.array([x, y, 0, psi])
            ref_path, v_ref = self.gen_ref_path(pose)
            dyn_obs_pose[i] = {'state': state, 'path': ref_path, 'v_ref': v_ref}
        
        rate = rospy.Rate(self.pub_rate)
        msg_queue = queue.Queue()
        
        while not rospy.is_shutdown():
            header = rospy.Header(frame_id = 'map', stamp=rospy.Time.now())
            odom_array_msg = OdometryArray()
            odom_array_msg.header = header
            
            for i in range(self.num_dyn_obj):
                state = dyn_obs_pose[i]['state']
                ref_path = dyn_obs_pose[i]['path']
                v_ref = dyn_obs_pose[i]['v_ref']
                _, _, s = ref_path.get_closest_pts(state[:2])
                
                # replan randomly
                if (1-s)*ref_path.length < state[2] or np.random.uniform()<0.03:
                    pose = np.array([state[0], state[1], state[3]])
                    ref_path, v_ref = self.gen_ref_path(pose)

                    dyn_obs_pose[i]['path'] = ref_path
                    dyn_obs_pose[i]['v_ref'] = v_ref
                    _, _, s = ref_path.get_closest_pts(state[:2])
                
                # calculate control
                # look ahead 1 second
                look_ahead, _ = ref_path.interp(s*ref_path.length + 0.5)
                look_ahead_x = look_ahead[0,0]
                look_ahead_y = look_ahead[1,0]
                
                # apply pure pursuit
                alpha = np.arctan2(look_ahead_y - state[1], look_ahead_x - state[0]) - state[3]
                ld = np.sqrt((look_ahead_x - state[0])**2 + (look_ahead_y - state[1])**2)
                steer = np.arctan2(2*0.257*np.sin(alpha), ld)
                throttle = (v_ref - state[2])*0.5
                
                state_next = self.dyn.integrate(state, np.array([throttle, steer]), self.sigma)
                dyn_obs_pose[i]['state'] = state_next
                
                # create message
                odom_msg = Odometry()
                odom_msg.header = header
                odom_msg.pose.pose.position.x = state_next[0]
                odom_msg.pose.pose.position.y = state_next[1]
                odom_msg.pose.pose.position.z = 0
                
                q = quaternion_about_axis(state_next[3], (0,0,1))
                odom_msg.pose.pose.orientation.x = q[0]
                odom_msg.pose.pose.orientation.y = q[1]
                odom_msg.pose.pose.orientation.z = q[2]
                odom_msg.pose.pose.orientation.w = q[3]
                
                odom_msg.twist.twist.linear.x = state_next[2]
                odom_array_msg.odom_list.append(odom_msg)
                
            if self.reset_latency:
                print("Clearing Queue")
                msg_queue.queue.clear()
                self.reset_latency = False
                
            msg_queue.put(odom_array_msg)
            t_cur = rospy.Time.now().to_sec()
            t_queue_top = msg_queue.queue[0].header.stamp.to_sec()
            dt = t_cur - t_queue_top
            
            # simulate latency with delayed publishing
            if dt >=  self.latency:
                dyn_msg_to_pub = msg_queue.get()
                self.update_lock.acquire()
                static_msg_to_pub = self.static_obs_msg
                for obs in static_msg_to_pub.markers:
                    obs.header = dyn_msg_to_pub.header

                self.static_obs_publisher.publish(static_msg_to_pub)
                self.update_lock.release()

                self.dyn_obs_publisher.publish(dyn_msg_to_pub)
                
            rate.sleep()
            
    def gen_ref_path(self, pose):
        path = None
        
        while path is None:
            goal = self.lanelet_map.get_random_waypoint()
            path = self.lanelet_map.get_shortest_path(pose, goal, True, True, verbose=False)
        v_ref = np.random.uniform(0, 1.0) 
        return RefPath(path[:,:2].T), v_ref
    
    def load_static_obs(self, locations):
        # Setup static obstacles
        static_obs_msg = MarkerArray()
        for i, xy in enumerate(locations):
            # randomly sample a pose
            psi = np.random.uniform(-np.pi, np.pi) # random heading
            
            marker = Marker()   
            marker.ns = 'static_obs'
            marker.id = i
            marker.type = 1 # cube
            marker.action = 0 # add
            
            # pose of the marker
            marker.pose.position.x = xy[0]
            marker.pose.position.y = xy[1]
            marker.pose.position.z = self.static_obs_size/2.0
            
            q = quaternion_about_axis(psi, (0,0,1))
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
            
            marker.scale.x = self.static_obs_size
            marker.scale.y = self.static_obs_size
            marker.scale.z = self.static_obs_size
            
            marker.color.r = 0
            marker.color.g = 0
            marker.color.b = 153/255.0
            marker.color.a = 0.8
            
            marker.lifetime = rospy.Duration(1.5/self.pub_rate)
            
            static_obs_msg.markers.append(marker)
        return static_obs_msg
    
        
            
    def create_static_obs(self):
        print(f"create {self.num_static_obj} static obstacles")
        # Setup static obstacles
        static_obs_msg = MarkerArray()
        for i in range(self.num_static_obj):
            # randomly sample a pose
            x, y, _ = self.lanelet_map.get_random_waypoint(with_neighbors=True)
            psi = np.random.uniform(-np.pi, np.pi) # random heading
            
            marker = Marker()   
            marker.ns = 'static_obs'
            marker.id = i
            marker.type = 1 # cube
            marker.action = 0 # add
            
            # pose of the marker
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = self.static_obs_size/2.0
            
            q = quaternion_about_axis(psi, (0,0,1))
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
            
            marker.scale.x = self.static_obs_size
            marker.scale.y = self.static_obs_size
            marker.scale.z = self.static_obs_size
            
            marker.color.r = 0
            marker.color.g = 0
            marker.color.b = 153/255.0
            marker.color.a = 0.8
            
            marker.lifetime = rospy.Duration(1.5/self.pub_rate)
            
            static_obs_msg.markers.append(marker)
        return static_obs_msg
        