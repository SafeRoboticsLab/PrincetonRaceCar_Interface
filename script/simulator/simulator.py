import rospy
import numpy as np
from .realtime_buffer import RealtimeBuffer
from .dynamics import Bicycle4D
from nav_msgs.msg import Odometry
from racecar_msgs.msg import ServoMsg
from tf.transformations import quaternion_about_axis
import threading
from dynamic_reconfigure.server import Server
from racecar_interface.cfg import simConfig
from racecar_interface.srv import Reset
import queue


class Simulator:
    def __init__(self):
        # read parameters
        control_topic = rospy.get_param('~control_topic', '/control')
        odom_topic = rospy.get_param('~odom_topic', '/sim_pose')
        self.pub_rate = rospy.get_param('~pub_rate', 30)
        
        init_x= rospy.get_param('~init_x', 0)
        init_y = rospy.get_param('~init_y', 0)
        init_yaw = rospy.get_param('~init_yaw', 0)
        serice_name = rospy.get_param('~serice_name', '/simulation/reset')
        
        self.sigma = np.zeros(2)
        self.latency = 0 
        self.reset_latency = False
        self.update_lock = threading.Lock()
        
        self.current_state = np.array([init_x, init_y, 0, init_yaw])        
        self.dyn = Bicycle4D(1.0/self.pub_rate)
        
        self.control_buffer = RealtimeBuffer()
        
        self.odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size=1)
        self.control_sub = rospy.Subscriber(control_topic, ServoMsg, self.control_callback, queue_size=1)
        
        self.dyn_server = Server(simConfig, self.reconfigure_callback)
        
        self.reset_srv = rospy.Service(serice_name, Reset, self.reset_cb)
    
        threading.Thread(target=self.simulation_thread).start()
    
    def reset_cb(self, req):
        self.update_lock.acquire()
        self.current_state = np.array([req.x, req.y, 0, req.yaw])
        rospy.loginfo(f"Simulation Reset to {self.current_state}")
        self.update_lock.release()
        return True
    
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
        
    def control_callback(self, msg):
        control = np.array([msg.throttle, msg.steer])
        self.control_buffer.writeFromNonRT(control)
        
    def simulation_thread(self):
        rate = rospy.Rate(self.pub_rate)
        msg_queue = queue.Queue()
        while not rospy.is_shutdown():
            # read control
            self.update_lock.acquire()
            control = self.control_buffer.readFromRT()
            if control is not None:
                self.current_state = self.dyn.integrate(self.current_state, control, self.sigma)
            
            self.current_state[3] = np.arctan2(np.sin(self.current_state[3]), np.cos(self.current_state[3]))
            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = 'map'
            odom_msg.pose.pose.position.x = self.current_state[0]
            odom_msg.pose.pose.position.y = self.current_state[1]
            odom_msg.pose.pose.position.z = 0
            
            q = quaternion_about_axis(self.current_state[3], (0,0,1))
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]
            
            odom_msg.twist.twist.linear.x = self.current_state[2]
            
            if self.reset_latency:
                print("Clearing Queue")
                msg_queue.queue.clear()
                self.reset_latency = False
                
            msg_queue.put(odom_msg)
            t_cur = rospy.Time.now().to_sec()
            t_queue_top = msg_queue.queue[0].header.stamp.to_sec()
            dt = t_cur - t_queue_top
            
            # simulate latency with delayed publishing
            if dt >=  self.latency:
                odom_msg = msg_queue.get()
                self.odom_pub.publish(odom_msg)
            
            # latency.sleep()
            # self.odom_pub.publish(odom_msg)
            self.update_lock.release()
            rate.sleep()