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
from std_srvs.srv import Empty, EmptyResponse

class Simulator:
    def __init__(self):
        # read parameters
        control_topic = rospy.get_param('~control_topic', '/control')
        odom_topic = rospy.get_param('~odom_topic', '/sim_pose')
        self.pub_rate = rospy.get_param('~pub_rate', 30)
        
        init_x= rospy.get_param('~init_x', 0)
        init_y = rospy.get_param('~init_y', 0)
        init_yaw = rospy.get_param('~init_yaw', 0)
        
        self.sigma = np.zeros(2)
        self.update_lock = threading.Lock()
        
        self.current_state = np.array([init_x, init_y, 0, init_yaw])        
        self.dyn = Bicycle4D(1.0/self.pub_rate)
        
        self.control_buffer = RealtimeBuffer()
        
        self.odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size=1)
        self.control_sub = rospy.Subscriber(control_topic, ServoMsg, self.control_callback, queue_size=1)
        
        self.dyn_server = Server(simConfig, self.reconfigure_callback)
        
        self.reset_srv = rospy.Service('/simulation/reset', Reset, self.reset_cb)
        
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
        rospy.loginfo(f"Simulation Noise Updated to {self.sigma}")
        self.update_lock.release()
        return config
        
    def control_callback(self, msg):
        control = np.array([msg.throttle, msg.steer])
        self.control_buffer.writeFromNonRT(control)
        
    def simulation_thread(self):
        rate = rospy.Rate(self.pub_rate)
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
            
            self.odom_pub.publish(odom_msg)
            self.update_lock.release()
            rate.sleep()