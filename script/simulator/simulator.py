import rospy
import numpy as np
from .realtime_buffer import RealtimeBuffer
from .dynamics import Bicycle4D
from nav_msgs.msg import Odometry
from racecar_msgs.msg import ServoMsg
from tf.transformations import quaternion_about_axis
import threading

class Simulator:
    def __init__(self):
        # read parameters
        control_topic = rospy.get_param('~control_topic', '/control')
        odom_topic = rospy.get_param('~odom_topic', '/sim_pose')
        self.pub_rate = rospy.get_param('~pub_rate', 30)
        
        init_x= rospy.get_param('~init_x', 0)
        init_y = rospy.get_param('~init_y', 0)
        init_yaw = rospy.get_param('~init_yaw', 0)
        
        self.current_state = np.array([init_x, init_y, 0, init_yaw])        
        self.dyn = Bicycle4D(1.0/self.pub_rate)
        
        self.control_buffer = RealtimeBuffer()
        
        self.odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size=1)
        self.control_sub = rospy.Subscriber(control_topic, ServoMsg, self.control_callback, queue_size=1)
        
        threading.Thread(target=self.simulation_thread).start()
        
    def control_callback(self, msg):
        control = np.array([msg.throttle, msg.steer])
        self.control_buffer.writeFromNonRT(control)
        
    def simulation_thread(self):
        rate = rospy.Rate(self.pub_rate)
        while not rospy.is_shutdown():
            # read control
            control = self.control_buffer.readFromRT()
            if control is not None:
                self.current_state = self.dyn.integrate(self.current_state, control)
            
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
            rate.sleep()