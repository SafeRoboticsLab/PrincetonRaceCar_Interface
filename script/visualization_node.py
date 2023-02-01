#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point
import numpy as np
import copy

class TruckVis:
    def __init__(self) -> None:
        odom_topic = rospy.get_param('~odom_topic', '/slam_pose')
        
        # setup subscribers
        self.pose_sub = rospy.Subscriber(odom_topic, Odometry, self.odometry_callback, queue_size=1)
        
        # setup publishers
        self.car_pub = rospy.Publisher('/vis/truck', MarkerArray, queue_size=1)
        self.origin_pub = rospy.Publisher('/vis/origin', PoseStamped, queue_size=1)
        self.playground_pub = rospy.Publisher('/vis/playground', Marker, queue_size=1)

        
    def odometry_callback(self, msg):
        self.visualize_car(msg)
        self.visualize_origin()
        self.visualize_playground()
        
    def visualize_origin(self):
        marker = PoseStamped()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        self.origin_pub.publish(marker)
        
    def visualize_playground(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        
        marker.type = 4 # Line Strip
        marker.ns = 'playground'
        marker.id = 0
        marker.action = 0 # ADD/modify
        marker.scale.x = 0.02
        
        pt1 = Point()
        pt1.x = 0
        pt1.y = 0
        pt1.z = 0
        
        pt2 = Point()
        pt2.x = 6.1
        pt2.y = 0
        pt2.z = 0
        
        pt3 = Point()
        pt3.x = 6.1
        pt3.y = 6.1
        pt3.z = 0
        
        pt4 = Point()
        pt4.x = 0
        pt4.y = 6.1
        pt4.z = 0
        
        marker.points = [pt1, pt2, pt3, pt4, pt1]
        
        marker.color.r = 204/255.0
        marker.color.g = 0/255.0
        marker.color.b = 0/255.0
        marker.color.a = 1.0
        
        self.playground_pub.publish(marker)
        

    def visualize_car(self, msg):
        marker_array = MarkerArray()
        
        # Create the vehicle marker
        cubiod = Marker()
        cubiod.header = msg.header
        cubiod.ns = 'truck'
        cubiod.id = 0
        cubiod.type = 1 # CUBE
        cubiod.action = 0 # ADD/modify
        cubiod.scale.x = 0.42
        cubiod.scale.y = 0.19
        cubiod.scale.z = 0.188
        
        cubiod.pose = copy.deepcopy(msg.pose.pose)
        
        q = [cubiod.pose.orientation.x, cubiod.pose.orientation.y,
                cubiod.pose.orientation.z, cubiod.pose.orientation.w]
        
        yaw = euler_from_quaternion(q)[-1]
        cubiod.pose.position.x += 0.1285*np.cos(yaw)
        cubiod.pose.position.y += 0.1285*np.sin(yaw)
        cubiod.pose.position.z = 0 
        
        # ORANGE
        cubiod.color.r = 255/255.0
        cubiod.color.g = 165/255.0
        cubiod.color.b = 15/255.0
        cubiod.color.a = 0.5
        cubiod.lifetime = rospy.Duration(0)
        marker_array.markers.append(cubiod)
        
        # Create the arrow marker
        arrow = Marker()
        arrow.header = msg.header
        arrow.ns =  'truck'
        arrow.id = 1
        arrow.type = 0 # ARROW
        arrow.action = 0 # ADD/modify
        arrow.pose = copy.deepcopy(msg.pose.pose)
        
        arrow.scale.x = 0.3
        arrow.scale.y = 0.02
        arrow.scale.z = 0.02
        
        arrow.color.r = 255/255.0
        arrow.color.g = 255/255.0
        arrow.color.b = 255/255.0
        arrow.color.a = 1.0
        
        arrow.lifetime = rospy.Duration(0)
        marker_array.markers.append(arrow)
        
        # publish the marker array
        self.car_pub.publish(marker_array)

    
def main():
    rospy.init_node('visualization_node')
    rospy.loginfo("Start visualization node")
    
    vis = TruckVis()
    rospy.spin()


if __name__ == '__main__':
    main()
