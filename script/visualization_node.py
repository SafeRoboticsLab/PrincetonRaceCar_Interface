#!/usr/bin/env python3
import rclpy  # Replaces rospy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
from racecar_msgs.msg import OdometryArray
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import euler_from_quaternion
import numpy as np
import copy

class TruckVis(Node):
    """
    ROS 2 node that visualizes the truck, dynamic obstacles, and environment.
    """
    def __init__(self):
        super().__init__('visualization_node')

        self.get_logger().info("Visualization node started")

        # Declare parameters
        self.declare_parameter("odom_topic", "/slam_pose")
        self.declare_parameter("dyn_obs_topic", "/Obstacles/Dynamic")

        # Get parameter values
        odom_topic = self.get_parameter("odom_topic").value
        dyn_obs_topic = self.get_parameter("dyn_obs_topic").value

        # Setup subscribers
        self.pose_sub = self.create_subscription(Odometry, odom_topic, self.odometry_callback, 1)
        self.dyn_obs_sub = self.create_subscription(OdometryArray, dyn_obs_topic, self.dyn_obs_callback, 1)

        # Setup publishers
        self.car_pub = self.create_publisher(MarkerArray, "/vis/truck", 1)
        self.origin_pub = self.create_publisher(PoseStamped, "/vis/origin", 1)
        self.playground_pub = self.create_publisher(Marker, "/vis/playground", 1)
        self.dyn_obs_pub = self.create_publisher(MarkerArray, "/vis/dyn_obs", 1)

    def dyn_obs_callback(self, msg):
        color = [204/255.0, 51/255.0, 0/255.0, 0.5]
        msg_to_pub = MarkerArray()
        for i, obs in enumerate(msg.odom_list):
            self.visualize_car(obs, 'obs', i, color, msg_to_pub)
        self.dyn_obs_pub.publish(msg_to_pub)

    def odometry_callback(self, msg):
        color = [255/255.0, 165/255.0, 15/255.0, 0.5]
        msg_to_pub = MarkerArray()
        self.visualize_car(msg, 'ego', 0, color, msg_to_pub)
        self.car_pub.publish(msg_to_pub)

        self.visualize_origin()
        self.visualize_playground()

    def visualize_origin(self):
        marker = PoseStamped()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        self.origin_pub.publish(marker)

    def visualize_playground(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_STRIP
        marker.ns = 'playground'
        marker.id = 0
        marker.action = Marker.ADD
        marker.scale.x = 0.02

        marker.points = [
            Point(x=0, y=0, z=0),
            Point(x=6.1, y=0, z=0),
            Point(x=6.1, y=6.1, z=0),
            Point(x=0, y=6.1, z=0),
            Point(x=0, y=0, z=0)
        ]

        marker.color.r = 204 / 255.0
        marker.color.g = 0 / 255.0
        marker.color.b = 0 / 255.0
        marker.color.a = 1.0

        self.playground_pub.publish(marker)

    def visualize_car(self, msg, ns, id, color, marker_array):
        # Create vehicle marker (CUBE)
        cuboid = Marker()
        cuboid.header = msg.header
        cuboid.ns = ns
        cuboid.id = id
        cuboid.type = Marker.CUBE
        cuboid.action = Marker.ADD
        cuboid.scale.x = 0.42
        cuboid.scale.y = 0.19
        cuboid.scale.z = 0.188
        cuboid.pose = copy.deepcopy(msg.pose.pose)

        q = [cuboid.pose.orientation.x, cuboid.pose.orientation.y, cuboid.pose.orientation.z, cuboid.pose.orientation.w]
        yaw = euler_from_quaternion(q)[-1]
        cuboid.pose.position.x += 0.1285 * np.cos(yaw)
        cuboid.pose.position.y += 0.1285 * np.sin(yaw)
        cuboid.pose.position.z = 0

        cuboid.color.r = color[0]
        cuboid.color.g = color[1]
        cuboid.color.b = color[2]
        cuboid.color.a = 0.5
        cuboid.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        marker_array.markers.append(cuboid)

        # Create direction arrow marker
        arrow = Marker()
        arrow.header = msg.header
        arrow.ns = ns + "_arrow"
        arrow.id = id
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = copy.deepcopy(msg.pose.pose)
        arrow.scale.x = 0.3
        arrow.scale.y = 0.02
        arrow.scale.z = 0.02
        arrow.color.r = 1.0
        arrow.color.g = 1.0
        arrow.color.b = 1.0
        arrow.color.a = 1.0
        arrow.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        marker_array.markers.append(arrow)

def main():
    rclpy.init()
    node = TruckVis()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down visualization node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
