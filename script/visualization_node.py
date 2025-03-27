#!/usr/bin/env python3
import rclpy  # ROS2 client library
from rclpy.node import Node  # Base class for ROS2 nodes
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
from racecar_msgs.msg import OdometryArray
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration  # For marker lifetime
from tf_transformations import euler_from_quaternion  # Ensure ROS2-compatible package is installed
import numpy as np
import copy

class TruckVis(Node):
    def __init__(self):
        # Initialize the node with the name 'visualization_node'
        super().__init__('visualization_node')
        
        # Declare parameters and retrieve them.
        # These parameters can be overridden via YAML (e.g., sim_params.yaml)
        self.declare_parameter('odom_topic', '/slam_pose')
        self.declare_parameter('dyn_obs_topic', '/Obstacles/Dynamic')
        odom_topic = self.get_parameter('odom_topic').value
        dyn_obs_topic = self.get_parameter('dyn_obs_topic').value
        
        # Setup subscribers using ROS2 API.
        self.pose_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odometry_callback,
            1
        )
        self.dyn_obs_sub = self.create_subscription(
            OdometryArray,
            dyn_obs_topic,
            self.dyn_obs_callback,
            1
        )
        
        # Setup publishers using ROS2 API.
        self.car_pub = self.create_publisher(MarkerArray, '/vis/truck', 1)
        self.origin_pub = self.create_publisher(PoseStamped, '/vis/origin', 1)
        self.playground_pub = self.create_publisher(Marker, '/vis/playground', 1)
        self.dyn_obs_pub = self.create_publisher(MarkerArray, '/vis/dyn_obs', 1)
        
    def dyn_obs_callback(self, msg):
        # Callback for dynamic obstacles. Visualizes each obstacle.
        color = [204/255.0, 51/255.0, 0/255.0, 0.5]
        msg_to_pub = MarkerArray()
        # Iterate over each obstacle in the received OdometryArray.
        for i, obs in enumerate(msg.odom_list):
            self.visualize_car(obs, 'obs', i, color, msg_to_pub)
        self.dyn_obs_pub.publish(msg_to_pub)
        
    def odometry_callback(self, msg):
        # Callback for odometry messages (e.g., from SLAM).
        color = [255/255.0, 165/255.0, 15/255.0, 0.5]
        msg_to_pub = MarkerArray()
        self.visualize_car(msg, 'ego', 0, color, msg_to_pub)
        self.car_pub.publish(msg_to_pub)
        
        # Publish the origin and playground markers.
        self.visualize_origin()
        self.visualize_playground()
        
    def visualize_origin(self):
        # Publish a PoseStamped message representing the origin.
        marker = PoseStamped()
        marker.header.frame_id = 'map'
        # Set current time using the node's clock.
        marker.header.stamp = self.get_clock().now().to_msg()
        self.origin_pub.publish(marker)
        
    def visualize_playground(self):
        # Create and publish a Marker message representing a rectangular playground.
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # Use enum values for marker type and action if available.
        marker.type = Marker.LINE_STRIP  # Replaces numeric value 4
        marker.ns = 'playground'
        marker.id = 0
        marker.action = Marker.ADD  # Replaces numeric value 0
        marker.scale.x = 0.02
        
        # Define the rectangle corners.
        pt1 = Point(x=0.0, y=0.0, z=0.0)
        pt2 = Point(x=6.1, y=0.0, z=0.0)
        pt3 = Point(x=6.1, y=6.1, z=0.0)
        pt4 = Point(x=0.0, y=6.1, z=0.0)
        
        marker.points = [pt1, pt2, pt3, pt4, pt1]
        
        marker.color.r = 204/255.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.playground_pub.publish(marker)
        
    def visualize_car(self, msg, ns, id, color, marker_array):
        # Create a cube marker to represent the vehicle.
        cuboid = Marker()
        cuboid.header = msg.header
        cuboid.ns = ns
        cuboid.id = id
        cuboid.type = Marker.CUBE  # Replaces numeric value 1
        cuboid.action = Marker.ADD  # Replaces numeric value 0
        cuboid.scale.x = 0.42
        cuboid.scale.y = 0.19
        cuboid.scale.z = 0.188
        
        # Copy the pose from the incoming message.
        cuboid.pose = copy.deepcopy(msg.pose.pose)
        
        # Calculate yaw from the quaternion.
        q = [
            cuboid.pose.orientation.x,
            cuboid.pose.orientation.y,
            cuboid.pose.orientation.z,
            cuboid.pose.orientation.w
        ]
        yaw = euler_from_quaternion(q)[-1]
        cuboid.pose.position.x += 0.1285 * np.cos(yaw)
        cuboid.pose.position.y += 0.1285 * np.sin(yaw)
        cuboid.pose.position.z = 0.0
        
        # Set the color for the cuboid.
        cuboid.color.r = color[0]
        cuboid.color.g = color[1]
        cuboid.color.b = color[2]
        cuboid.color.a = 0.5
        # Set lifetime using ROS2's Duration message.
        cuboid.lifetime = Duration(sec=0, nanosec=0)
        marker_array.markers.append(cuboid)
        
        # Create an arrow marker to show orientation.
        arrow = Marker()
        arrow.header = msg.header
        arrow.ns = ns + "arrow"
        arrow.id = id
        arrow.type = Marker.ARROW  # Replaces numeric value 0
        arrow.action = Marker.ADD  # Replaces numeric value 0
        arrow.pose = copy.deepcopy(msg.pose.pose)
        
        arrow.scale.x = 0.3
        arrow.scale.y = 0.02
        arrow.scale.z = 0.02
        
        arrow.color.r = 1.0  # Equivalent to 255/255.0
        arrow.color.g = 1.0
        arrow.color.b = 1.0
        arrow.color.a = 1.0
        
        arrow.lifetime = Duration(sec=0, nanosec=0)
        marker_array.markers.append(arrow)
        
def main(args=None):
    # Initialize the ROS2 Python client library.
    rclpy.init(args=args)
    
    # Create an instance of the TruckVis node.
    vis_node = TruckVis()
    vis_node.get_logger().info("Start visualization node")
    
    try:
        # Spin the node to process callbacks.
        rclpy.spin(vis_node)
    except KeyboardInterrupt:
        vis_node.get_logger().info("Visualization node interrupted by keyboard")
    finally:
        # Clean up and shut down the ROS2 node.
        vis_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
