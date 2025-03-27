#!/usr/bin/env python3
import rclpy  # Import ROS2 client library instead of rospy
from simulator import Simulator  # Import Simulator from the local module (unchanged)

def main(args=None):
    # Initialize the ROS2 client library.
    # In ROS1, we used rospy.init_node; in ROS2, we initialize with rclpy.init().
    rclpy.init(args=args)
    
    # Create an instance of the Simulator node.
    # In ROS2, Simulator is now a subclass of rclpy.node.Node, making it a proper ROS2 node.
    node = Simulator()
    
    # Log an informational message using the ROS2 logging system.
    # In ROS1, we used rospy.loginfo; here we use node.get_logger().info().
    node.get_logger().info("Start simulation node")
    
    try:
        # Spin the node to process callbacks.
        # In ROS1, we would use rospy.spin(), but in ROS2 we use rclpy.spin(node).
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle shutdown gracefully if the node is interrupted (e.g., by Ctrl+C).
        node.get_logger().info("Simulation node interrupted by keyboard")
    finally:
        # Cleanup: destroy the node and shut down the ROS2 client library.
        # This is the equivalent of any rospy cleanup and ensures all resources are released.
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
