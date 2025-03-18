#!/usr/bin/env python3
import rclpy  # Replaces rospy
from rclpy.node import Node
from simulator import Simulator

class SimulationNode(Node):
    """
    ROS 2 node that initializes and runs the Simulator.
    """
    def __init__(self):
        super().__init__('simulation_node')  # Initializes the node in ROS 2
        self.get_logger().info("Simulation node started")

        # Initialize the Simulator
        self.simulator = Simulator()

def main():
    rclpy.init()  # 🔹 Got rid of `rospy.init_node()` because ROS 2 initializes nodes differently
    node = SimulationNode()

    try:
        rclpy.spin(node)  # Keeps the node running
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down simulation node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()  # 🔹 Got rid of `rospy.spin()` because `rclpy.spin()` is used instead

if __name__ == '__main__':
    main()
