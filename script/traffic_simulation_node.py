#!/usr/bin/env python3
import rclpy  # Replaces rospy
from rclpy.node import Node
from simulator import TrafficSimulator

class TrafficSimulationNode(Node):
    """
    ROS 2 node that initializes and runs the Traffic Simulator.
    """
    def __init__(self):
        super().__init__('traffic_simulation_node')  # Initializes the node in ROS 2
        self.get_logger().info("Traffic simulation node started")

        # Declare and get the map file parameter (must be declared in ROS 2 before use)
        self.declare_parameter("map_file", "")
        map_file = self.get_parameter("map_file").value

        # Initialize the Traffic Simulator
        self.simulator = TrafficSimulator(map_file)

def main():
    rclpy.init()  # 🔹 Got rid of `rospy.init_node()` because ROS 2 initializes nodes differently
    node = TrafficSimulationNode()

    try:
        rclpy.spin(node)  # Keeps the node running
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down traffic simulation node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()  # 🔹 Got rid of `rospy.spin()` because `rclpy.spin()` is used instead

if __name__ == '__main__':
    main()
