#!/usr/bin/env python3
import rclpy  # Replaces rospy
from rclpy.node import Node
from simulator import TrafficSimulator

class TrafficSimulationNode(Node):
    """
    ROS 2 node that initializes and runs the Traffic Simulator.
    """
    def __init__(self):
        super().__init__('traffic_simulation_node')
        self.get_logger().info("Traffic simulation node started")

        # Declare parameters (Must be declared before using get_parameter in ROS 2)
        self.declare_parameter("map_file", "path/to/default_map.pkl")
        self.declare_parameter("num_dyn_obs", 1)
        self.declare_parameter("num_static_obs", 1)
        self.declare_parameter("static_obs_size", 0.2)
        self.declare_parameter("static_obs_topic", "/Obstacles/Static")
        self.declare_parameter("dyn_obs_topic", "/Obstacles/Dynamic")
        self.declare_parameter("pub_rate", 30)

        # Retrieve parameter values
        map_file = self.get_parameter("map_file").value
        num_dyn_obs = self.get_parameter("num_dyn_obs").value
        num_static_obs = self.get_parameter("num_static_obs").value
        static_obs_size = self.get_parameter("static_obs_size").value
        static_obs_topic = self.get_parameter("static_obs_topic").value
        dyn_obs_topic = self.get_parameter("dyn_obs_topic").value
        pub_rate = self.get_parameter("pub_rate").value

        # Initialize the Traffic Simulator with all required parameters
        self.simulator = TrafficSimulator(
            map_file, num_dyn_obs, num_static_obs, static_obs_size, static_obs_topic, dyn_obs_topic, pub_rate
        )

def main():
    rclpy.init()
    node = TrafficSimulationNode()

    try:
        rclpy.spin(node)  # Keeps the node running
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down traffic simulation node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
