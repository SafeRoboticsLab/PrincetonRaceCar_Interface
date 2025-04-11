#!/usr/bin/env python3
import rclpy  # ROS2 client library
from rclpy.node import Node  # Base class for ROS2 nodes
from simulator import TrafficSimulator  # Import the traffic simulator node

class TrafficSimulationNode(Node):
    """
    ROS 2 node that initializes and runs the Traffic Simulator.
    
    Changes compared to a potential ROS1 version:
      - Uses rclpy and subclasses rclpy.node.Node.
      - Declares and retrieves parameters using ROS2 parameter API.
      - Instead of passing all parameters from here, only the map file is passed
        because TrafficSimulator reads its own parameters internally.
    """
    def __init__(self):
        # Initialize the node with the given name.
        super().__init__('traffic_simulation_node')
        self.get_logger().info("Traffic simulation node started")

        # Declare parameters. In ROS2 each parameter must be declared before use.
        # The default map file can be overridden via a launch file or YAML.
        self.declare_parameter("map_file", "path/to/default_map.pkl")
        # Although additional parameters are declared here in the original ROS1 code,
        # our TrafficSimulator already handles its own parameters internally.
        # Therefore, we only need to retrieve the map file here.
        map_file = self.get_parameter("map_file").value

        # Initialize the TrafficSimulator node.
        # Note: We only pass the map_file argument since other parameters
        # are managed inside TrafficSimulator via its read_parameters() method.
        self.simulator = TrafficSimulator(map_file)

def main():
    # Initialize the ROS2 client library.
    rclpy.init()
    # Create an instance of the TrafficSimulationNode.
    node = TrafficSimulationNode()
    try:
        # Spin the node to process callbacks.
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down traffic simulation node...")
    finally:
        # Clean up: destroy the node and shutdown rclpy properly.
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
