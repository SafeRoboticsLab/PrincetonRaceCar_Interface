#!/usr/bin/env python3
import rclpy  # Replaces rospy
from rclpy.node import Node
from simulator import Simulator

class SimulationNode(Node):
    """
    ROS 2 node that initializes and runs the Simulator.
    """
    def __init__(self):
        super().__init__('simulation_node')

        self.get_logger().info("Simulation node started")

        # Declare parameters from YAML
        self.declare_parameter("throttle_noise_sigma", 0.0)
        self.declare_parameter("steer_noise_sigma", 0.0)
        self.declare_parameter("latency", 0.0)
        self.declare_parameter("odom_topic", "/Simulation/Pose")
        self.declare_parameter("control_topic", "/Control")
        self.declare_parameter("pub_rate", 30)
        self.declare_parameter("init_x", 0.0)
        self.declare_parameter("init_y", 0.0)
        self.declare_parameter("init_yaw", 0.0)

        # Get parameter values
        throttle_noise_sigma = self.get_parameter("throttle_noise_sigma").value
        steer_noise_sigma = self.get_parameter("steer_noise_sigma").value
        latency = self.get_parameter("latency").value
        odom_topic = self.get_parameter("odom_topic").value
        control_topic = self.get_parameter("control_topic").value
        pub_rate = self.get_parameter("pub_rate").value
        init_x = self.get_parameter("init_x").value
        init_y = self.get_parameter("init_y").value
        init_yaw = self.get_parameter("init_yaw").value

        # Initialize the Simulator with parameters
        self.simulator = Simulator(
            odom_topic=odom_topic,
            control_topic=control_topic,
            pub_rate=pub_rate,
            init_x=init_x,
            init_y=init_y,
            init_yaw=init_yaw,
            throttle_noise_sigma=throttle_noise_sigma,
            steer_noise_sigma=steer_noise_sigma,
            latency=latency
        )

def main():
    rclpy.init()
    node = SimulationNode()

    try:
        rclpy.spin(node)  # Keeps the node running
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down simulation node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
