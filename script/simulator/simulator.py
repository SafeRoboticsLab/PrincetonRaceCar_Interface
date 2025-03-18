#!/usr/bin/env python3
import rclpy
import numpy as np
import threading
import time
import queue

# Import ROS 2 message and service types
from nav_msgs.msg import Odometry
from racecar_msgs.msg import ServoMsg
from racecar_interface.srv import Reset

# Import transformation functions (ensure you have the appropriate ROS2-compatible package)
from tf_transformations import quaternion_about_axis

# Import custom modules (ensure they are ROS2 compatible)
from .realtime_buffer import RealtimeBuffer
from .dynamics import Bicycle4D

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter import SetParametersResult


class Simulator(Node):
    def __init__(self):
        super().__init__('simulation_node')

        # Declare parameters and read values
        self.declare_parameters()

        # Initialize state and dynamics
        self.sigma = np.zeros(2)
        self.latency = 0.0
        self.reset_latency = False
        self.update_lock = threading.Lock()
        self.current_state = np.array([self.init_x, self.init_y, 0, self.init_yaw])
        self.dyn = Bicycle4D(1.0 / self.pub_rate)

        # Set up the control buffer
        self.control_buffer = RealtimeBuffer()

        # Create publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 1)
        # Create subscription for control input
        self.control_sub = self.create_subscription(ServoMsg, self.control_topic, self.control_callback, 1)
        # Create service for resetting simulation
        self.reset_srv = self.create_service(Reset, self.service_name, self.reset_cb)

        # Set up dynamic parameter callback to emulate dynamic_reconfigure
        self.add_on_set_parameters_callback(self.reconfigure_callback)

        # Start simulation thread in a separate thread
        threading.Thread(target=self.simulation_thread, daemon=True).start()

        self.get_logger().info("Simulation node started successfully.")

    def declare_parameters(self):
        """Declare ROS 2 parameters with default values."""
        self.control_topic = self.declare_parameter('control_topic', '/control').value
        self.odom_topic = self.declare_parameter('odom_topic', '/sim_pose').value
        self.pub_rate = self.declare_parameter('pub_rate', 30).value
        self.init_x = self.declare_parameter('init_x', 0.0).value
        self.init_y = self.declare_parameter('init_y', 0.0).value
        self.init_yaw = self.declare_parameter('init_yaw', 0.0).value
        self.service_name = self.declare_parameter('service_name', '/simulation/reset').value
        # Declare noise parameters (for dynamic updates)
        self.declare_parameter('throttle_noise_sigma', 0.0)
        self.declare_parameter('steer_noise_sigma', 0.0)
        self.declare_parameter('latency', 0.0)

    def reset_cb(self, request, response):
        """ROS 2 service callback to reset the simulation."""
        with self.update_lock:
            # Reset state to the provided values
            self.current_state = np.array([request.x, request.y, 0, request.yaw])
            self.get_logger().info(f"Simulation reset to {self.current_state}")
        response.success = True
        return response

    def reconfigure_callback(self, params):
        """Handles dynamic parameter changes in ROS 2."""
        with self.update_lock:
            for param in params:
                if param.name == 'throttle_noise_sigma':
                    self.sigma[0] = param.value
                elif param.name == 'steer_noise_sigma':
                    self.sigma[1] = param.value
                elif param.name == 'latency':
                    new_latency = param.value
                    self.reset_latency = (new_latency != self.latency)
                    self.latency = new_latency
            self.get_logger().info(f"Simulation noise updated to {self.sigma}. Latency updated to {self.latency} s")
        return SetParametersResult(successful=True)

    def control_callback(self, msg):
        """Handles incoming control commands."""
        control = np.array([msg.throttle, msg.steer])
        self.control_buffer.writeFromNonRT(control)

    def simulation_thread(self):
        """Main simulation loop running in a separate thread."""
        rate = 1.0 / self.pub_rate
        msg_queue = queue.Queue()
        while rclpy.ok():
            with self.update_lock:
                # Read the latest control input
                control = self.control_buffer.readFromRT()
                if control is not None:
                    self.current_state = self.dyn.integrate(self.current_state, control, self.sigma)

                # Normalize the yaw angle
                self.current_state[3] = np.arctan2(np.sin(self.current_state[3]), np.cos(self.current_state[3]))

                # Create an Odometry message
                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = 'map'
                odom_msg.pose.pose.position.x = self.current_state[0]
                odom_msg.pose.pose.position.y = self.current_state[1]
                odom_msg.pose.pose.position.z = 0

                q = quaternion_about_axis(self.current_state[3], (0, 0, 1))
                odom_msg.pose.pose.orientation.x = q[0]
                odom_msg.pose.pose.orientation.y = q[1]
                odom_msg.pose.pose.orientation.z = q[2]
                odom_msg.pose.pose.orientation.w = q[3]

                odom_msg.twist.twist.linear.x = self.current_state[2]

                if self.reset_latency:
                    self.get_logger().info("Clearing message queue due to latency reset")
                    with msg_queue.mutex:
                        msg_queue.queue.clear()
                    self.reset_latency = False

                msg_queue.put(odom_msg)
                t_cur = self.get_clock().now().nanoseconds * 1e-9  # convert to seconds
                t_queue_top = msg_queue.queue[0].header.stamp.sec + msg_queue.queue[0].header.stamp.nanosec * 1e-9
                dt = t_cur - t_queue_top

                # Publish the odometry message if the latency delay has been met
                if dt >= self.latency:
                    odom_msg = msg_queue.get()
                    self.odom_pub.publish(odom_msg)

            time.sleep(rate)


def main():
    rclpy.init()
    node = Simulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down simulation node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
