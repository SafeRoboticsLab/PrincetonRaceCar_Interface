#!/usr/bin/env python3
import rclpy
import numpy as np
import threading
import time
import queue

# Import ROS2 message and service types
from nav_msgs.msg import Odometry
from racecar_msgs.msg import ServoMsg
from racecar_interface.srv import Reset

# Import transformation function (ensure you have a ROS2-compatible package)
from tf_transformations import quaternion_about_axis

# Import custom modules (ensure they are updated for ROS2)
from realtime_buffer import RealtimeBuffer
from dynamics import Bicycle4D

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter import SetParametersResult

class Simulator(Node):
    def __init__(self):
        # Initialize the node with a name. We choose 'simulation_node' so that you can load parameters
        # from sim_params.yaml if you update its top-level key accordingly.
        super().__init__('simulation_node')
        
        # Declare parameters with defaults.
        # These parameters can be overridden by a YAML file (sim_params.yaml) loaded via the launch file.
        self.declare_parameter('control_topic', '/control')
        self.declare_parameter('odom_topic', '/sim_pose')
        self.declare_parameter('pub_rate', 30)
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw', 0.0)
        self.declare_parameter('service_name', '/simulation/reset')
        self.declare_parameter('throttle_noise_sigma', 0.0)
        self.declare_parameter('steer_noise_sigma', 0.0)
        self.declare_parameter('latency', 0.0)
        
        # Retrieve parameter values.
        control_topic = self.get_parameter('control_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.pub_rate = self.get_parameter('pub_rate').value
        init_x = self.get_parameter('init_x').value
        init_y = self.get_parameter('init_y').value
        init_yaw = self.get_parameter('init_yaw').value
        service_name = self.get_parameter('service_name').value
        
        # Initialize noise and latency parameters.
        self.sigma = np.zeros(2)
        self.latency = 0.0 
        self.reset_latency = False
        
        # Lock for safe concurrent access.
        self.update_lock = threading.Lock()
        
        # Initialize simulation state and dynamics.
        self.current_state = np.array([init_x, init_y, 0.0, init_yaw])
        self.dyn = Bicycle4D(1.0 / self.pub_rate)
        
        # Set up the control buffer.
        self.control_buffer = RealtimeBuffer()
        
        # Create publisher and subscriber.
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 1)
        self.control_sub = self.create_subscription(ServoMsg, control_topic, self.control_callback, 1)
        
        # Create service for resetting simulation.
        self.reset_srv = self.create_service(Reset, service_name, self.reset_cb)
        
        # Add parameter callback for dynamic updates.
        self.add_on_set_parameters_callback(self.reconfigure_callback)
        
        # Start simulation thread (daemonized so it shuts down with the node).
        threading.Thread(target=self.simulation_thread, daemon=True).start()
        
        self.get_logger().info("Simulation node started successfully.")

    def reset_cb(self, request, response):
        # ROS2 service callback to reset simulation state.
        with self.update_lock:
            self.current_state = np.array([request.x, request.y, 0.0, request.yaw])
            self.get_logger().info(f"Simulation reset to {self.current_state}")
        response.success = True
        return response

    def reconfigure_callback(self, params):
        # ROS2 parameter callback to update noise and latency parameters.
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
        # Update the control buffer with new control inputs.
        control = np.array([msg.throttle, msg.steer])
        self.control_buffer.writeFromNonRT(control)

    def simulation_thread(self):
        # Main simulation loop running in a separate thread.
        rate = 1.0 / self.pub_rate
        msg_queue = queue.Queue()
        while rclpy.ok():
            with self.update_lock:
                # Process control input if available.
                control = self.control_buffer.readFromRT()
                if control is not None:
                    self.current_state = self.dyn.integrate(self.current_state, control, self.sigma)
                
                # Normalize the yaw angle.
                self.current_state[3] = np.arctan2(np.sin(self.current_state[3]), np.cos(self.current_state[3]))
                
                # Create an Odometry message.
                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = 'map'
                odom_msg.pose.pose.position.x = self.current_state[0]
                odom_msg.pose.pose.position.y = self.current_state[1]
                odom_msg.pose.pose.position.z = 0.0
                
                q = quaternion_about_axis(self.current_state[3], (0, 0, 1))
                odom_msg.pose.pose.orientation.x = q[0]
                odom_msg.pose.pose.orientation.y = q[1]
                odom_msg.pose.pose.orientation.z = q[2]
                odom_msg.pose.pose.orientation.w = q[3]
                
                odom_msg.twist.twist.linear.x = self.current_state[2]
                
                # Clear the queue if latency was reset.
                if self.reset_latency:
                    self.get_logger().info("Clearing message queue due to latency reset")
                    with msg_queue.mutex:
                        msg_queue.queue.clear()
                    self.reset_latency = False
                
                msg_queue.put(odom_msg)
                
                # Determine if the latency delay has been met.
                t_cur = self.get_clock().now().nanoseconds * 1e-9  # current time in seconds
                t_queue_top = msg_queue.queue[0].header.stamp.sec + msg_queue.queue[0].header.stamp.nanosec * 1e-9
                dt = t_cur - t_queue_top
                
                if dt >= self.latency:
                    odom_msg = msg_queue.get()
                    self.odom_pub.publish(odom_msg)
                    
            time.sleep(rate)

def main(args=None):
    rclpy.init(args=args)
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
