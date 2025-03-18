#!/usr/bin/env python3
import rclpy
import pickle
import numpy as np
import threading
import queue

from rclpy.node import Node
from rclpy.parameter import SetParametersResult
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
from racecar_msgs.msg import OdometryArray
from racecar_interface.srv import ResetObstacle
from tf_transformations import quaternion_about_axis

from .dynamics import Bicycle4D
from .ref_path import RefPath
from .ros_utility import get_ros_param


class TrafficSimulator(Node):
    def __init__(self, map_file):
        super().__init__('traffic_simulator')

        # Load map
        with open(map_file, 'rb') as f:
            self.lanelet_map = pickle.load(f)
        self.lanelet_map.build_graph(0.5)

        # Read parameters
        self.read_parameters()
        self.sigma = np.zeros(2)
        self.K = np.ones(2)
        self.latency = 0
        self.reset_latency = False
        self.update_lock = threading.Lock()
        self.dyn = Bicycle4D(1.0 / self.pub_rate)

        # Initialize publishers
        self.setup_publisher()

        # Create reset service
        self.reset_srv = self.create_service(ResetObstacle, '/simulation/reset_static_obstacle', self.reset_cb)

        # Add dynamic parameter callback
        self.add_on_set_parameters_callback(self.reconfigure_callback)

        # Start simulation thread
        threading.Thread(target=self.simulation_thread, daemon=True).start()

    def reset_cb(self, request, response):
        """Handles reset requests for static obstacles."""
        with self.update_lock:
            self.num_static_obj = request.n
            self.static_obs_msg = self.create_static_obs()
            self.get_logger().info(f"Static Obstacle Reset")
        response.success = True
        return response

    def read_parameters(self):
        """Reads ROS parameters for obstacle simulation."""
        self.num_dyn_obj = self.declare_parameter('num_dyn_obs', 1).value
        self.num_static_obj = self.declare_parameter('num_static_obs', 1).value
        self.static_obs_size = self.declare_parameter('static_obs_size', 0.2).value
        self.static_obs_topic = self.declare_parameter('static_obs_topic', '/Obstacles/Static').value
        self.dyn_obs_topic = self.declare_parameter('dyn_obs_topic', '/Obstacles/Dynamic').value
        self.pub_rate = self.declare_parameter('pub_rate', 30).value
        self.static_obs_location = self.declare_parameter('static_obs_location', None).value

        if self.static_obs_location is not None:
            self.static_obs_msg = self.load_static_obs(self.static_obs_location)
        else:
            self.static_obs_msg = self.create_static_obs()

    def setup_publisher(self):
        """Sets up ROS publishers."""
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.static_obs_publisher = self.create_publisher(MarkerArray, self.static_obs_topic, qos_profile)
        self.dyn_obs_publisher = self.create_publisher(OdometryArray, self.dyn_obs_topic, qos_profile)

    def reconfigure_callback(self, params):
        """Handles dynamic parameter updates."""
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

    def simulation_thread(self):
        """Runs the main simulation loop."""
        self.get_logger().info("Entering simulation thread")
        dyn_obs_pose = {}

        for i in range(self.num_dyn_obj):
            pose = self.lanelet_map.get_random_waypoint()
            x, y, psi = pose
            state = np.array([x, y, 0, psi])
            ref_path, v_ref = self.gen_ref_path(pose)
            dyn_obs_pose[i] = {'state': state, 'path': ref_path, 'v_ref': v_ref}

        msg_queue = queue.Queue()
        rate = 1.0 / self.pub_rate

        while rclpy.ok():
            header = self.get_clock().now().to_msg()
            odom_array_msg = OdometryArray()
            odom_array_msg.header.stamp = header
            odom_array_msg.header.frame_id = 'map'

            for i in range(self.num_dyn_obj):
                state = dyn_obs_pose[i]['state']
                ref_path = dyn_obs_pose[i]['path']
                v_ref = dyn_obs_pose[i]['v_ref']

                _, _, s = ref_path.get_closest_pts(state[:2])

                if (1 - s) * ref_path.length < state[2] or np.random.uniform() < 0.03:
                    pose = np.array([state[0], state[1], state[3]])
                    ref_path, v_ref = self.gen_ref_path(pose)

                    dyn_obs_pose[i]['path'] = ref_path
                    dyn_obs_pose[i]['v_ref'] = v_ref
                    _, _, s = ref_path.get_closest_pts(state[:2])

                look_ahead, _ = ref_path.interp(s * ref_path.length + 0.5)
                look_ahead_x, look_ahead_y = look_ahead[0, 0], look_ahead[1, 0]

                alpha = np.arctan2(look_ahead_y - state[1], look_ahead_x - state[0]) - state[3]
                ld = np.sqrt((look_ahead_x - state[0])**2 + (look_ahead_y - state[1])**2)
                steer = np.arctan2(2 * 0.257 * np.sin(alpha), ld)
                throttle = (v_ref - state[2]) * 0.5

                state_next = self.dyn.integrate(state, np.array([throttle, steer]), self.sigma)
                dyn_obs_pose[i]['state'] = state_next

                odom_msg = Odometry()
                odom_msg.header = odom_array_msg.header
                odom_msg.pose.pose.position.x = state_next[0]
                odom_msg.pose.pose.position.y = state_next[1]
                q = quaternion_about_axis(state_next[3], (0, 0, 1))
                odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y = q[0], q[1]
                odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w = q[2], q[3]
                odom_msg.twist.twist.linear.x = state_next[2]

                odom_array_msg.odom_list.append(odom_msg)

            self.dyn_obs_publisher.publish(odom_array_msg)
            time.sleep(rate)

    def gen_ref_path(self, pose):
        """Generates a reference path for a dynamic obstacle."""
        path = None
        while path is None:
            goal = self.lanelet_map.get_random_waypoint()
            path = self.lanelet_map.get_shortest_path(pose, goal, True, True, verbose=False)
        v_ref = np.random.uniform(0, 1.0)
        return RefPath(path[:, :2].T), v_ref
