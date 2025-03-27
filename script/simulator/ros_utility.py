# ros_utility.py (ROS 2)
# Utility function for retrieving parameters from the ROS 2 parameter server with fallback to default values.

import rclpy
from rclpy.node import Node

def get_ros_param(node: Node, param_name: str, default):
    '''
    Read a parameter from the ROS 2 parameter server. If the parameter does not exist, return the default value.
    
    Args:
        node: the current ROS 2 node instance (required for accessing parameters in ROS 2)
        param_name: string, name of the parameter
        default: default value

    Return:
        value of the parameter
    '''

    # In ROS 1, parameters were stored in a global parameter server that could be searched and accessed with rospy.get_param.
    # In ROS 2, parameters are strictly scoped to a specific node, and you must use the node instance to access them.

    # Check if the parameter exists (has been declared or set externally)
    if node.has_parameter(param_name):
        # Return its value using the rclpy parameter interface
        return node.get_parameter(param_name).get_parameter_value().value
    else:
        # If not found, we log a warning and return the default.
        # This replaces both rospy.logwarn and the rospy.search_param fallback mechanism from ROS 1.
        node.get_logger().warn(f"Parameter '{param_name}' not found, using default: {default}")
        return default

    # Note: Unlike ROS 1, there is no parameter search mechanism (e.g., search_param) in ROS 2.
    # Nodes are expected to explicitly declare the parameters they use, and their names are not auto-resolved across namespaces.
