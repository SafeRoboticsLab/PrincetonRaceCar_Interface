import rclpy
from rclpy.node import Node

def get_ros_param(node: Node, param_name: str, default):
    '''
    Read a parameter from the ROS 2 parameter server. If the parameter does not exist, return the default value.
    
    Args:
        node: ROS 2 node instance
        param_name: string, name of the parameter
        default: default value

    Returns:
        Value of the parameter
    '''

    # 🔹 Got rid of `rospy.has_param(param_name)` because ROS 2 requires parameters to be explicitly declared before use.
    #    Instead, we check if the parameter has been declared using `node.has_parameter()`.
    if node.has_parameter(param_name):
        return node.get_parameter(param_name).value
    else:
        # 🔹 Got rid of `rospy.search_param(param_name)` because ROS 2 does not support global parameter searches.
        #    Unlike ROS 1, parameters are node-specific and must be explicitly declared.
        
        node.get_logger().warn(f"Parameter '{param_name}' not found, using default: {default}")
        return default
