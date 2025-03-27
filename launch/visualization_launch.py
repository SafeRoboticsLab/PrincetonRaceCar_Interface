#!/usr/bin/env python3
"""
This ROS2 launch file is a conversion of the original ROS1 visualization.launch file.
It sets an environment variable, declares launch arguments, launches RViz2, rqt_gui,
and the visualization_node from the racecar_interface package, and includes external
launch files from the racecar_routing package.

Detailed inline comments explain each change made during the migration.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ------------------------------------------------------------------------------------
    # Set environment variable for library linkage fix (as in RoboStack).
    # This replicates the ROS1 <env> tag.
    # ------------------------------------------------------------------------------------
    ld_preload_value = [EnvironmentVariable('CONDA_PREFIX'), '/lib/libyaml-cpp.so']

    # ------------------------------------------------------------------------------------
    # Declare launch arguments (replacing the ROS1 <arg> tags).
    # These allow users to override default settings.
    # ------------------------------------------------------------------------------------
    output_arg = DeclareLaunchArgument(
        'output',
        default_value='screen',
        description='Display output to screen or log file'
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/SLAM/Pose',
        description='ROS topic for the pose'
    )
    control_topic_arg = DeclareLaunchArgument(
        'control_topic',
        default_value='/Control',
        description='ROS topic for control input'
    )
    enable_routing_arg = DeclareLaunchArgument(
        'enable_routing',
        default_value='False',
        description='Launch routing map server'
    )
    lane_change_cost_arg = DeclareLaunchArgument(
        'lane_change_cost',
        default_value='2',
        description='Cost of lane change'
    )

    # ------------------------------------------------------------------------------------
    # Get package share directories to locate resources and included launch files.
    # ------------------------------------------------------------------------------------
    racecar_interface_share = get_package_share_directory('racecar_interface')
    racecar_routing_share = get_package_share_directory('racecar_routing')

    # ------------------------------------------------------------------------------------
    # Define the RViz2 node.
    # Note: In ROS2, RViz is typically provided by the 'rviz2' package.
    # ------------------------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output=LaunchConfiguration('output'),
        arguments=['-d', os.path.join(racecar_interface_share, 'rviz', 'truck.rviz')]
    )

    # ------------------------------------------------------------------------------------
    # Define the rqt_gui node.
    # ------------------------------------------------------------------------------------
    rqt_gui_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='rqt_gui',
        output=LaunchConfiguration('output'),
        arguments=['--perspective-file', os.path.join(racecar_interface_share, 'rviz', 'rqt_truck.perspective')]
    )

    # ------------------------------------------------------------------------------------
    # Define the visualization_node from the racecar_interface package.
    # Parameters are passed as a dictionary using LaunchConfiguration substitutions.
    # ------------------------------------------------------------------------------------
    visualization_node = Node(
        package='racecar_interface',
        executable='visualization_node.py',
        name='visualization_node',
        output=LaunchConfiguration('output'),
        parameters=[{
            'odom_topic': LaunchConfiguration('odom_topic'),
            'control_topic': LaunchConfiguration('control_topic')
        }]
    )

    # ------------------------------------------------------------------------------------
    # Include external launch files from the racecar_routing package.
    # These assume the original XML files have been converted to ROS2 Python launch files.
    # ------------------------------------------------------------------------------------
    visualize_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(racecar_routing_share, 'launch', 'visualize_map.launch.py')
        ),
        launch_arguments={'output': LaunchConfiguration('output')}.items()
    )

    routing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(racecar_routing_share, 'launch', 'routing.launch.py')
        ),
        launch_arguments={
            'output': LaunchConfiguration('output'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'lane_change_cost': LaunchConfiguration('lane_change_cost'),
            'click_goal': LaunchConfiguration('enable_routing')
        }.items()
    )

    # ------------------------------------------------------------------------------------
    # Construct and return the launch description with all actions.
    # ------------------------------------------------------------------------------------
    return LaunchDescription([
        # Set the environment variable.
        SetEnvironmentVariable(name='LD_PRELOAD', value=ld_preload_value),

        # Declare launch arguments.
        output_arg,
        odom_topic_arg,
        control_topic_arg,
        enable_routing_arg,
        lane_change_cost_arg,

        # Launch nodes.
        rviz_node,
        rqt_gui_node,
        visualization_node,

        # Include external launch files.
        visualize_map_launch,
        routing_launch
    ])

if __name__ == '__main__':
    generate_launch_description()
