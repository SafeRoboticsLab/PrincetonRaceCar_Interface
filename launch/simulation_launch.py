#!/usr/bin/env python3
"""
This ROS2 launch file is a conversion of the original ROS1 XML launch file.
It sets an environment variable, declares launch arguments, and launches several nodes:
- The simulation and visualization nodes from the racecar_interface package.
- Included launch files from racecar_routing (visualize_map and routing).
- RViz2 and rqt_gui for visualization.
  
Detailed inline comments explain the changes made during migration.
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
    # Set environment variable for LD_PRELOAD.
    # This replicates the ROS1 <env> tag. The value is constructed using the CONDA_PREFIX
    # environment variable (via a substitution) concatenated with the library path.
    # ------------------------------------------------------------------------------------
    ld_preload_value = [EnvironmentVariable('CONDA_PREFIX'), '/lib/libyaml-cpp.so']

    # ------------------------------------------------------------------------------------
    # Declare launch arguments (replacing ROS1 <arg> tags).
    # These arguments allow users to override default parameters when launching.
    # ------------------------------------------------------------------------------------
    output_arg = DeclareLaunchArgument(
        'output',
        default_value='screen',
        description='Display output to screen or log file'
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/Simulation/Pose',
        description='ROS topic for the pose'
    )
    control_topic_arg = DeclareLaunchArgument(
        'control_topic',
        default_value='/Control',
        description='ROS topic for control input'
    )
    enable_routing_arg = DeclareLaunchArgument(
        'enable_routing',
        default_value='false',
        description='Click and generate the route'
    )
    lane_change_cost_arg = DeclareLaunchArgument(
        'lane_change_cost',
        default_value='1',
        description='Cost of lane change'
    )
    pub_rate_arg = DeclareLaunchArgument(
        'pub_rate',
        default_value='30',
        description='Rate at which to publish the pose'
    )
    init_x_arg = DeclareLaunchArgument(
        'init_x',
        default_value='0',
        description='Initial x position'
    )
    init_y_arg = DeclareLaunchArgument(
        'init_y',
        default_value='0',
        description='Initial y position'
    )
    init_yaw_arg = DeclareLaunchArgument(
        'init_yaw',
        default_value='0',
        description='Initial yaw position'
    )

    # ------------------------------------------------------------------------------------
    # Get package share directories.
    # These are used to locate included launch files and configuration files.
    # ------------------------------------------------------------------------------------
    racecar_interface_share = get_package_share_directory('racecar_interface')
    racecar_routing_share = get_package_share_directory('racecar_routing')

    # ------------------------------------------------------------------------------------
    # Define nodes for simulation and visualization.
    # In ROS2, nodes are launched using the Node action from launch_ros.
    # Parameters are provided as dictionaries using LaunchConfiguration substitutions.
    # ------------------------------------------------------------------------------------
    simulation_node = Node(
        package='racecar_interface',
        executable='simulation_node.py',  # Executable script (make sure it is executable)
        name='simulation_node',
        output=LaunchConfiguration('output'),
        parameters=[{
            'odom_topic': LaunchConfiguration('odom_topic'),
            'control_topic': LaunchConfiguration('control_topic'),
            'pub_rate': LaunchConfiguration('pub_rate'),
            'init_x': LaunchConfiguration('init_x'),
            'init_y': LaunchConfiguration('init_y'),
            'init_yaw': LaunchConfiguration('init_yaw')
        }]
    )

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
    # Include external launch files from racecar_routing.
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
    # Launch RViz2 for visualization.
    # Note: In ROS2, RViz is typically provided by the 'rviz2' package.
    # ------------------------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output=LaunchConfiguration('output'),
        arguments=['-d', os.path.join(racecar_interface_share, 'rviz', 'simulation.rviz')]
    )

    # ------------------------------------------------------------------------------------
    # Launch rqt_gui for GUI-based tools.
    # ------------------------------------------------------------------------------------
    rqt_gui_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='rqt_gui',
        output=LaunchConfiguration('output'),
        arguments=['--perspective-file', os.path.join(racecar_interface_share, 'rviz', 'rqt_sim.perspective')]
    )

    # ------------------------------------------------------------------------------------
    # Construct and return the launch description with all actions.
    # ------------------------------------------------------------------------------------
    return LaunchDescription([
        # Set environment variable for library linkage fix.
        SetEnvironmentVariable(name='LD_PRELOAD', value=ld_preload_value),

        # Declare launch arguments.
        output_arg,
        odom_topic_arg,
        control_topic_arg,
        enable_routing_arg,
        lane_change_cost_arg,
        pub_rate_arg,
        init_x_arg,
        init_y_arg,
        init_yaw_arg,

        # Launch simulation and visualization nodes.
        simulation_node,
        visualization_node,

        # Include external launch files.
        visualize_map_launch,
        routing_launch,

        # Launch RViz2 and rqt_gui.
        rviz_node,
        rqt_gui_node
    ])

if __name__ == '__main__':
    generate_launch_description()
