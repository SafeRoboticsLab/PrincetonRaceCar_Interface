import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Fix for RoboStack library linkage issue
    ld_preload = SetEnvironmentVariable(name="LD_PRELOAD", value=os.path.join(os.environ.get("CONDA_PREFIX", ""), "lib/libyaml-cpp.so"))

    # Declare launch arguments (replacing <arg> from ROS 1)
    output_arg = DeclareLaunchArgument("output", default_value="screen", description="Display output to screen or log file")
    odom_topic_arg = DeclareLaunchArgument("odom_topic", default_value="/SLAM/Pose", description="ROS topic for the pose")
    control_topic_arg = DeclareLaunchArgument("control_topic", default_value="/Control", description="ROS topic for control input")
    enable_routing_arg = DeclareLaunchArgument("enable_routing", default_value="False", description="Launch routing map server")
    lane_change_cost_arg = DeclareLaunchArgument("lane_change_cost", default_value="2", description="Cost of lane change")

    # Visualization Node
    visualization_node = Node(
        package="racecar_interface",
        executable="visualization_node",
        name="visualization_node",
        output=LaunchConfiguration("output"),
        parameters=[{
            "odom_topic": LaunchConfiguration("odom_topic"),
            "control_topic": LaunchConfiguration("control_topic"),
        }]
    )

    # Include other launch files
    visualize_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            os.environ["ROS_PACKAGE_PATH"], "racecar_routing", "launch", "visualize_map.launch.py"
        )),
        launch_arguments={"output": LaunchConfiguration("output")}.items(),
    )

    routing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            os.environ["ROS_PACKAGE_PATH"], "racecar_routing", "launch", "routing.launch.py"
        )),
        launch_arguments={
            "output": LaunchConfiguration("output"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "lane_change_cost": LaunchConfiguration("lane_change_cost"),
            "click_goal": LaunchConfiguration("enable_routing"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_routing"))  # Conditional execution
    )

    # RViz2 Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", os.path.join(os.environ["ROS_PACKAGE_PATH"], "racecar_interface", "rviz", "truck.rviz")]
    )

    # RQT GUI Node
    rqt_gui_node = Node(
        package="rqt_gui",
        executable="rqt_gui",
        name="rqt_gui",
        arguments=["--perspective-file", os.path.join(os.environ["ROS_PACKAGE_PATH"], "racecar_interface", "rviz", "rqt_truck.perspective")]
    )

    return LaunchDescription([
        ld_preload,
        output_arg, odom_topic_arg, control_topic_arg, enable_routing_arg, lane_change_cost_arg,
        visualization_node, visualize_map_launch, routing_launch, rviz_node, rqt_gui_node
    ])
