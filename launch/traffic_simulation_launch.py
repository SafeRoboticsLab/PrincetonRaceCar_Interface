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
    odom_topic_arg = DeclareLaunchArgument("odom_topic", default_value="/Simulation/Pose", description="ROS topic for the pose")
    control_topic_arg = DeclareLaunchArgument("control_topic", default_value="/Control", description="ROS topic for control input")
    enable_routing_arg = DeclareLaunchArgument("enable_routing", default_value="false", description="Enable routing map server")
    lane_change_cost_arg = DeclareLaunchArgument("lane_change_cost", default_value="1", description="Cost of lane change")
    pub_rate_arg = DeclareLaunchArgument("pub_rate", default_value="30", description="Rate at which to publish the pose")
    init_x_arg = DeclareLaunchArgument("init_x", default_value="0", description="Initial x position")
    init_y_arg = DeclareLaunchArgument("init_y", default_value="0", description="Initial y position")
    init_yaw_arg = DeclareLaunchArgument("init_yaw", default_value="0", description="Initial yaw")
    num_dyn_obs_arg = DeclareLaunchArgument("num_dyn_obs", default_value="1", description="Number of dynamic obstacles")
    num_static_obs_arg = DeclareLaunchArgument("num_static_obs", default_value="1", description="Number of static obstacles")
    static_obs_size_arg = DeclareLaunchArgument("static_obs_size", default_value="0.2", description="Size of static obstacles")
    static_obs_topic_arg = DeclareLaunchArgument("static_obs_topic", default_value="/Obstacles/Static", description="ROS topic for static obstacles")
    dyn_obs_topic_arg = DeclareLaunchArgument("dyn_obs_topic", default_value="/Obstacles/Dynamic", description="ROS topic for dynamic obstacles")

    # Simulation Node
    simulation_node = Node(
        package="racecar_interface",
        executable="simulation_node",
        name="simulation_node",
        output=LaunchConfiguration("output"),
        parameters=[{
            "odom_topic": LaunchConfiguration("odom_topic"),
            "control_topic": LaunchConfiguration("control_topic"),
            "pub_rate": LaunchConfiguration("pub_rate"),
            "init_x": LaunchConfiguration("init_x"),
            "init_y": LaunchConfiguration("init_y"),
            "init_yaw": LaunchConfiguration("init_yaw"),
        }]
    )

    # Traffic Simulation Node
    traffic_simulation_node = Node(
        package="racecar_interface",
        executable="traffic_simulation_node",
        name="traffic_simulation_node",
        output=LaunchConfiguration("output"),
        parameters=[{
            "map_file": os.path.join(os.environ["ROS_PACKAGE_PATH"], "racecar_routing", "cfg", "track.pkl"),
            "num_dyn_obs": LaunchConfiguration("num_dyn_obs"),
            "num_static_obs": LaunchConfiguration("num_static_obs"),
            "static_obs_size": LaunchConfiguration("static_obs_size"),
            "static_obs_topic": LaunchConfiguration("static_obs_topic"),
            "dyn_obs_topic": LaunchConfiguration("dyn_obs_topic"),
            "pub_rate": LaunchConfiguration("pub_rate"),
        }]
    )

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
        arguments=["-d", os.path.join(os.environ["ROS_PACKAGE_PATH"], "racecar_interface", "rviz", "simulation.rviz")]
    )

    # RQT GUI Node
    rqt_gui_node = Node(
        package="rqt_gui",
        executable="rqt_gui",
        name="rqt_gui",
        arguments=["--perspective-file", os.path.join(os.environ["ROS_PACKAGE_PATH"], "racecar_interface", "rviz", "rqt_sim.perspective")]
    )

    return LaunchDescription([
        ld_preload,
        output_arg, odom_topic_arg, control_topic_arg, enable_routing_arg, lane_change_cost_arg,
        pub_rate_arg, init_x_arg, init_y_arg, init_yaw_arg, num_dyn_obs_arg, num_static_obs_arg,
        static_obs_size_arg, static_obs_topic_arg, dyn_obs_topic_arg,
        simulation_node, traffic_simulation_node, visualization_node, visualize_map_launch, routing_launch, rviz_node, rqt_gui_node
    ])
