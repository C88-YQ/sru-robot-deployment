from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    controller_params_default = PathJoinSubstitution([
        FindPackageShare("go2_controllers"),
        "config",
        "go2_controllers.yaml",
    ])
    bridge_config_default = PathJoinSubstitution([
        FindPackageShare("b2w_gazebo_ros2"),
        "config",
        "go2_gz_bridge.yaml",
    ])

    declared_arguments = [
        DeclareLaunchArgument(
            "robot_description_package",
            default_value="go2_description",
            description="Package that provides the robot description launch file",
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value="go2",
            description="Entity name used when spawning the robot in Gazebo",
        ),
        DeclareLaunchArgument(
            "bridge_config_file",
            default_value=bridge_config_default,
            description="Path to the ros_gz bridge configuration file",
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value="ISAACLAB_TRAIN.world",
            description="World file to load from b2w_sim_worlds/worlds",
        ),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="2.5"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("paused", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("debug", default_value="false"),
        DeclareLaunchArgument("verbose", default_value="false"),
        DeclareLaunchArgument("run_gui", default_value="true"),
        DeclareLaunchArgument(
            "controller_params_file",
            default_value=controller_params_default,
            description="Path to the controller parameter file",
        ),
        DeclareLaunchArgument(
            "controller_environment_profile",
            default_value="default",
            description="Environment profile override for b2w_controllers",
        ),
        DeclareLaunchArgument(
            "controller_policy_path",
            default_value="",
            description="Optional absolute path to override the policy file",
        ),
        DeclareLaunchArgument(
            "enable_low_level_controller",
            default_value="false",
            description="Enable the low-level gazebo controller node",
        ),
        DeclareLaunchArgument(
            "enable_mesh_publisher",
            default_value="false",
            description="Publish Gazebo meshes as RViz markers",
        ),
        DeclareLaunchArgument(
            "enable_set_pose_bridge",
            default_value="false",
            description="Bridge the Gazebo set_pose service via ros_gz_bridge",
        ),
        DeclareLaunchArgument(
            "enable_rviz",
            default_value="false",
            description="Launch RViz with the demo configuration",
        ),
        DeclareLaunchArgument(
            "collision_torque_threshold",
            default_value="30.0",
            description="Combined torque threshold used by collision_monitor",
        ),
    ]

    enable_low_level_controller = LaunchConfiguration("enable_low_level_controller")
    enable_mesh_publisher = LaunchConfiguration("enable_mesh_publisher")
    enable_set_pose_bridge = LaunchConfiguration("enable_set_pose_bridge")
    enable_rviz = LaunchConfiguration("enable_rviz")

    # Include the main gazebo launch file with spawn position arguments
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("b2w_gazebo_ros2"),
                "launch",
                "gazebo.launch.py"
            ])
        ),
        launch_arguments={
            "robot_description_package": LaunchConfiguration("robot_description_package"),
            "robot_name": LaunchConfiguration("robot_name"),
            "bridge_config_file": LaunchConfiguration("bridge_config_file"),
            "world_file": LaunchConfiguration("world_file"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "yaw": LaunchConfiguration("yaw"),
            "paused": LaunchConfiguration("paused"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "debug": LaunchConfiguration("debug"),
            "verbose": LaunchConfiguration("verbose"),
            "run_gui": LaunchConfiguration("run_gui"),
        }.items(),
    )
    
    static_tf_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="log",
    )

    # Kinematics controller node
    kinematics_controller = Node(
        package="go2_controllers",
        executable="go2_controllers",
        name="go2_controllers",
        output="screen",
        parameters=[
            LaunchConfiguration("controller_params_file"),
            {
                "environment.profile": LaunchConfiguration("controller_environment_profile"),
                "policy.path": LaunchConfiguration("controller_policy_path"),
            },
        ],
    )

    # Low-level gazebo controller node
    low_level_controller = Node(
        package="b2w_low_level_controller_gazebo",
        executable="b2w_low_level_controller_gazebo",
        name="b2w_low_level_controller_gazebo",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(enable_low_level_controller),
    )
    # mesh publisher node
    mesh_publisher = Node(
        package="b2w_sim_worlds",
        executable="publish_mesh_node",
        name="publish_mesh_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(enable_mesh_publisher),
    )

    # Service bridge for set_pose service
    service_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="set_pose_service_bridge",
        arguments=[
            PythonExpression([
                "'/world/' + '",
                LaunchConfiguration("world_file"),
                "'.replace('.world', '') + '_world/set_pose@ros_gz_interfaces/srv/SetEntityPose'"
            ])
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(enable_set_pose_bridge),
    )

    # Collision monitor node
    collision_monitor = Node(
        package="b2w_collision_monitor",
        executable="b2w_collision_detector",
        name="collision_detector",
        output="screen",
        parameters=[
            {"torque_threshold": LaunchConfiguration("collision_torque_threshold")},
            {"cooldown_period_ms": 5000},
            {"enable_force_torque": True},
            {"enable_pointcloud": False},
            {"use_sim_time": True},
        ],
    )

    # Add rviz2 node
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(enable_rviz),
    )

    return LaunchDescription(declared_arguments + [
        gazebo_launch,
        kinematics_controller,
        low_level_controller,
        mesh_publisher,
        service_bridge,
        collision_monitor,
        rviz2,
        static_tf_map_to_odom,
    ])
