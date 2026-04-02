from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='If "true", run in simulation mode.'
    )
    sim = LaunchConfiguration('sim')

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/dlio/odom_node/odom',
        description='Odometry topic for go2 navigation.'
    )
    odom_topic = LaunchConfiguration('odom_topic')

    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/realsense/depth/image_rect_raw',
        description='RealSense depth topic for go2 navigation.'
    )
    depth_topic = LaunchConfiguration('depth_topic')

    preprocess_model_arg = DeclareLaunchArgument(
        'preprocess_model',
        default_value=PathJoinSubstitution([
            FindPackageShare('rl_nav_controller'),
            'deployment_policies',
            'vae_realsense.onnx'
        ]),
        description='RealSense VAE model path.'
    )
    preprocess_model = LaunchConfiguration('preprocess_model')

    policy_model_arg = DeclareLaunchArgument(
        'policy_model',
        default_value=PathJoinSubstitution([
            FindPackageShare('rl_nav_controller'),
            'deployment_policies',
            'go2_nav_policy.onnx'
        ]),
        description='go2 policy model path.'
    )
    policy_model = LaunchConfiguration('policy_model')

    joy_node = Node(
        package='joy',
        executable='game_controller_node',
        name='joy_rsl',
        output='screen',
        remappings=[
            ('/joy', 'rsl_joy'),
            ('/joy_vel', 'rsl_joy_vel'),
        ],
        parameters=[{
            'autorepeat_rate': 50.0
        }]
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '0.32715', '0.0', '0.10',
            '0', '0', '0',
            'base', 'realsense_camera'
        ],
    )

    rl_navigation_node = Node(
        package='rl_nav_controller',
        executable='rl_nav_controller',
        name='rl_nav_controller_go2',
        output='screen',
        parameters=[{
            'use_sim_time': sim
        }],
        arguments=[
            PythonExpression([
                "'",
                sim,
                "' == 'true' and '--sim' or ''"
            ]),
            '--odom-topic', odom_topic,
            '--depth-topic', depth_topic,
            '--preprocess-model', preprocess_model,
            '--policy-model', policy_model,
            '--min-depth', '0.3',
            '--max-depth', '5.0',
            '--control-frequency', '5.0',
        ]
    )

    return LaunchDescription([
        sim_arg,
        odom_topic_arg,
        depth_topic_arg,
        preprocess_model_arg,
        policy_model_arg,
        joy_node,
        static_tf_node,
        rl_navigation_node,
    ])
