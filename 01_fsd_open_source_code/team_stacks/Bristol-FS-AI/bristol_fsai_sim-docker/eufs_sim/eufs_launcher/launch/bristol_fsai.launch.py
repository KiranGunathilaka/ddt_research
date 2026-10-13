from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    return LaunchDescription([

        #########################################################
        # Bristol FSAI Launch Arguments
        #########################################################

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            name='bristol_fsai_debug',
            default_value='false',
            description='Enable Bristol FSAI debugging and visualizations'
        ),
        DeclareLaunchArgument(
            name='bristol_fsai_hardware_mode',
            default_value='true',
            description='Run in hardware mode'
        ),
        # ROS CAN Launch Arguments
        DeclareLaunchArgument(
            name='can_debug',
            default_value='0',
            description='Enable ROS CAN debugging and visualizations'
        ),
        DeclareLaunchArgument(
            name='simulate_can',
            default_value='0', # change to 1 when simulating CAN
            description='Simulate CAN if true'
        ),
        DeclareLaunchArgument(
            name='can_interface',
            default_value='can0', # change to vcan0 when simulating CAN
            description='CAN interface to use'
        ),

        #########################################################
        # Bristol FSAI Nodes
        #########################################################

        # Perception
        Node(
            name='YOLONode',
            package='perception',
            executable='YOLONode',
            parameters=[
                {'bristol_fsai_hardware_mode': LaunchConfiguration('bristol_fsai_hardware_mode')},
                {'bristol_fsai_debug': LaunchConfiguration('bristol_fsai_debug')},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),
        Node(
            name='ConePositionEstimator',
            package='perception',
            executable='ConePositionEstimator',
            parameters=[
                {'bristol_fsai_hardware_mode': LaunchConfiguration('bristol_fsai_hardware_mode')},
                {'bristol_fsai_debug': LaunchConfiguration('bristol_fsai_debug')},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),

        # Control
        Node(
            name='control_cpp',
            package='control_cpp',
            executable='control_cpp',
            parameters=[
                {'bristol_fsai_debug': LaunchConfiguration('bristol_fsai_debug')},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),
        # Launch odometry_to_twist_converter node if not in hardware mode
        Node(
            name='odometry_to_twist_converter',
            package='control_cpp',
            executable='odometry_to_twist_converter',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=UnlessCondition(LaunchConfiguration('bristol_fsai_hardware_mode'))
        ),

        # Launch CAN interface node if in hardware mode
        Node(
            name='ros_can',
            package='ros_can',
            executable='ros_can_node',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}, 
                {'can_debug': LaunchConfiguration('can_debug')},
                {'simulate_can': LaunchConfiguration('simulate_can')},
                {'can_interface': LaunchConfiguration('can_interface')}],
            condition=IfCondition(LaunchConfiguration('bristol_fsai_hardware_mode'))
        )
    ])