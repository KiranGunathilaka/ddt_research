import os
from os.path import join
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def get_argument(context, arg):
    return LaunchConfiguration(arg).perform(context)

def spawn_car(context, *args, **kwargs):
    # Get the values of the arguments
    robot_name = get_argument(context, 'robot_name')
    
    # In hardware mode, some simulation-specific parameters are not needed
    # We'll use placeholders or fixed values where appropriate
    vehicle_model = 'DynamicBicycle' 
    command_mode = 'acceleration'
    vehicle_model_config = 'configDry.yaml'
    publish_tf = 'false'
    pub_ground_truth = 'false'
    simulate_perception = 'false'
    
    config_file = join(get_package_share_directory('eufs_racecar'), 'robots', robot_name, vehicle_model_config)
    noise_file = join(get_package_share_directory('eufs_models'), 'config', 'noise.yaml')
    recolor_config = join(get_package_share_directory('eufs_plugins'), 'config', 'cone_recolor.yaml')
    bounding_boxes_file = join(get_package_share_directory('eufs_plugins'), 'config', 'boundingBoxes.yaml')
    
    xacro_path = join(get_package_share_directory('eufs_racecar'), 'robots', robot_name, 'robot.urdf.xacro')
    urdf_path = join(get_package_share_directory('eufs_racecar'), 'robots', robot_name, 'robot.urdf')

    if not os.path.isfile(urdf_path):
        open(urdf_path, 'w').close()

    doc = xacro.process_file(xacro_path, mappings={
        'robot_name': robot_name,
        'vehicle_model': vehicle_model,
        'command_mode': command_mode,
        'config_file': config_file,
        'noise_config': noise_file, # Needed to avoid error
        'recolor_config': recolor_config, # Needed to avoid error
        'publish_tf': publish_tf, # Needed to avoid error
        'simulate_perception': simulate_perception, # Needed to avoid error
        'pub_ground_truth': pub_ground_truth, # Needed to avoid error
        'bounding_box_settings': bounding_boxes_file, # Needed to avoid error
    })
    
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    joint_state_publisher_node = Node(
        name='joint_state_publisher',
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'rate': 200,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        arguments=[urdf_path],
        remappings=[('/joint_states', '/eufs/joint_states')]
    )

    robot_state_publisher_node = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'rate': 200,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[('/joint_states', '/eufs/joint_states')],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'base_footprint']
    )

    return [
        joint_state_publisher_node,
        robot_state_publisher_node,
        static_tf
    ]

def generate_launch_description():

    #########################################################
    # Debugging and Visualizations
    #########################################################

    rviz_config_file = join(get_package_share_directory('eufs_launcher'), 'config', 'default.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation time (true) or real time (false)'
        ),
        DeclareLaunchArgument(
            name='robot_name',
            default_value='ads-dv',
            description='The name of the robot (e.g., ads-dv, eufs)'
        ),
        DeclareLaunchArgument(
            name='rviz',
            default_value='true',
            description='Launch RViz for visualization'
        ),
        DeclareLaunchArgument(
            name='bristol_fsai_debug',
            default_value='false',
            description='Enable Bristol FSAI debugging and visualizations'
        ),

        # Spawn the car
        OpaqueFunction(
            function=spawn_car,
            condition=IfCondition(LaunchConfiguration('bristol_fsai_debug'))
        ),
        # Launch RViz2 for visualization
        Node(
            name='rviz2',
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('bristol_fsai_debug'))
        ),

        #########################################################
        # Launch Bristol FSAI in hardware mode
        #########################################################

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                join(get_package_share_directory('eufs_launcher'), 'bristol_fsai.launch.py')
            ),
            launch_arguments={
                'bristol_fsai_hardware_mode': 'true',
                'bristol_fsai_debug': LaunchConfiguration('bristol_fsai_debug'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items()
        ),
    ])