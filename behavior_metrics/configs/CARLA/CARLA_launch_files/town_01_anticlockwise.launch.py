from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    host = DeclareLaunchArgument('host', default_value='localhost', description='Host for CARLA server')
    port = DeclareLaunchArgument('port', default_value='2000', description='Port for CARLA server')
    timeout = DeclareLaunchArgument('timeout', default_value='10', description='Timeout for CARLA connection')
    role_name = DeclareLaunchArgument('role_name', default_value='ego_vehicle', description='Role name for the vehicle')
    vehicle_filter = DeclareLaunchArgument('vehicle_filter', default_value='vehicle.*', description='Filter for vehicle spawning')
    spawn_point = DeclareLaunchArgument(
        'spawn_point',
        default_value='100.0, 2.0, 1.37, 0.0, 0.0, 180.0',
        description='Spawn point for the ego vehicle'
    )
    town = DeclareLaunchArgument('town', default_value='Town01', description='CARLA town to load')
    passive = DeclareLaunchArgument('passive', default_value='', description='Enable/disable passive mode')
    sync_mode = DeclareLaunchArgument(
        'synchronous_mode_wait_for_vehicle_control_command',
        default_value='False',
        description='Wait for vehicle control command in synchronous mode'
    )
    fixed_delta = DeclareLaunchArgument(
        'fixed_delta_seconds',
        default_value='0.05',
        description='Fixed delta time for simulation steps'
    )

    # Nodes
    carla_ros_bridge_node = Node(
        package='carla_ros_bridge',
        executable='carla_ros_bridge',
        name='carla_ros_bridge',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'timeout': LaunchConfiguration('timeout'),
            'town': [LaunchConfiguration('town'), TextSubstitution(text='_Opt')],  # 🔹 Corrección aquí
            'passive': LaunchConfiguration('passive'),
            'synchronous_mode_wait_for_vehicle_control_command': LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command'),
            'fixed_delta_seconds': LaunchConfiguration('fixed_delta_seconds'),
        }]
    )

    spawn_objects_node = Node(
        package='carla_spawn_objects',
        executable='carla_spawn_objects',
        name='spawn_objects',
        output='screen',
        parameters=[{
            'objects_definition_file': 'config/objects.json',
            'role_name': LaunchConfiguration('role_name'),
            'spawn_point_ego_vehicle': LaunchConfiguration('spawn_point'),
            'spawn_sensors_only': False,
        }]
    )

    manual_control_node = Node(
        package='carla_manual_control',
        executable='carla_manual_control',
        name='manual_control',
        output='screen',
        parameters=[{
            'role_name': LaunchConfiguration('role_name'),
        }]
    )

    # Launch description
    return LaunchDescription([
        host,
        port,
        timeout,
        role_name,
        vehicle_filter,
        spawn_point,
        town,
        passive,
        sync_mode,
        fixed_delta,
        carla_ros_bridge_node,
        spawn_objects_node,
        manual_control_node,
    ])
