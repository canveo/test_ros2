import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- argumentos genéricos (ros_bridge) -------------------------------
    host_arg = DeclareLaunchArgument("host", default_value="localhost")
    port_arg = DeclareLaunchArgument("port", default_value="2000")
    timeout_arg = DeclareLaunchArgument("timeout", default_value="10")
    town_arg = DeclareLaunchArgument("town", default_value="Town02")

    # Sincronización CARLA <-> ROS
    fixed_delta_arg = DeclareLaunchArgument(
        "fixed_delta_seconds", default_value="0.05",
        description="Delta de tiempo fijo (0.05s = 20Hz)"
    )
    sync_mode_arg = DeclareLaunchArgument(
        "synchronous_mode", default_value="False",
        description="Enable synchronous mode in CARLA"
    )
    sync_wait_arg = DeclareLaunchArgument(
        "synchronous_mode_wait_for_vehicle_control_command",
        default_value="False",
        description="Wait for vehicle control command in synchronous mode"
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="True",
        description="Use simulation clock from /clock topic"
    )

    # Spawn point del vehículo ego
    spawn_point_arg = DeclareLaunchArgument(
        "spawn_point_ego_vehicle",
        default_value="170.0,-105.3,0.42,0.00,0.00,180.00",  # Town02
        description="Pose inicial del vehículo (x,y,z,roll,pitch,yaw)",
    )

    # --- include: carla_ros_bridge ---------------------------------------
    carla_ros_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("carla_ros_bridge"),
                "carla_ros_bridge.launch.py",
            )
        ),
        launch_arguments={
            "host": LaunchConfiguration("host"),
            "port": LaunchConfiguration("port"),
            "town": LaunchConfiguration("town"),
            "timeout": LaunchConfiguration("timeout"),
            "synchronous_mode": LaunchConfiguration("synchronous_mode"),
            "fixed_delta_seconds": LaunchConfiguration("fixed_delta_seconds"),
            "synchronous_mode_wait_for_vehicle_control_command": LaunchConfiguration("synchronous_mode_wait_for_vehicle_control_command"),
            "Publish_vehicle_control": "False",   # deshabilita el control del vehículo desde ROS bridge
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    # --- include: spawnea el ego ----------------------------------------
    spawn_objects_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("carla_spawn_objects"),
                "carla_example_ego_vehicle.launch.py",
            )
        ),
        launch_arguments={
            "spawn_point_ego_vehicle": LaunchConfiguration("spawn_point_ego_vehicle"),
        }.items(),
    )

    # --- opcional: control manual ---------------------------------------
    role_name_arg = DeclareLaunchArgument("role_name", default_value="ego_vehicle")
    manual_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("carla_manual_control"),
                "carla_manual_control.launch.py",
            )
        ),
        launch_arguments={
            "role_name": LaunchConfiguration("role_name"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    return launch.LaunchDescription(
        [
            # argumentos
            host_arg,
            port_arg,
            timeout_arg,
            town_arg,
            fixed_delta_arg,
            sync_mode_arg,
            sync_wait_arg,
            use_sim_time_arg,
            spawn_point_arg,
            role_name_arg,

            # includes
            carla_ros_bridge_launch,
            spawn_objects_launch,
            manual_control_launch,
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
