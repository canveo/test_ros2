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
    fixed_delta_arg = DeclareLaunchArgument("fixed_delta_seconds", default_value="0.05") # 20 Hz
    sync_wait_arg = DeclareLaunchArgument(
        "synchronous_mode_wait_for_vehicle_control_command",
        default_value="True",
        description="Wait for vehicle control command in synchronous mode",
    )

    spawn_point_arg = DeclareLaunchArgument(
        "spawn_point_ego_vehicle",
        # default_value="142.91,-215.42,1.37,0.0,0.0,0.0", # town03
        # default_value="171.6,-105.3,0.42,0.00,0.00,180.00",  # town02
        default_value=" ", # for random choise
        description="Pose inicial del vehículo (x,y,z,roll,pitch,yaw)",
    )

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
            # habilita el modo síncrono a 20 Hz
            "synchronous_mode": "True",
            "fixed_delta_seconds": LaunchConfiguration("fixed_delta_seconds"),
            "synchronous_mode_wait_for_vehicle_control_command": "True",
            "Publish_vehicle_control": "False",  # deshabilita el control del vehículo
        }.items(),
    )

    # --- include que spawnea el ego  -------------------------------------
    spawn_objects_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("carla_spawn_objects"),
                "carla_example_ego_vehicle.launch.py",
            )
        ),
        launch_arguments={
            # "objects_definition_file": LaunchConfiguration("objects_definition_file"),
            "spawn_point_ego_vehicle": LaunchConfiguration("spawn_point_ego_vehicle"),   
            # "spawn_sensors_only": LaunchConfiguration("spawn_sensors_only"),
        }.items(),
    )

    # # --- opcional: control manual (mantiene role_name) -------------------
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
            sync_wait_arg,
            # objects_file_arg,
            spawn_point_arg,
            # spawn_only_sensors_arg,
            # use_sim_time_arg,
            # role_name_arg,
            # nodos / includes
            carla_ros_bridge_launch,
            spawn_objects_launch,
            manual_control_launch,
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
