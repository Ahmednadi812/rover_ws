import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.launch_description_sources import PythonLaunchDescriptionSource 
from launch_ros.actions import Node



def generate_launch_description():
    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rover_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rover_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "true",
            "use_python": "False"
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rover_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )
    zigzag = Node(
        package="rover_planning",   # اسم الباكدج اللي فيه الـ zigzag_planner
        executable="zigzag_planner",
        name="zigzag_planner",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "base_frame": "base_footprint",
            "margin": 2.0,
            "lane_spacing": 1.0,
            "point_step": 0.1,
            "lanes_along_x": True,
        }],
    )


    # ==========================================================
    # Map
    # ==========================================================

    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rover_map"),
                "launch",
                "map_server.launch.py"
            )
        )
    )


    # ==========================================================
    # Localization / EKF
    # ==========================================================

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rover_localization"),
                "launch",
                "local_localization.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "True",
            "use_python": "False"
        }.items()
    )

    imu_driver_node = Node(
        package="rover_firmware",
        executable="mpu6050_driver.py"
    )
    
    return LaunchDescription([
        hardware_interface,
        controller,
        joystick,
        zigzag,
        map_launch,
        localization,
       # imu_driver_node,
    ])