import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    # ==========================================================
    # Gazebo
    # ==========================================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rover_description"),
                "launch",
                "gazebo.launch.py"
            )
        )
    )


    # ==========================================================
    # Controller
    # ==========================================================

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rover_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={
            "use_simple_controller": "true",
            "use_python": "False"
        }.items(),
    )


    # ==========================================================
    # Joystick
    # ==========================================================

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rover_controller"),
                "launch",
                "joystick_teleop.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
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

    rviz_config = os.path.join(
        get_package_share_directory("rover_map"),
        "rviz",
        "simulation.rviz"
    )


# rviz config file path


    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz_config
        ],
        parameters=[
            {
                "use_sim_time": True
            }
        ]
    )
    zigzag = Node(
        package="rover_planning",   # اسم الباكدج اللي فيه الـ zigzag_planner
        executable="zigzag_planner",
        name="zigzag_planner",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "base_frame": "base_footprint",
            "margin": 0.5,
            "lane_spacing": 1.0,
            "point_step": 0.1,
            "lanes_along_x": True,
        }],
    )
    # ==========================================================
    # Launch Everything
    # ==========================================================

    return LaunchDescription([

        gazebo,

        controller,

        joystick,

        map_launch,

        localization,

        rviz,
        
        zigzag,
    ])