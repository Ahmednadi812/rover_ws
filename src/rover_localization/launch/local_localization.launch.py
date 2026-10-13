from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition

import os


def generate_launch_description():

    # ==========================================================
    # Arguments
    # ==========================================================

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )

    use_python = LaunchConfiguration(
        "use_python"
    )

    use_sim_time = LaunchConfiguration(
        "use_sim_time"
    )


    # ==========================================================
    # Static TF
    # base_footprint_ekf -> imu_link_ekf
    # ==========================================================

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",

        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0.103",

            "--qx", "0",
            "--qy", "0",
            "--qz", "0",
            "--qw", "1",

            "--frame-id",
            "base_footprint_ekf",

            "--child-frame-id",
            "imu_link_ekf",
        ],

        output="screen",
    )


    # ==========================================================
    # EKF
    # ==========================================================

    robot_localization = Node(
        package="robot_localization",

        executable="ekf_node",

        name="ekf_filter_node",

        output="screen",

        parameters=[
            os.path.join(
                get_package_share_directory(
                    "rover_localization"
                ),
                "config",
                "ekf.yaml"
            ),

            {
                "use_sim_time":
                    use_sim_time
            }
        ],
    )


    # ==========================================================
    # IMU Republisher - Python
    # ==========================================================

    imu_republisher_py = Node(
        package="rover_localization",

        executable="imu_republisher.py",

        condition=IfCondition(
            use_python
        ),

        parameters=[
            {
                "use_sim_time":
                    use_sim_time
            }
        ],
    )


    # ==========================================================
    # IMU Republisher - C++
    # ==========================================================

    imu_republisher_cpp = Node(
        package="rover_localization",

        executable="imu_republisher",

        condition=UnlessCondition(
            use_python
        ),

        parameters=[
            {
                "use_sim_time":
                    use_sim_time
            }
        ],
    )


    # ==========================================================
    # Launch
    # ==========================================================

    return LaunchDescription([
        use_python_arg,
        use_sim_time_arg,

        static_transform_publisher,

        robot_localization,

        imu_republisher_py,
        imu_republisher_cpp,
    ])