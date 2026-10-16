import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'rover_map'

    package_share = get_package_share_directory(package_name)

    map_path = os.path.join(
        package_share,
        'maps',
        'map.pgm'
    )

    output_map_path = '/home/nadi/rover_ws/src/rover_map/maps/annotated_map.png'#-------------->
    
    map_file = os.path.join(
            get_package_share_directory('rover_map'),
            'maps',
            'map.yaml'
        )

    map_set=Node( 
        package="rover_map",
        executable="annotated_map_publisher",
        parameters=[
                {
                    'input_map_path': map_path,
                    'output_map_path': output_map_path,

                    'resolution': 0.025,
                    'origin_x': -0.5,
                    'origin_y': -0.5,
                }
            ],
        )
    #publish map
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_file}
        ]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {
                'autostart': True,
                'node_names': ['map_server']
            }
        ]
    )
    set_pos_map=Node(
        package="rover_map",
        executable="set_init_pos"
    )
    static_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_tf',
        arguments=[
            '--x', '19',
            '--y', '0',
            '--z', '0',
            '--yaw', '3.14159',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'odom',
            '--child-frame-id', 'map'
        ]
    )



    
    return LaunchDescription([

        map_server,
        lifecycle_manager,
        set_pos_map,
        static_map_tf,
        map_set,
        
    ])