"""courier.launch.py - one command for the full AcadBot Courier demo.

    ros2 launch acadbot_courier courier.launch.py
    ros2 launch acadbot_courier courier.launch.py headless:=true     # no Gazebo GUI
    ros2 launch acadbot_courier courier.launch.py rviz:=false       # skip RViz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('acadbot_bringup')
    pkg_courier = get_package_share_directory('acadbot_courier')

    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'autonomy.launch.py')
        ),
        launch_arguments={
            'localization': LaunchConfiguration('localization'),
            'nav2_delay': LaunchConfiguration('nav2_delay'),
            'headless': LaunchConfiguration('headless'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    courier = Node(
        package='acadbot_courier',
        executable='courier_server',
        name='courier_server',
        output='screen',
        parameters=[os.path.join(pkg_courier, 'config', 'courier.yaml')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('localization', default_value='amcl'),
        DeclareLaunchArgument('nav2_delay', default_value='12.0'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        autonomy,
        courier,
    ])