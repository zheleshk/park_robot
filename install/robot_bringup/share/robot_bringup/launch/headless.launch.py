import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Путь к нашему минимальному URDF
    urdf_file = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf',
        'park_robot.urdf'
    )

    # Читаем содержимое URDF
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    # Нода, которая публикует описание робота
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Нода RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    ])
