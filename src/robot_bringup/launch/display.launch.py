import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():

    # --- Получаем путь к URDF-файлу ---
    robot_description_pkg_dir = get_package_share_directory('robot_description')
    urdf_file_path = os.path.join(robot_description_pkg_dir, 'urdf', 'park_robot.urdf')
    
    # --- Нода для публикации состояния робота из URDF ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file_path).read()}]
    )

    # --- Нода для запуска RViz с конфигурацией по умолчанию ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    ])
