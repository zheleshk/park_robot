import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Запуск драйвера лидара
    # Используем твой конфиг X4-Pro
    ydlidar_pkg_dir = get_package_share_directory('ydlidar_ros2_driver')
    # Укажи тут имя файла конфига, который мы правили (ydlidar.yaml или X4-Pro.yaml)
    ydlidar_params_file = os.path.join(ydlidar_pkg_dir, 'params', 'ydlidar.yaml')

    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[ydlidar_params_file]
    )

    # 2. Запуск моста с Arduino (Одометрия)
    arduino_node = Node(
        package='robot_control',
        executable='arduino_bridge',
        name='arduino_bridge',
        output='screen'
    )
    
    # 3. Запуск TF (Связывает лидар с роботом)
    # Это то самое звено, которого не хватает!
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        # x y z yaw pitch roll parent child
        arguments=['0.11', '0', '0.02', '3.14', '0', '0', 'base_link', 'laser_frame'],
    )

    return LaunchDescription([
        ydlidar_node,
        arduino_node,
        tf_node
    ])
