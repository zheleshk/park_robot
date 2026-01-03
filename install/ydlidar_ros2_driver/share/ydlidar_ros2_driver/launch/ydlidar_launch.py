#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Получаем путь к директории пакета
    pkg_dir = get_package_share_directory('ydlidar_ros2_driver')
    
    # Формируем полный путь к вашему файлу параметров
    params_yaml_file = os.path.join(pkg_dir, 'params', 'X4-Pro.yaml')

    # Создаем ноду драйвера лидара
    ydlidar_ros2_driver_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        # Указываем использовать наш файл параметров
        parameters=[params_yaml_file]
    )

    # Создаем ноду для публикации статической трансформации (TF)
    # Это связывает систему координат лидара ('laser_frame') с базовой системой робота ('base_link')
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0.02', '0', '0', '0', 'base_link', 'laser_frame']
    )

    # Возвращаем описание запуска, которое содержит обе ноды
    return LaunchDescription([
        ydlidar_ros2_driver_node,
        static_transform_publisher_node,
    ])
