# CMake generated Testfile for 
# Source directory: /home/kirill/folders/lidar/YDLidar-SDK/python
# Build directory: /home/kirill/folders/lidar/YDLidar-SDK/ydlidar_ros2_ws/python
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(ydlidar_py_test "/usr/bin/python3" "/home/kirill/folders/lidar/YDLidar-SDK/python/test/pytest.py")
set_tests_properties(ydlidar_py_test PROPERTIES  ENVIRONMENT "PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:/home/kirill/folders/lidar/YDLidar-SDK/ydlidar_ros2_ws/python" _BACKTRACE_TRIPLES "/home/kirill/folders/lidar/YDLidar-SDK/python/CMakeLists.txt;42;add_test;/home/kirill/folders/lidar/YDLidar-SDK/python/CMakeLists.txt;0;")
subdirs("examples")
