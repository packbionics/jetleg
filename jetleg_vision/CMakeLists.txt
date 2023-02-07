cmake_minimum_required(VERSION 3.5)
project(jetleg_vision)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(OpenCV REQUIRED )
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

add_executable(jetleg_pointcloud_proc src/jetleg_pointcloud_proc.hpp src/jetleg_pointcloud_proc.cpp src/main.cpp)
target_include_directories(jetleg_pointcloud_proc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(jetleg_pointcloud_proc
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
  "cv_bridge"
  "rclcpp"
)

install(TARGETS jetleg_pointcloud_proc
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python executables
install(PROGRAMS
  scripts/jetleg_naive_slam.py
  scripts/jetleg_pointcloud_proc.py

  DESTINATION lib/${PROJECT_NAME}
)

ament_package()