cmake_minimum_required(VERSION 3.5)
project(jetleg_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/jetleg_gait_generator.py
    scripts/jetleg_gazebo_gait_generator.py
    scripts/jetleg_teleop_key.py

    DESTINATION lib/${PROJECT_NAME}
)

ament_package()