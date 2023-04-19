cmake_minimum_required(VERSION 3.5)
project(jetleg_control)

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

find_package(rclpy REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python executables
install(PROGRAMS
  scripts/jetleg_gait_generator.py
  scripts/jetleg_gazebo_gait_generator.py
  scripts/jetleg_teleop_key.py
  scripts/leg_actor.py
  scripts/run_leg_agent.py
  scripts/train_leg_agent.py

  DESTINATION lib/${PROJECT_NAME}
)

ament_package()