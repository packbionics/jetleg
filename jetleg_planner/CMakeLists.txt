cmake_minimum_required(VERSION 3.8)
project(jetleg_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  moveit_ros_planning
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
)

set(DEPENDENCY_DEPS
  rclcpp
  std_srvs
  moveit_ros_planning
)

set(CONTROL_LIB
  control_helper
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(std_srvs REQUIRED)

find_package(generate_parameter_library REQUIRED)

generate_parameter_library(
  jetleg_planner_parameters
  src/jetleg_planner_parameters.yml
)

# Add library for generic finite state control
add_library(${CONTROL_LIB}
  src/finite_state_controller.cpp)
target_include_directories(${CONTROL_LIB}
  PUBLIC include
)

# Add library providing a ROS 2 interface to the
# generic finite state contorl library
add_library(
  control_node_lib
  src/finite_state_controller_node.cpp
)
target_include_directories(control_node_lib
  PUBLIC include
)
ament_target_dependencies(control_node_lib
    ${DEPENDENCY_DEPS})
target_link_libraries(control_node_lib
  ${CONTROL_LIB})

# Add executable for running an instance of the jetleg_planner
add_executable(move_group_interface_tutorial
    src/moveit_example.cpp)
target_include_directories(move_group_interface_tutorial
    PUBLIC include)
target_link_libraries(move_group_interface_tutorial
  control_node_lib
  jetleg_planner_parameters
  )
ament_target_dependencies(move_group_interface_tutorial
    ${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)

  # Find and set up default linters provided by ROS 2
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install artifacts into the appropriate directories
install(TARGETS move_group_interface_tutorial
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
