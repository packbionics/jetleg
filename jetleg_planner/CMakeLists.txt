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

add_library(${CONTROL_LIB}
  src/finite_state_controller.cpp)
target_include_directories(${CONTROL_LIB}
  PUBLIC include
)

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

add_executable(move_group_interface_tutorial
    src/moveit_example.cpp)
target_include_directories(move_group_interface_tutorial
    PUBLIC include)
target_link_libraries(move_group_interface_tutorial
  control_node_lib)
ament_target_dependencies(move_group_interface_tutorial
    ${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS move_group_interface_tutorial
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
