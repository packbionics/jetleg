cmake_minimum_required(VERSION 3.8)
project(jetleg_planner)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
target_compile_options(${CONTROL_LIB} PRIVATE --coverage)
target_link_libraries(${CONTROL_LIB} -lgcov)

# Add library providing a ROS 2 interface to the
# generic finite state control library
add_library(
  control_node_lib
  src/finite_state_controller_service.cpp
)
target_include_directories(control_node_lib
  PUBLIC include
)
ament_target_dependencies(control_node_lib
    ${DEPENDENCY_DEPS})
target_link_libraries(control_node_lib
  ${CONTROL_LIB})

# Add executable for running an instance of the jetleg_planner
add_executable(${PROJECT_NAME}
    src/jetleg_planner.cpp)
target_include_directories(${PROJECT_NAME}
    PUBLIC include)
target_link_libraries(${PROJECT_NAME}
    control_node_lib
    jetleg_planner_parameters)
ament_target_dependencies(${PROJECT_NAME}
    ${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)

  # Find and set up default linters provided by ROS 2
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  # Add unit tests for Finite State Controller
  ament_add_gtest(finite_state_controller_test test/test_finite_state_controller.cpp)
  target_include_directories(finite_state_controller_test
    PUBLIC include
  )
  target_link_libraries(finite_state_controller_test ${CONTROL_LIB})

  # Add unit tests for Finite State Controller Service
  ament_add_gmock(finite_state_controller_service_test test/test_finite_state_controller_service.cpp)
  target_include_directories(finite_state_controller_service_test
    PUBLIC include)
  target_link_libraries(finite_state_controller_service_test
      control_node_lib)
  ament_target_dependencies(finite_state_controller_service_test
      ${THIS_PACKAGE_INCLUDE_DEPENDS})

endif()

# Install artifacts into the appropriate directories
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
