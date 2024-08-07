cmake_minimum_required(VERSION 3.8)
project(jetleg_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Minimal dependencies for hardware interface
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(serial_interface REQUIRED)

find_package(PkgConfig REQUIRED)

pkg_check_modules(libserial REQUIRED)


# Add compile directive for hardware interface
add_library(jetleg_system SHARED src/jetleg_system.cpp)
target_include_directories(jetleg_system PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  ${serial_interface_INCLUDE_DIRS}
)
ament_target_dependencies(jetleg_system PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  serial_interface
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(jetleg_system PRIVATE "HARDWARE_INTERFACE_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  hardware_interface jetleg_system_plugin_description.xml
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Run unit tests
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_jetleg_system test/test_jetleg_system.cpp)
  target_include_directories(test_jetleg_system PRIVATE include)
  ament_target_dependencies(test_jetleg_system
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )

endif()

# Specify install directives for hardware interface
install(
  DIRECTORY include/
  DESTINATION include/
)
install(
  TARGETS jetleg_system
  EXPORT export_jetleg_system
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_export_libraries(
  jetleg_system
)
ament_export_targets(
  export_jetleg_system
)

ament_package()
