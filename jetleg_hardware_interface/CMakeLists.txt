cmake_minimum_required(VERSION 3.5)
project(jetleg_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    pluginlib
    rclcpp_lifecycle
    hardware_interface
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Mock components
add_library(
  mock_components
  SHARED
  src/mock_components/generic_system.cpp
)
target_include_directories(
  mock_components
  PUBLIC
  include
)
target_link_libraries(
  mock_components
)
ament_target_dependencies(
  mock_components
  pluginlib
  rcpputils
  hardware_interface
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(mock_components PRIVATE "HARDWARE_INTERFACE_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  hardware_interface mock_components_plugin_description.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  mock_components
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)

  # Find testing dependencies
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # Test the hardware interface
  ament_add_gmock(test_generic_system test/mock_components/test_generic_system.cpp)
    target_include_directories(test_generic_system PRIVATE include)
    ament_target_dependencies(test_generic_system
      pluginlib
      hardware_interface
      ros2_control_test_assets
    )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  mock_components
)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
