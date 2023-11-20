cmake_minimum_required(VERSION 3.8)
project(jetleg_hardware_interfaces)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(jetleg_system SHARED
  src/jetleg_system.cpp
)
target_compile_features(jetleg_system PUBLIC cxx_std_17)
target_include_directories(jetleg_system PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/jetleg_hardware_interfaces>
)
ament_target_dependencies(jetleg_system PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(jetleg_system PRIVATE "HARDWARE_INTERFACE_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  hardware_interface jetleg_system_plugin_description.xml)

if(BUILD_TESTING)
# Add unit tests here
endif()

install(
  DIRECTORY include/
  DESTINATION include/
)
install(
  TARGETS jetleg_system
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()