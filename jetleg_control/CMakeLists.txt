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
find_package(generate_parameter_library REQUIRED)

ament_python_install_package(${PROJECT_NAME})

generate_parameter_library(
  classifier_parameters # cmake target name for the parameter library
  src/classifier_parameters.yaml # path to input yaml file
)

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/jetleg_gait_generator.py
    scripts/jetleg_teleop_key.py
    scripts/forwarder.py
    scripts/impedance_controller.py
    scripts/send_impedance_update.py
    scripts/rule_based_classifier.py

    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests
    test/test_gait_mode.py
    test/test_rule_list.py
    test/test_rule_based_classifier.py
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()
endif()

ament_package()