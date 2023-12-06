// Copyright 2023 Pack Bionics
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#include "jetleg_system/jetleg_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

namespace jetleg_system
{
CallbackReturn JetlegSystem::on_init(const hardware_interface::HardwareInfo & info)
{

  // Used to share status of hardware interface
  rclcpp::Logger logger = rclcpp::get_logger("JetlegSystem");

  // Delegate to base class to perform initial hardware setup
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Make sure each joint has at most one command interface
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() > 1) {

      // Record the error and return with non-successful status
      RCLCPP_ERROR(
        logger,
        "<%s> has multiple detected command interfaces. "
        "This is currently not supported.", joint.name.c_str()
      );
      return CallbackReturn::ERROR;
    }
  }

  // A list of interfaces created for each joint
  mJointStates.resize(info_.joints.size());

  // Each list contains the state information for the corresponding joint
  const int numStateInterfaces = 2;
  for (auto & joint : mJointStates) {
    joint.resize(numStateInterfaces);
  }

  RCLCPP_INFO(logger, "JetlegSystem hardware interface has been initialized.");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> JetlegSystem::export_state_interfaces()
{

  // Used to share status of hardware interface
  rclcpp::Logger logger = rclcpp::get_logger("JetlegSystem");

  // Shares joint states with the rest of ros2_control
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Possible supported state interface types
  // Note: Not all may be supported at this time
  const std::vector<std::string> standard_interfaces = {
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_ACCELERATION, hardware_interface::HW_IF_EFFORT
  };

  // Look over each joint in the robot
  int jointOffset = 0;
  for (const auto & joint : info_.joints) {

    // For each joint, add accessible joint state interfaces
    for (const auto & state_interface : joint.state_interfaces) {

      // Find state interface if it exists
      auto stateInterfacePtr = std::find(
        standard_interfaces.begin(),
        standard_interfaces.end(),
        state_interface.name
      );

      // Make an entry in list of StateInterfaces if found
      if (stateInterfacePtr != standard_interfaces.end()) {
        int interfaceOffset = std::distance(standard_interfaces.begin(), stateInterfacePtr);
        state_interfaces.emplace_back(
          joint.name, *stateInterfacePtr,
          &mJointStates[jointOffset][interfaceOffset]
        );
      } else {
        RCLCPP_ERROR(
          logger,
          "State interface <%s> not found. "
          "Please ensure the hardware is correctly described in URDF.",
          state_interface.name.c_str()
        );
      }
    }

    jointOffset++;
  }

  RCLCPP_INFO(logger, "JetlegSystem hardware interface has exported state interfaces.");
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> JetlegSystem::export_command_interfaces()
{

  // Used to share status of hardware interface
  rclcpp::Logger logger = rclcpp::get_logger("JetlegSystem");

  // Advertises command interfaces to the rest of ros2_control
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Possible supported state interface types
  // Note: Not all may be supported at this time
  const std::vector<std::string> standard_interfaces = {
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_ACCELERATION, hardware_interface::HW_IF_EFFORT
  };


  int jointOffset = 0;
  for (const auto & joint : info_.joints) {

    // Search for valid command interfaces to add to list of CommandInterfaces
    for (const auto & command_interface : joint.command_interfaces) {

      // Find state interface if it exists
      auto commandInterfacePtr = std::find(
        standard_interfaces.begin(),
        standard_interfaces.end(),
        command_interface.name
      );

      // Make an entry in list of StateInterfaces if found
      if (commandInterfacePtr != standard_interfaces.end()) {
        command_interfaces.emplace_back(
          joint.name, *commandInterfacePtr,
          &mJointCommands[jointOffset]
        );
      } else {
        RCLCPP_ERROR(
          logger,
          "Command interface <%s> not found. "
          "Please ensure the hardware is correctly described in URDF.",
          command_interface.name.c_str()
        );
      }

      jointOffset++;
    }
  }

  RCLCPP_INFO(logger, "JetlegSystem hardware interface has exported command interfaces.");
  return command_interfaces;
}

hardware_interface::return_type JetlegSystem::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type JetlegSystem::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(jetleg_system::JetlegSystem, hardware_interface::SystemInterface)
