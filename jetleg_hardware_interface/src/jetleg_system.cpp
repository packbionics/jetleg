#include "jetleg_system/jetleg_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

namespace jetleg_system
{
CallbackReturn JetlegSystem::on_init(const hardware_interface::HardwareInfo & info)
{

  rclcpp::Logger logger = rclcpp::get_logger("JetlegSystem");

  // Delegate to base class to perform initial hardware setup
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Make sure each joint has at most one command interface
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() > 1) {
      RCLCPP_ERROR(
        logger,
        "<%s> has multiple detected command interfaces. "
        "This is currently not supported.", joint.name.c_str()
      );
      return CallbackReturn::ERROR;

      // throw std::runtime_error(
      //         "<" + joint.name + "> has multiple detected command interfaces. "
      //         "This is currently not supported."
      // );
    }
  }

  // A list of interfaces created for each joint
  mJointStates.resize(info_.joints.size());

  // Each list contains the state information for the corresponding joint
  const int numStateInterfaces = 2;
  for (auto & joint : mJointStates) {
    joint.resize(numStateInterfaces);
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> JetlegSystem::export_state_interfaces()
{

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
        throw std::runtime_error(
                "State interface " + state_interface.name + "not found. "
                "Please ensure the hardware is correctly described in URDF."
        );
      }
    }

    jointOffset++;
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> JetlegSystem::export_command_interfaces()
{

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
        throw std::runtime_error(
                "State interface " + command_interface.name + "not found. "
                "Please ensure the hardware is correctly described in URDF."
        );
      }

      jointOffset++;
    }
  }

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
