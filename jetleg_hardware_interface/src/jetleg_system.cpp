#include "jetleg_system/jetleg_system.hpp"

namespace jetleg_system
{
CallbackReturn JetlegSystem::on_init(const hardware_interface::HardwareInfo & info)
{

  // Delegate to base class to perform initial hardware setup
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // A list of interfaces created for each joint
  mJointStates.resize(info_.joints.size());

  // Each list contains the state information for the corresponding joint
  const int numStateInterfaces = 2;
  for(auto & joint : mJointStates) {
    joint.resize(numStateInterfaces);
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> JetlegSystem::export_state_interfaces()
{
  return {};
}

std::vector<hardware_interface::CommandInterface> JetlegSystem::export_command_interfaces()
{
  return {};
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
