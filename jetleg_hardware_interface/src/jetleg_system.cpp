#include "jetleg_system/jetleg_system.hpp"

namespace jetleg_system
{
CallbackReturn JetlegSystem::on_init(const hardware_interface::HardwareInfo & /*info*/)
{
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
