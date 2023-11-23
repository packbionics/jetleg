#include "jetleg_hardware_interface/jetleg_system.hpp"

namespace jetleg_hardware_interface
{
CallbackReturn on_init(const hardware_interface::HardwareInfo & info)
{

}

std::vector<hardware_interface::StateInterface> export_state_interfaces()
{

}

std::vector<hardware_interface::CommandInterface> export_command_interfaces()
{

}

hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period)
{

}

hardware_interface::return_type write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{

}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(jetleg_hardware_interface::JetlegSystem, hardware_interface::SystemInterface)
