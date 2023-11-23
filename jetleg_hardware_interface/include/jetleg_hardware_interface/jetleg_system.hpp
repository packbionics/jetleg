#ifndef JETLEG_SYSTEM_HPP
#define JETLEG_SYSTEM_HPP

#include "hardware_interface/system_interface.hpp"
#include "jetleg_hardware_interface/visibility_control.h"

namespace jetleg_hardware_interface
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class JetlegSystem : public hardware_interface::SystemInterface
{
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/) override;
};

}

#endif // JETLEG_SYSTEM_HPP
