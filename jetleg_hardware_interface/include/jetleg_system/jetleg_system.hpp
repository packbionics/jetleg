#ifndef JETLEG_SYSTEM_HPP
#define JETLEG_SYSTEM_HPP

#include "hardware_interface/system_interface.hpp"
#include "jetleg_system/visibility_control.h"

namespace jetleg_system
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HARDWARE_INTERFACE_PUBLIC JetlegSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/) override;
private:

  std::vector<std::vector<double>> mJointStates;
  std::vector<double> mJointCommands;

};

}

#endif // JETLEG_SYSTEM_HPP
