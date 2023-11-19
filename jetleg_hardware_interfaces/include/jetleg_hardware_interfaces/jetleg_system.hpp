// Copyright (c) 2023 Pack Bionics
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef JETLEG_HARDWARE_INTERFACES_HPP
#define JETLEG_HARDWARE_INTERFACES_HPP

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::return_type;

namespace jetleg_hardware_interfaces
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

static constexpr size_t POSITION_INTERFACE_INDEX = 0;
static constexpr size_t VELOCITY_INTERFACE_INDEX = 1;
static constexpr size_t ACCELERATION_INTERFACE_INDEX = 2;
static constexpr size_t EFFORT_INTERFACE_INDEX = 3;

class HARDWARE_INTERFACE_PUBLIC JetlegSystem : public hardware_interface::SystemInterface
{
public:

  /**
   * @brief Initializes the interface to the electronic system
   * 
   * Opens the connections to the main interface to the sensors and actuators
   * 
   * @param info stores information about the hardware as described in URDF
   * @return CallbackReturn denotes the success or failure of the initialization step
   */
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief Accesses the available state interfaces as described by URDF
   * 
   * @return std::vector<hardware_interface::StateInterface> references to available state interfaces
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief Accesses the available command interfaces as described by URDF
   * 
   * @return std::vector<hardware_interface::CommandInterface> references to available command interfaces
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief Prepares for switching to new mode of command interface
   * 
   * @param start_interfaces command interfaces which will be utilized
   * @param stop_interfaces command interfaces which will be unutiliized
   * @return return_type successful if the switch is valid or unsuccessful otherwise
   */

  /**
   * @brief Reads the state information from sensors
   * 
   * @param time time of the current control loop cycle
   * @param period time elapsed since previous control loop cycle
   * @return return_type successful if the read operation was successful or unsuccessful otherwise
   */
  return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

  /**
   * @brief Sends the desired commands to the actuators of the system
   * 
   * @param time time of the current control loop cycle
   * @param period time elapsed since previous control loop cycle
   * @return return_type successful if the write operation was successful or unsuccessful otherwise
   */
  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & period) override;

protected:
  
  /**
   * By splitting the standard interfaces from other type, the users are able to inherit this
   * class and simply create small "simulation" with desired dynamic behavior.
   * The advantage over using Gazebo is that enables "quick & dirty" tests of robot's URDF and
   * controllers.
   */
  const std::vector<std::string> standard_interfaces_ = {
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_ACCELERATION, hardware_interface::HW_IF_EFFORT};

  /// The size of this vector is (standard_interfaces_.size() x nr_joints)
  std::vector<std::vector<double>> joint_commands_;
  std::vector<std::vector<double>> joint_states_;

private:
  template <typename HandleType>
  bool get_interface(
    const std::string & name, const std::vector<std::string> & interface_list,
    const std::string & interface_name, const size_t vector_index,
    std::vector<std::vector<double>> & values, std::vector<HandleType> & interfaces);

  void initialize_storage_vectors(
    std::vector<std::vector<double>> & commands, std::vector<std::vector<double>> & states,
    const std::vector<std::string> & interfaces,
    const std::vector<hardware_interface::ComponentInfo> & component_infos);

  template <typename InterfaceType>
  bool populate_interfaces(
    const std::vector<hardware_interface::ComponentInfo> & components,
    std::vector<std::string> & interfaces, std::vector<std::vector<double>> & storage,
    std::vector<InterfaceType> & target_interfaces, bool using_state_interfaces);

  std::vector<size_t> joint_control_mode_;
};

}  // namespace jetleg_hardware_interfaces

#endif  // JETLEG_HARDWARE_INTERFACES_HPP