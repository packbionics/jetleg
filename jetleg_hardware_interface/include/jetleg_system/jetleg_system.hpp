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


#ifndef JETLEG_SYSTEM_HPP
#define JETLEG_SYSTEM_HPP


#include "hardware_interface/system_interface.hpp"
#include "jetleg_system/visibility_control.h"
#include "serial_interface/libserial_bridge.hpp"

namespace jetleg_system
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief Provides an interface for communicating with the Jetleg prosthetic leg system
 *
 * This class describes an interface for regulating the communication between low-level
 * controller implemented with ros2_control and the electronic components of the
 * Jetleg prosthetic device for users with transfemoral amputation. The available
 * state information shall be angular position and velocity of the knee (and ankle
 * if available). The command given to the device will be assumed to be torque
 */
class HARDWARE_INTERFACE_PUBLIC JetlegSystem : public hardware_interface::SystemInterface
{
public:
  /**
   * @brief Performs basic error-checking and allocates memory for joint state and commands
   *
   * @param info structured hardware description as given by URDF
   * @return CallbackReturn with value CallbackReturn::OK if no error were encountered
   *          or CallbackReturn::ERROR otherwise
   */
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief Advertises available state interfaces to the rest of ros2_control
   *
   * @return std::vector<hardware_interface::StateInterface> with entries associated
   *          with maintained joint states to share with the rest of ros2_control
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief Advertises available command interfaces to the rest of ros2_control
   *
   * @return std::vector<hardware_interface::CommandInterface> with entries associated
   *          with current commands to send to system
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief Updates the maintained joint states
   *
   * The current record of joint states shall be updated based on
   * the received signals from the system. These updated values
   * will be used by the rest of ros2_control
   *
   * @param time current record of time
   * @param period Time elapsed since last control cycle
   * @return hardware_interface::return_type with value OK if successful or
   *          ERROR otherwise
   */
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  /**
   * @brief Updates the current command signals to send to the system
   *
   * @return hardware_interface::return_type with value OK if successful or
   *          ERROR otherwise
   */
  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/) override;

private:
  /** Maintains record of current joint states */
  std::vector<std::vector<double>> mJointStates;

  /** Maintains a reference to the interface to the arduino*/
  std::shared_ptr<serial::LibSerialBridge> serialBridgePointer;

  /** Maintains current commands sent to the system */
  std::vector<double> mJointCommands;

  /** Baud Rate for serial port/stream IO */
  const int BaudRate = 1;
  

};

}

#endif // JETLEG_SYSTEM_HPP
