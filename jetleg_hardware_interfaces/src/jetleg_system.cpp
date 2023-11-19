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

#include "jetleg_hardware_interfaces/jetleg_system.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>
#include <string>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rcutils/logging_macros.h"

namespace jetleg_hardware_interfaces
{

// Reads a double floating point value from string
double parse_double(const std::string & text)
{
  double result_value;
  const auto parse_result = std::from_chars(text.data(), text.data() + text.size(), result_value);
  
  // If successful, return the parsed value
  if (parse_result.ec == std::errc())
  {
    return result_value;
  }

  // Otherwise, return 0 by default
  return 0.0;
}

CallbackReturn JetlegSystem::on_init(const hardware_interface::HardwareInfo & info)
{

  // Process basic information from hardware description
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Initialize storage for joint interfaces
  initialize_storage_vectors(joint_commands_, joint_states_, standard_interfaces_, info_.joints);
  
  // Set all states without initial values to 0 by default
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    for (auto j = 0u; j < standard_interfaces_.size(); j++)
    {
      if (std::isnan(joint_states_[j][i]))
      {
        joint_states_[j][i] = 0.0;
      }
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> JetlegSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Joints' state interfaces
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    const auto & joint = info_.joints[i];
    for (const auto & interface : joint.state_interfaces)
    {
      // Denote interface as available to ros2_control
      if (!get_interface(
            joint.name, standard_interfaces_, interface.name, i, joint_states_, state_interfaces))
      {
        throw std::runtime_error(
          "Interface is not found in the standard list. "
          "This should never happen!");
      }
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> JetlegSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Joints' state interfaces
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto & joint = info_.joints[i];
    for (const auto & interface : joint.command_interfaces)
    {
      // Add interface: if not in the standard list than use "other" interface list
      if (!get_interface(
            joint.name, standard_interfaces_, interface.name, i, joint_commands_,
            command_interfaces))
      {
        
        throw std::runtime_error(
          "Interface is not found in the standard nor other list. "
          "This should never happen!");
      }
    }
  }
  // Set position control mode per default
  joint_control_mode_.resize(info_.joints.size(), EFFORT_INTERFACE_INDEX);

  return command_interfaces;
}

return_type JetlegSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t j = 0; j < joint_states_[POSITION_INTERFACE_INDEX].size(); ++j)
  {
    
    for (size_t j = 0; j < joint_states_[POSITION_INTERFACE_INDEX].size(); ++j)
    {
      // TODO: Implement state feedback from sensor data
    }
  }

  return return_type::OK;
}

// Private methods
template <typename HandleType>
bool JetlegSystem::get_interface(
  const std::string & name, const std::vector<std::string> & interface_list,
  const std::string & interface_name, const size_t vector_index,
  std::vector<std::vector<double>> & values, std::vector<HandleType> & interfaces)
{
  auto it = std::find(interface_list.begin(), interface_list.end(), interface_name);
  if (it != interface_list.end())
  {
    auto j = std::distance(interface_list.begin(), it);
    interfaces.emplace_back(name, *it, &values[j][vector_index]);
    return true;
  }
  return false;
}

// Allocates memory for the command and state interfaces
// for each component. If an initial value is provided in the URDF,
// then it is parsed and stored in the corresponding state memory
void JetlegSystem::initialize_storage_vectors(
  std::vector<std::vector<double>> & commands, std::vector<std::vector<double>> & states,
  const std::vector<std::string> & interfaces,
  const std::vector<hardware_interface::ComponentInfo> & component_infos)
{
  // Initialize storage for all joints, regardless of their existence
  commands.resize(interfaces.size());
  states.resize(interfaces.size());
  for (auto i = 0u; i < interfaces.size(); i++)
  {
    commands[i].resize(component_infos.size(), std::numeric_limits<double>::quiet_NaN());
    states[i].resize(component_infos.size(), std::numeric_limits<double>::quiet_NaN());
  }

  // Initialize with values from URDF
  for (auto i = 0u; i < component_infos.size(); i++)
  {
    const auto & component = component_infos[i];
    for (const auto & interface : component.state_interfaces)
    {
      auto it = std::find(interfaces.begin(), interfaces.end(), interface.name);

      // Check if interface is supported and an initial value is given
      if (it != interfaces.end() && !interface.initial_value.empty())
      {
        auto index = std::distance(interfaces.begin(), it);
        states[index][i] = parse_double(interface.initial_value);
      }
    }
  }
}

template <typename InterfaceType>
bool JetlegSystem::populate_interfaces(
  const std::vector<hardware_interface::ComponentInfo> & components,
  std::vector<std::string> & interface_names, std::vector<std::vector<double>> & storage,
  std::vector<InterfaceType> & target_interfaces, bool using_state_interfaces)
{
  for (auto i = 0u; i < components.size(); i++)
  {
    const auto & component = components[i];
    const auto interfaces =
      (using_state_interfaces) ? component.state_interfaces : component.command_interfaces;
    for (const auto & interface : interfaces)
    {
      if (!get_interface(
            component.name, interface_names, interface.name, i, storage, target_interfaces))
      {
        return false;
      }
    }
  }

  return true;
}
}  // namespace mock_components

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(jetleg_hardware_interfaces::JetlegSystem, hardware_interface::SystemInterface)