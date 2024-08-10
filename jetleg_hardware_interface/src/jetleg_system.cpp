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
#include <tf2/LinearMath/Quaternion.h>
#include <serial_interface/stream_reader.hpp>
#include <serial_interface/stream_parser.hpp>


namespace jetleg_system
{

enum class ERROR_TYPE {
  MULTIPLE_CMD_IF,
  INVALID_SENSOR_COUNT,
  INVALID_STATE_IF,
  INVALID_CMD_IF
};

class TmpPort : public StreamReader
{
public:
  explicit TmpPort(const std::string& name) : StreamReader(name)
  {}

  std::string getBytes(std::size_t /*numBytes*/) override
  {
    return "";
  }
  std::string getLine(const std::string& /*delimiter*/) override
  {
    return "";
  }
};

class TmpParser : public StreamParser
{
public:
  std::shared_ptr<SensorState> next(std::shared_ptr<StreamReader> /*port*/) override
  {
    auto result = toggle ? std::make_shared<SensorState>(sensor_msgs::msg::Imu(), 0.0) : nullptr;
    toggle = false;

    return result;
  }
private:
  bool toggle = true;
};

const static std::shared_ptr<TmpParser> parser = std::make_shared<TmpParser>();

static std::map<ERROR_TYPE, const std::string> ERROR_MSG_TEMPLATES = {
  {ERROR_TYPE::MULTIPLE_CMD_IF, "<%s> has multiple detected command interfaces. This is currently not supported."},
  {ERROR_TYPE::INVALID_SENSOR_COUNT, "Invalid number of sensors: expected: <1>, actual: <%s>"},
  {ERROR_TYPE::INVALID_STATE_IF, "State interface <%s> for Joint <%s> not found. Please ensure the hardware is correctly described in URDF."},
  {ERROR_TYPE::INVALID_CMD_IF, "Command interface <%s> not found. Please ensure the hardware is correctly described in URDF."}
};

CallbackReturn JetlegSystem::on_init(const hardware_interface::HardwareInfo & info)
{

  // Used to share status of hardware interface
  rclcpp::Logger logger = rclcpp::get_logger("JetlegSystem");
  RCLCPP_INFO(logger, "Initializing JetlegSystem hardware interface...");

  // Delegate to base class to perform initial hardware setup
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Make sure each joint has at most one command interface
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() > 1) {

      // Record the error and return with non-successful status
      RCLCPP_ERROR(
        logger, ERROR_MSG_TEMPLATES[ERROR_TYPE::MULTIPLE_CMD_IF].c_str(),joint.name.c_str()
      );
      return CallbackReturn::ERROR;
    }
  }

  // Make sure there is only 1 IMU sensor
  if(info_.sensors.size() != 1) {
    RCLCPP_ERROR(
      logger, ERROR_MSG_TEMPLATES[ERROR_TYPE::INVALID_SENSOR_COUNT].c_str(),info_.sensors.size()
    );

    return CallbackReturn::ERROR;
  }

  mMCUInterface = std::make_shared<MCUInterface>(std::make_shared<TmpPort>(""));

  RCLCPP_INFO(logger, "JetlegSystem hardware interface has been initialized.");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> JetlegSystem::export_state_interfaces()
{

  // Used to share status of hardware interface
  rclcpp::Logger logger = rclcpp::get_logger("JetlegSystem");
  RCLCPP_INFO(logger, "Exporting JetlegSystem state interfaces...");

  // Shares joint states with the rest of ros2_control
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Possible supported state interface types
  // Note: Not all may be supported at this time
  const std::set<std::string> standard_interfaces = {
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_ACCELERATION, hardware_interface::HW_IF_EFFORT
  };

  // Look over each joint in the robot
  for (const auto & joint : info_.joints) {

    // For each joint, add accessible joint state interfaces
    for (const auto & state_interface : joint.state_interfaces) {

      // Find state interface if it exists
      bool isValidInterface = standard_interfaces.count(state_interface.name) == 1;

      // Make an entry in list of StateInterfaces if found
      if (isValidInterface) {

        mJointStates[joint.name][state_interface.name] = 0.0;
        double* const jointReference = &mJointStates[joint.name][state_interface.name];
        state_interfaces.emplace_back(joint.name, state_interface.name, jointReference);

      } else {
        RCLCPP_ERROR(
          logger, ERROR_MSG_TEMPLATES[ERROR_TYPE::INVALID_STATE_IF].c_str(),
          state_interface.name.c_str(), joint.name.c_str()
        );
      }
    }
  }

  // Add the state interfaces for each IMU sensor data
  for(const auto& imu : info_.sensors)
  {
    for (const auto& interface : imu.state_interfaces)
    {
      const auto it = mSensorData[imu.name].insert({interface.name, 0.0});
      state_interfaces.emplace_back(imu.name, it.first->first, &it.first->second);
    }
  }

  std::string interfaceListString = "";
  for(size_t i = 0; i < state_interfaces.size(); i++) {
    interfaceListString += "\n\tname: " + state_interfaces[i].get_name();
  }

  RCLCPP_INFO(logger, "Available state interfaces: [%s\n]", interfaceListString.c_str());

  RCLCPP_INFO(logger, "JetlegSystem hardware interface has exported state interfaces.");
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> JetlegSystem::export_command_interfaces()
{

  // Used to share status of hardware interface
  rclcpp::Logger logger = rclcpp::get_logger("JetlegSystem");
  RCLCPP_INFO(logger, "Exporting JetlegSystem command interfaces...");

  // Advertises command interfaces to the rest of ros2_control
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Possible supported state interface types
  // Note: Not all may be supported at this time
  const std::set<std::string> standard_interfaces = {
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_ACCELERATION, hardware_interface::HW_IF_EFFORT
  };

  for (const auto & joint : info_.joints) {

    // Search for valid command interfaces to add to list of CommandInterfaces
    for (const auto & command_interface : joint.command_interfaces) {

      // Find state interface if it exists
      bool isValidInterface = standard_interfaces.count(command_interface.name) == 1;

      // Make an entry in list of StateInterfaces if found
      if (isValidInterface) {

        mJointCommands[joint.name] = 0.0;
        command_interfaces.emplace_back(joint.name, command_interface.name, &mJointCommands[joint.name]);
      } else {
        RCLCPP_ERROR(
          logger, ERROR_MSG_TEMPLATES[ERROR_TYPE::INVALID_CMD_IF].c_str(),
          command_interface.name.c_str()
        );
      }
    }
  }

  RCLCPP_INFO(logger, "JetlegSystem hardware interface has exported command interfaces.");
  return command_interfaces;
}

hardware_interface::return_type JetlegSystem::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  mMCUInterface->processStream(parser);

  updateSensorData();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type JetlegSystem::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // TODO: Add logic for sending input to the MCU
  // mMCUInterface->updateInput(mJointCommands);

  return hardware_interface::return_type::OK;
}

void JetlegSystem::updateSensorData()
{
  sensor_msgs::msg::Imu::SharedPtr imu = mMCUInterface->getImu();

  const static std::vector<std::string> interfaces = {
    "orientation.x", "orientation.y", "orientation.z", "orientation.w",
    "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
    "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"
  };
  std::vector<double> values = {
    imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w,
    imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z,
    imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z
  };
  for(size_t i = 0; i < values.size(); i++) {
    mSensorData["imu0"][interfaces[i]] = values[i];
  }

  // Update knee position
  mJointStates["knee_joint_"]["position"] = mMCUInterface->getKneeSignal();

  // TODO: Transfer this logic into a separate program / ROS node
  // Update hip position
  // tf2::Quaternion structuredOrientation(orientation[0], orientation[1], orientation[2],
  //   orientation[3]);

  // tf2::Vector3 rotationAxis = structuredOrientation.getAxis();
  // double rotationAngle = structuredOrientation.getAngle();

  // mJointStates[2][0] = rotationAxis[1] * rotationAngle;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(jetleg_system::JetlegSystem, hardware_interface::SystemInterface)
