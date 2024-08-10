// Copyright (c) 2021 PickNik, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Denis Stogl
//
// Modified by Pack Bionics 2023

#include <gmock/gmock.h>

#include "hardware_interface/resource_manager.hpp"

#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

TEST(TestJetlegSystem, load_jetleg_system_2dof)
{
  std::string hardware_system_2dof =
    R"(
  <ros2_control name="HardwareSystem2dof" type="system">
    <hardware>
      <plugin>jetleg_system/JetlegSystem</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">1.57</param>
      </state_interface>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.7854</param>
      </state_interface>
    </joint>
  </ros2_control>
)";

  auto urdf = ros2_control_test_assets::urdf_head + hardware_system_2dof +
    ros2_control_test_assets::urdf_tail;
  ASSERT_THROW(hardware_interface::ResourceManager rm(urdf), std::runtime_error);
}

TEST(TestJetlegSystem, load_jetleg_system_2dof_multiple_cmd_interfaces)
{
  std::string hardware_system_2dof =
    R"(
  <ros2_control name="HardwareSystem2dof" type="system">
    <hardware>
      <plugin>jetleg_system/JetlegSystem</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position">
        <param name="initial_value">1.57</param>
      </state_interface>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.7854</param>
      </state_interface>
    </joint>
  </ros2_control>
)";

  auto urdf = ros2_control_test_assets::urdf_head + hardware_system_2dof +
    ros2_control_test_assets::urdf_tail;
  ASSERT_THROW(hardware_interface::ResourceManager rm(urdf), std::runtime_error);
}

TEST(TestJetlegSystem, load_jetleg_example_interfaces) {
  std::string hardware_system_2dof =
    R"(
    <ros2_control name="jetleg_" type="system">
      <hardware>
        <plugin>jetleg_system/JetlegSystem</plugin>
      </hardware>
      <joint name="vertical_rail_to_mount_">
        <command_interface name="position"/>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
      </joint>
      <joint name="knee_joint_">
        <command_interface name="effort"/>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
      </joint>
      <joint name="ankle_joint_">
        <command_interface name="effort"/>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
      </joint>

      <sensor name="imu0">
        <state_interface name="orientation.x" />
        <state_interface name="orientation.y" />
        <state_interface name="orientation.z" />
        <state_interface name="orientation.w" />

        <state_interface name="angular_velocity.x" />
        <state_interface name="angular_velocity.y" />
        <state_interface name="angular_velocity.z" />

        <state_interface name="linear_acceleration.x" />
        <state_interface name="linear_acceleration.y" />
        <state_interface name="linear_acceleration.z" />
      </sensor>
    </ros2_control>
    )";

  auto urdf = ros2_control_test_assets::urdf_head + hardware_system_2dof +
    ros2_control_test_assets::urdf_tail;
  hardware_interface::ResourceManager rm(urdf);
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
}
