
.. _program_listing_file_include_jetleg_system_jetleg_system.hpp:

Program Listing for File jetleg_system.hpp
==========================================

|exhale_lsh| :ref:`Return to documentation for file <file_include_jetleg_system_jetleg_system.hpp>` (``include/jetleg_system/jetleg_system.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

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
     void imuLogger();
   
     void updatePose(double timePeriod);
   
     void updateSensorData();
   
     void updateField(std::vector<std::string> interfaceNames, std::vector<double> sensorValues);
   
   
     // static void trapSum(
     //   std::vector<double> & original, const std::vector<double> & vel, size_t timePeriod);
   
     std::vector<std::vector<double>> mJointStates;
   
     std::shared_ptr<serial::LibSerialBridge> serialBridgePointer;
   
     std::vector<double> mJointCommands;
   
     std::vector<double> mLinearStates;
   
     std::vector<double> mLinearSubStates;
   
     std::vector<double> mAngularStates;
   
     std::vector<std::map<std::string, double>> mSensorData;
   
     static constexpr size_t LINEAR_COORDS = 3;
   
     static constexpr size_t ANGULAR_COORDS = 3;
   
     static constexpr size_t BAUD_RATE = 1;
   
   
   };
   
   }
   
   #endif // JETLEG_SYSTEM_HPP
