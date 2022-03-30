#include "jetleg_pointcloud_proc.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JetLegPointCloudProc>());

  rclcpp::shutdown();
  return 0;
}
