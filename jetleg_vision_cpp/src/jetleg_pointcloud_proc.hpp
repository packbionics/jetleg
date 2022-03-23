#ifndef _POINTCLOUD_PROC_HPP
#define _POINTCLOUD_PROC_HPP

#include "rclcpp/rclcpp.hpp"

#include <vector>
#include <chrono>
#include <cv_bridge/cv_bridge.h>

#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class JetLegPointCloudProc : public rclcpp::Node {
    public:
        JetLegPointCloudProc();
    private:
        void listener_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void convert_heightmap(cv::Mat &heightmap, std::vector<std::array<float, 3>> cloud_array);
        void compute_traversibility(std::vector<std::vector<float>> heightmap);

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
};

#endif //_POINTCLOUD_PROC_HPP