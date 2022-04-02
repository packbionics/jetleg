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
        ~JetLegPointCloudProc();
    private:
        void listener_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        void load_data(std::vector<std::array<float, 3>> &data, const unsigned int step_size, const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        void convert_heightmap(std::vector<std::array<float, 3>> cloud_array);
        void compute_traversibility(cv::Mat &heightmap, cv::Mat &traversibility_map);

        void publish_heightmap(cv::Mat source);

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;

        const float x_min_restriction;
        const float x_max_restriction;

        const float y_min_restriction;
        const float y_max_restriction;

        const unsigned int map_rows;
        const unsigned int map_cols;

        float** heightmap_array;

        cv::Mat heightmap;
        cv::Mat heightmap_in_bytes;
};

#endif //_POINTCLOUD_PROC_HPP