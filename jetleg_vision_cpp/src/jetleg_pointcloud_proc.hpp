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

        // Loads point cloud data from PointCloud2 topic
        void load_data(std::vector<std::array<float, 3>> &data, const unsigned int step_size, const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        // Generates heightmap
        void convert_heightmap(std::vector<std::array<float, 3>> cloud_array);

        // Updates values min and max values when appropriate
        void update_minmax(std::array<float, 3> point, float &x_min, float &x_max, float &y_min, float &y_max);

        // Generates traversibility map used to determine possible steps
        void compute_traversibility(cv::Mat &heightmap, cv::Mat &traversibility_map);

        // Visualizes heightmap by publishing Image topic
        void publish_heightmap(cv::Mat src);

        // Prints RCLCPP Info message
        void print_info(std::string msg);

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;

        // Dimensions of heightmap
        const unsigned int MAP_ROWS;
        const unsigned int MAP_COLS;

        // Fields per point
        const unsigned int X;
        const unsigned int Y;
        const unsigned int Z;

        // Represents PI as a floating-point value
        const float PI;

        // Clip values for point cloud
        const float X_MIN;
        const float X_MAX;

        const float Y_MIN;
        const float Y_MAX;

        const float Z_MAX;

        // Contains processed heightmap
        cv::Mat heightmap;
        cv::Mat traversibility_map;

        // Contains processed heightmap (in bytes)
        cv::Mat heightmap_in_bytes;
};

#endif //_POINTCLOUD_PROC_HPP