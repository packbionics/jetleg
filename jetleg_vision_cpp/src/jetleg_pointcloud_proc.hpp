#ifndef _POINTCLOUD_PROC_HPP
#define _POINTCLOUD_PROC_HPP

#include <vector>
#include <chrono>
#include <cv_bridge/cv_bridge.h>

#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/rotate_vector.hpp>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/header.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class JetLegPointCloudProc : public rclcpp::Node {
    public:
        JetLegPointCloudProc();
        ~JetLegPointCloudProc();
    private:

        // Updates point cloud information
        void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        // Updates camera pose information
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        // Loads point cloud data from PointCloud2 topic
        void load_data(std::vector<glm::vec3> &data, const unsigned int step_size, const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        // Generates heightmap
        void convert_heightmap(const std::vector<glm::vec3> &cloud_array);

        // Generates traversibility map used to determine possible steps
        void compute_traversibility(cv::Mat &heightmap, cv::Mat &traversibility_map);

        // Visualizes heightmap by publishing Image topic
        void publish_image(const cv::Mat &src, cv::Mat &out, float scale = 255.0f);

        bool close_to(float a, float b, float threshold);

        // Prints RCLCPP Info message
        void print_info(std::string msg);
        
        // Transforms points from camera space to world space
        void convertToWorldFramePoint(std::vector<glm::vec3> &cloud_array, unsigned int index);

        // ROS subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber;

        // ROS publishers
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;

        // Position and orientation of camera
        glm::vec3 position;
        glm::quat orientation;

        // Orientation of camera represented as euler rotations XYZ
        glm::vec3 eulerAngles;

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

        // Dimensions of heightmap
        const unsigned int MAP_ROWS;
        const unsigned int MAP_COLS;

        // Contains processed images
        cv::Mat heightmap;
        cv::Mat processed_heightmap;
        cv::Mat traversibility_map;
        cv::Mat gradient_map;

        // Contains processed images (in bytes)
        cv::Mat heightmap_in_bytes;
        cv::Mat traversibility_in_bytes;
};

#endif //_POINTCLOUD_PROC_HPP