#ifndef _POINTCLOUD_PROC_HPP
#define _POINTCLOUD_PROC_HPP

#include <vector>
#include <chrono>
#include <cv_bridge/cv_bridge.h>

#include <glm/vec4.hpp>
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

        /************ MEMBER FUNCTIONS ************/

        // Updates point cloud information
        void pointcloudSubCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        // Updates camera pose information
        void poseStampedSubCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        // Loads point cloud data from PointCloud2 topic
        void loadData(std::vector<glm::vec4> &data, const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        // Generates heightmap
        void convertHeightmap(const std::vector<glm::vec4> &cloud_array);

        // Generates traversibility map used to determine possible steps
        void computeTraversibility(cv::Mat &heightmap, cv::Mat &traversibility_map);

        // Visualizes heightmap by publishing Image topic
        void publishImage(const cv::Mat &src, cv::Mat &out, float scale = 255.0f);

        bool closeTo(float a, float b, float threshold);

        // Prints RCLCPP Info message
        void printInfo(std::string msg);
        
        // Transforms points from camera space to world space
        void convertToWorldFramePoint(std::vector<glm::vec4> &cloud_array, unsigned int index);

        /************ MEMBER VARIABLES ************/

        // ROS subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subPointCloud;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subPoseStamped;

        // ROS publishers
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;

        // Position and orientation of camera
        glm::vec4 position;
        glm::quat orientation;

        // Orientation of camera represented as euler rotations XYZ
        glm::vec3 eulerAngles;

        // Fields per point
        const unsigned int X = 0;
        const unsigned int Y = 1;
        const unsigned int Z = 2;

        // Represents PI as a floating-point value
        const float PI = 3.141592f;

        // Clip values for point cloud
        const float X_MIN = 0.0f;
        const float X_MAX = 1.5f;

        const float Y_MIN = -0.4f;
        const float Y_MAX =  0.4f;

        const float Z_MAX = 55.0 * PI / 180.0f;

        // Dimensions of heightmap
        const unsigned int MAP_ROWS = (X_MAX - X_MIN) * 42;
        const unsigned int MAP_COLS = (Y_MAX - Y_MIN) * 42;

        //QOS of topics
        const unsigned int QOS = 10;

        // Bytes per field (e.g. sizeof(float) = 4)
        const unsigned int STEP_SIZE = sizeof(float);

        // Number of fields for each point (e.g. xyz_ --> 4)
        const unsigned int POINT_OFFSET = 4;

        // Used to compare two floating-point value
        const float FLOATING_POINT_THRESHOLD = 0.0001f;

        // Size of kernel used to process heightmap
        const unsigned int KERNEL_SIZE = 5;

        // Contains processed images
        cv::Mat heightmap;
        cv::Mat processedHeightmap;
        cv::Mat traversibilityMap;
        cv::Mat gradientMap;

        // Contains processed images (in bytes)
        cv::Mat heightmapInBytes;
        cv::Mat traversibilityInBytes;
};

#endif //_POINTCLOUD_PROC_HPP