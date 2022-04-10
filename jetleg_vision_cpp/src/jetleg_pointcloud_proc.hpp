#ifndef _POINTCLOUD_PROC_HPP
#define _POINTCLOUD_PROC_HPP

#include <vector>
#include <chrono>
#include <cv_bridge/cv_bridge.h>

#include "../glm/vec3.hpp"

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/header.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

/**
 * Structure to hold point position and define valid operations
 */
struct PointXYZ {
    float x;
    float y;
    float z;

    PointXYZ() {
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
    }

    PointXYZ(float x, float y, float z) {
      this->x = x;
      this->y = y;
      this->z = z;
    }

    // Adds 4 bytes of padding to turn the size of the structure
    // into a power of 2
    float padding;

    /**
     * Defines the addition operator for PointXYZ
     */
    PointXYZ operator+(PointXYZ b) {
      PointXYZ sum;

      sum.x = this->x + b.x;
      sum.y = this->y + b.y;
      sum.z = this->z + b.z;

      return sum;
    }

    /**
     * Defines the addition operator for PointXYZ
     */
    PointXYZ operator+(std::array<float, 3> b) {
      PointXYZ sum;

      sum.x = this->x + b[0];
      sum.y = this->y + b[1];
      sum.z = this->z + b[2];

      return sum;
    }

    /**
     * Defines the subtraction operator for PointXYZ
     */
    PointXYZ operator-(PointXYZ b) {
      PointXYZ difference;

      difference.x = this->x - b.x;
      difference.y = this->y - b.y;
      difference.z = this->z - b.z;

      return difference;
    }

    /**
     * Defines the subtraction operator for PointXYZ
     */
    PointXYZ operator-(std::array<float, 3> b) {
      PointXYZ difference;

      difference.x = this->x - b[0];
      difference.y = this->y - b[1];
      difference.z = this->z - b[2];

      return difference;
    }

    bool operator==(PointXYZ b) {
      if(abs(this->x - b.x) < 0.001) {
        if(abs(this->y - b.y) < 0.001) {
          if(abs(this->z - b.z) < 0.001) {
            return true;
          }
        }
      }

      return false;
    }

    bool operator==(std::array<float, 3> b) {
      if(abs(this->x - b[0]) < 0.001) {
        if(abs(this->y - b[1]) < 0.001) {
          if(abs(this->z - b[2]) < 0.001) {
            return true;
          }
        }
      }

      return false;
    }
};

class JetLegPointCloudProc : public rclcpp::Node {
    public:
        JetLegPointCloudProc();
        ~JetLegPointCloudProc();
    private:
        void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        void updatePoseStamped(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        // Loads point cloud data from PointCloud2 topic
        void load_data(std::vector<PointXYZ> &data, const unsigned int step_size, const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        // Generates heightmap
        void convert_heightmap(std::vector<PointXYZ> cloud_array);

        // Generates traversibility map used to determine possible steps
        void compute_traversibility(cv::Mat &heightmap, cv::Mat &traversibility_map);

        // Visualizes heightmap by publishing Image topic
        void publish_image(const cv::Mat &src, cv::Mat &out, float scale = 255.0f);

        bool close_to(float a, float b, float threshold);

        // Prints RCLCPP Info message
        void print_info(std::string msg);

        // Performs hamilton product
        geometry_msgs::msg::Quaternion calc_hamilton_product(geometry_msgs::msg::Quaternion a, geometry_msgs::msg::Quaternion b);

        // Rotates point using a left hamilton product of camera rotation and right hamilton product of conjugate rotation
        void rotate_point(geometry_msgs::msg::Quaternion quaternion, PointXYZ &point, geometry_msgs::msg::Quaternion inverseQuaternion);

        // Transforms points from camera space to world space
        void convertToWorldFramePoint(std::vector<PointXYZ> &cloud_array, unsigned int index);

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;

        geometry_msgs::msg::PoseStamped pose;
        geometry_msgs::msg::Quaternion orientation_conjugate;

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