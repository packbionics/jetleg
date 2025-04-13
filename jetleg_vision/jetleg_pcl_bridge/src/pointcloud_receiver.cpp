#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


void topic_callback(const sensor_msgs::msg::PointCloud2 & msg)
{
    
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pointcloud_receiver");

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSubscriber = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "points",
        rclcpp::SensorDataQoS(),
        &topic_callback
    );

    rclcpp::spin(node);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}