#include "jetleg_pointcloud_proc.hpp"

JetLegPointCloudProc::JetLegPointCloudProc() : rclcpp::Node("jetleg_pointcloud_proc") {
  RCLCPP_INFO(this->get_logger(), "jetleg_pointcloud_proc node has been created...");

  subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>("/zed2i/zed_node/point_cloud/cloud_registered", 
                                                                        10,
                                                                        std::bind(&JetLegPointCloudProc::listener_callback, this, std::placeholders::_1));
  publisher = this->create_publisher<sensor_msgs::msg::Image>("/heightmap", 10);
}

void JetLegPointCloudProc::listener_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  auto time_start = std::chrono::steady_clock::now();

  std::vector<float> data_tmp(msg->data.size() / sizeof(float));
  uchar tmp_char[4];
  for(unsigned int i = 0; i < data_tmp.size(); i++) {
    unsigned int float_index = i * sizeof(float);

    float tmp_float;

    tmp_char[0] = msg->data[float_index + 0];
    tmp_char[1] = msg->data[float_index + 1];
    tmp_char[2] = msg->data[float_index + 2];
    tmp_char[3] = msg->data[float_index + 3];

    memcpy(&tmp_float, &tmp_char, sizeof(float));
    data_tmp[i] = tmp_float;
  }

  std::vector<std::array<float, 3>> cloud_array(data_tmp.size());
  for(unsigned int i = 0; i < cloud_array.size(); i += 4) {
    std::array<float, 3> tmp = {data_tmp[i], data_tmp[i + 1], data_tmp[i + 2]};
    cloud_array[i / 4] = tmp;
  }

  cv::Mat heightmap;
  cv::Mat traversibility_map;

  convert_heightmap(heightmap, cloud_array);

  for(int j = 0; j < 40; j++) {
    for(int i = 0; i < 40; i++) {
      RCLCPP_INFO(this->get_logger(), std::to_string(i) + ", " + std::to_string(j) + ": " + std::to_string(heightmap.at<float>(i,j)));
    }
  }
  
  // compute_traversibility(heightmap, traversibility_map);

  // std::string column = "";
  // for(unsigned int i = 0; i < traversibility_map.size().width; i++) {
  //   column += std::to_string(traversibility_map.at<uchar>(i, 0)) + " ";
  // }
  // RCLCPP_INFO(this->get_logger(), column);

  auto time_end = std::chrono::steady_clock::now();
  RCLCPP_INFO(this->get_logger(), "Time (s) per Tick: " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count() / 1000.0f));
}

void JetLegPointCloudProc::convert_heightmap(cv::Mat &heightmap, std::vector<std::array<float, 3>> cloud_array) {
  const float PI = 3.141592f;
  float theta_z_upper = 5.0 * PI / 180.0f;

  std::vector<std::array<float, 3>> cloud_restriction;
  for(unsigned int i = 0; i < cloud_array.size(); i++) {
    if(cloud_array[i][0] <= 2.0f && cloud_array[i][0] > 0.0f) {
      if(cloud_array[i][1] <= 0.6f && cloud_array[i][1] >= -0.6f) {
        if(cloud_array[i][2] <= cloud_array[i][0] * tan(theta_z_upper)) {
          cloud_restriction.push_back(cloud_array[i]);
        }
      }
    }
  }

  float x_minimum = cloud_restriction[0][0];
  float x_maximum = cloud_restriction[0][0];

  float y_minimum = cloud_restriction[0][1];
  float y_maximum = cloud_restriction[0][1];

  for(unsigned int i = 0; i < cloud_restriction.size(); i++) {
    if(cloud_restriction[i][0] < x_minimum && cloud_restriction[i][0] != 0.0f) {
      x_minimum = cloud_restriction[i][0];
    }

    if(cloud_restriction[i][0] > x_maximum) {
      x_maximum = cloud_restriction[i][0];
    }

    if(cloud_restriction[i][1] < y_minimum) {
      y_minimum = cloud_restriction[i][1];
    }

    if(cloud_restriction[i][1] > y_maximum) {
      y_maximum = cloud_restriction[i][1];
    }
  }

  const unsigned int map_rows = 40;
  const unsigned int map_cols = 40;

  heightmap = cv::Mat(map_rows, map_cols, CV_32FC1);

  int idx_x = 0;
  int idx_y = 0;

  unsigned int map_rows_minus_one = map_rows - 1;
  unsigned int map_cols_minus_one = map_cols - 1;

  float x_range = (x_maximum - x_minimum) / map_rows_minus_one;
  float y_range = (y_maximum - y_minimum) / map_cols_minus_one;

  float heightmap_array[map_rows][map_cols];
  for(unsigned int i = 0; i < map_rows; i++) {
    for(unsigned int j = 0; j < map_cols; j++) {
      heightmap_array[i][j] = 0;
    }
  }

  for(unsigned int i = 0; i < cloud_restriction.size(); i++) {
    idx_x = (cloud_restriction[i][0] - x_minimum) / x_range;
    idx_y = (cloud_restriction[i][1] - y_minimum) / y_range;

    if(heightmap.at<float>(idx_x, idx_y) < 0.001f) {
      heightmap_array[idx_x][idx_y] = cloud_restriction[i][2];
      // heightmap.at<float>(idx_y, idx_x) = cloud_restriction[i][2];
    } else {
      heightmap_array[idx_x][idx_y] = std::max(cloud_restriction[i][2], heightmap_array[idx_x][idx_y]);
      // heightmap.at<float>(idx_y, idx_x) = std::max(cloud_restriction[i][2], heightmap.at<float>(idx_y, idx_x));
    }
  }

  for(int i = 0; i < map_rows; i++) {
    for(int j = 0; j < map_cols; j++) {
      heightmap.at<float>(i, j) = heightmap_array[i][j];
    }
  }

  // heightmap.at<float>(0,0) = cloud_restriction[20][5];
  // RCLCPP_INFO(this->get_logger(), "ATTENTION: " + std::to_string(heightmap.at<float>(0, 0)));
  
  cv::Mat kernel(5, 5, CV_8UC1);

  kernel.col(0) = (cv::Mat_<uint8_t>(1,5) << 0, 0, 1, 0, 0);
  kernel.col(1) = (cv::Mat_<uint8_t>(1,5) << 1, 1, 1, 1, 1);
  kernel.col(2) = (cv::Mat_<uint8_t>(1,5) << 1, 1, 1, 1, 1);
  kernel.col(3) = (cv::Mat_<uint8_t>(1,5) << 1, 1, 1, 1, 1);
  kernel.col(4) = (cv::Mat_<uint8_t>(1,5) << 0, 0, 1, 0, 0);

  cv::erode(heightmap, heightmap, kernel);
  cv::dilate(heightmap, heightmap, kernel);

  // publish_heightmap(heightmap);

  // for(int j = 0; j < 40; j++) {
  //   for(int i = 0; i < 40; i++) {
  //     RCLCPP_INFO(this->get_logger(), std::to_string(i) + ", " + std::to_string(j) + ": " + std::to_string(heightmap.at<float>(i,j)));
  //   }
  // }
} 

void JetLegPointCloudProc::publish_heightmap(cv::Mat heightmap) {
  cv::Mat heightmap_in_bytes;

  float heightmap_array[heightmap.size().width][heightmap.size().height];
  for(unsigned int i = 0; i < heightmap.size().height; i++) {
    for(unsigned int j = 0; j < heightmap.size().width; j++) {
      if(std::abs(heightmap.at<float>(i, j)) < 0.001) {
        heightmap_array[i][j] = 0.0f;
        continue;
      }

      heightmap_array[i][j] = (1.0f + heightmap.at<float>(i, j)) * 255.0f;
      heightmap.convertTo(heightmap_in_bytes, CV_8UC1);
    }
  }

  cv_bridge::CvImage img_bridge = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_8UC1, heightmap_in_bytes);
  sensor_msgs::msg::Image imgmsg;

  img_bridge.toImageMsg(imgmsg);
  publisher->publish(imgmsg);
}

void JetLegPointCloudProc::compute_traversibility(cv::Mat &heightmap, cv::Mat &traversibility_map) {
  cv::Mat sobel_x;
  cv::Mat sobel_y;

  // for(int j = 0; j < 40; j++) {
  //   for(int i = 0; i < 40; i++) {
  //     RCLCPP_INFO(this->get_logger(), std::to_string(i) + ", " + std::to_string(j) + ": " + std::to_string(heightmap.at<float>(i,j)));
  //   }
  // }

  cv::Sobel(heightmap, sobel_x, CV_64F, 1, 0, 5);
  // RCLCPP_INFO(this->get_logger(), "Sobel X: " + std::to_string(sobel_x.at<double>(5,20)));

  cv::Sobel(heightmap, sobel_y, CV_64F, 0, 1, 5);
  // RCLCPP_INFO(this->get_logger(), "Sobel Y: " + std::to_string(sobel_y.at<double>(5,20)));

  cv::Mat gradient_map(sobel_x.size(), CV_32FC1);
  for(int i = 0; i < sobel_x.size().width; i++) {
    for(int j = 0; j < sobel_x.size().height; j++) {
      gradient_map.at<float>(j, i) = std::max(sobel_x.at<float>(j, i), sobel_y.at<float>(j, i));
    }
  }

  traversibility_map = cv::Mat(gradient_map.size(), CV_8UC1);
  for(int i = 0; i < traversibility_map.size().width; i++) {
    for(int j = 0; j < traversibility_map.size().height; j++) {
      if(gradient_map.at<float>(j, i) < 5) {

        traversibility_map.at<uchar>(j, i) = 1;
      } else if(gradient_map.at<float>(j, i) >= 5) {

        traversibility_map.at<uchar>(j, i) = 2;
      } else if(gradient_map.at<float>(j, i) >= 20) {

        traversibility_map.at<uchar>(j, i) = 4;
      } else if(gradient_map.at<float>(j, i) >= 35) {

        traversibility_map.at<uchar>(j, i) = 4;
      }

      else {
        traversibility_map.at<uchar>(j, i) = 4;
      }
    }
  }
}