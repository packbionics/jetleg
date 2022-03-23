#include "jetleg_pointcloud_proc.hpp"

JetLegPointCloudProc::JetLegPointCloudProc() : rclcpp::Node("jetleg_pointcloud_proc") {
  RCLCPP_INFO(this->get_logger(), "Instance of jetleg_pointcloud_proc node has been created...");

  subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>("/zed2i/zed_node/point_cloud/cloud_registered", 
                                                                        10,
                                                                        std::bind(&JetLegPointCloudProc::listener_callback, this, std::placeholders::_1));
  publisher = this->create_publisher<sensor_msgs::msg::Image>("/heightmap", 10);
}

void JetLegPointCloudProc::listener_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  auto time_start = std::chrono::steady_clock::now();

  std::vector<float> data_tmp(msg->data.size() / sizeof(float));
  uchar tmp_char[4];
  float tmp_float;
  for(int i = 0; i < data_tmp.size(); i++) {
    unsigned int float_index = i * sizeof(float);
    tmp_float;

    tmp_char[0] = msg->data[float_index + 0];
    tmp_char[1] = msg->data[float_index + 1];
    tmp_char[2] = msg->data[float_index + 2];
    tmp_char[3] = msg->data[float_index + 3];

    memcpy(&tmp_float, &tmp_char, sizeof(float));
    data_tmp[i] = tmp_float;
  }

  // for(int i = 0; i < 320; i++) {
  //   RCLCPP_INFO(this->get_logger(), "Values: " + std::to_string(data_tmp[i]) + " at index " + std::to_string(i));
  // }

  std::vector<std::array<float, 3>> cloud_array(data_tmp.size());
  for(unsigned int i = 0; i < cloud_array.size(); i += 4) {
    std::array<float, 3> tmp = {data_tmp[i], data_tmp[i + 1], data_tmp[i + 2]};
    cloud_array[i / 4] = tmp;
  }

  cv::Mat heightmap;
  convert_heightmap(heightmap, cloud_array);

  auto time_end = std::chrono::steady_clock::now();
  // RCLCPP_INFO(this->get_logger(), std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count() / 1000.0f));
}

void JetLegPointCloudProc::convert_heightmap(cv::Mat &heightmap, std::vector<std::array<float, 3>> cloud_array) {
  const float PI = 3.141592f;
  float theta_z_upper = 5.0 * PI / 180.0f;

  int counter = 0;
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

  unsigned int map_rows = 40;
  unsigned int map_cols = 40;

  heightmap = cv::Mat(map_rows, map_cols, CV_32F);

  int idx_x = 0;
  int idx_y = 0;

  unsigned int map_rows_minus_one = map_rows - 1;
  unsigned int map_cols_minus_one = map_cols - 1;

  float x_range = (x_maximum - x_minimum) / map_rows_minus_one;
  float y_range = (y_maximum - y_minimum) / map_cols_minus_one;

  float heightmap_array[40][40];
  for(int i = 0; i < 40; i++) {
    for(int j = 0; j < 40; j++) {
      heightmap_array[i][j] = 0;
    }
  }

  for(unsigned int i = 0; i < cloud_restriction.size(); i++) {
    idx_x = (cloud_restriction[i][0] - x_minimum) / x_range;
    idx_y = (cloud_restriction[i][1] - y_minimum) / y_range;

    // if(heightmap.at<float>(idx_y, idx_x) == 0) {
    //   heightmap.at<float>(idx_y, idx_x) = cloud_restriction[i][2];
    // } else {
    //   heightmap.at<float>(idx_y, idx_x) = std::max(cloud_restriction[i][2], heightmap.at<float>(idx_y, idx_x));
    // }

    if(abs(heightmap_array[idx_x][idx_y]) < 0.000001) {
      heightmap_array[idx_x][idx_y] = cloud_restriction[i][2];
    } else {
      heightmap_array[idx_x][idx_y] = std::max(cloud_restriction[i][2], heightmap_array[idx_x][idx_y]);
    }
  }

  heightmap = cv::Mat(40, 40, CV_32FC1, &heightmap_array);

  // cv::Mat kernel(5, 5, CV_8UC1);

  // kernel.row(0) = (cv::Mat_<uint8_t>(1,5) << 0, 0, 1, 0, 0);
  // kernel.row(1) = (cv::Mat_<uint8_t>(1,5) << 1, 1, 1, 1, 1);
  // kernel.row(2) = (cv::Mat_<uint8_t>(1,5) << 1, 1, 1, 1, 1);
  // kernel.row(3) = (cv::Mat_<uint8_t>(1,5) << 1, 1, 1, 1, 1);
  // kernel.row(4) = (cv::Mat_<uint8_t>(1,5) << 0, 0, 1, 0, 0);

  // cv::erode(heightmap, heightmap, kernel);
  // cv::dilate(heightmap, heightmap, kernel);

  cv::Mat heightmap_in_bytes;
  for(int i = 0; i < 40; i++) {
    for(int j = 0; j < 40; j++) {
      heightmap_array[i][j] = 1.0 + heightmap_array[i][j];
      heightmap_array[i][j] *= 255.0f;
    }
  }
  heightmap_in_bytes = cv::Mat(40, 40, CV_32FC1, &heightmap_array);
  heightmap_in_bytes.convertTo(heightmap_in_bytes, CV_8UC1);


  cv_bridge::CvImage img_bridge = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_8UC1, heightmap_in_bytes);
  sensor_msgs::msg::Image imgmsg;
  img_bridge.toImageMsg(imgmsg);
  publisher->publish(imgmsg);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JetLegPointCloudProc>());

  rclcpp::shutdown();
  return 0;
}
