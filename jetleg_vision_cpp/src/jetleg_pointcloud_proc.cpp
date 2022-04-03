#include "jetleg_pointcloud_proc.hpp"

JetLegPointCloudProc::JetLegPointCloudProc() : rclcpp::Node("jetleg_pointcloud_proc"),
                                               MAP_ROWS(40),
                                               MAP_COLS(40),
                                               X(0),
                                               Y(1),
                                               Z(2),
                                               PI(3.141592f),
                                               X_MIN(0.0f),
                                               X_MAX(2.0f),
                                               Y_MIN(-0.6f),
                                               Y_MAX(0.6f),
                                               Z_MAX(5.0 * PI / 180.0f) {
  RCLCPP_INFO(this->get_logger(), "jetleg_pointcloud_proc node has been created...");

  subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>("/zed2i/zed_node/point_cloud/cloud_registered", 
                                                                        10,
                                                                        std::bind(&JetLegPointCloudProc::listener_callback, this, std::placeholders::_1));
  publisher = this->create_publisher<sensor_msgs::msg::Image>("/heightmap", 10);

  // Stores output as floats
  heightmap = cv::Mat(MAP_ROWS, MAP_COLS, CV_32FC1);
  traversibility_map = cv::Mat(MAP_ROWS, MAP_COLS, CV_8UC1);
  gradient_map = cv::Mat(MAP_ROWS, MAP_COLS, CV_32FC1);

  // Stores outputs as bytes to be compatible with RVIZ
  heightmap_in_bytes = cv::Mat(MAP_ROWS, MAP_COLS, CV_8UC1);
  traversibility_in_bytes = cv::Mat(MAP_ROWS, MAP_COLS, CV_8UC1);
}

void JetLegPointCloudProc::listener_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  auto time_start = std::chrono::steady_clock::now();

  // Bytes per field (e.g. sizeof(float) = 4)
  const unsigned int step_size = sizeof(float);
  std::vector<std::array<float, 3>> cloud_array(msg->data.size() / step_size);
    
  // Convert from byte array to float array of structure XYZ
  load_data(cloud_array, step_size, msg);

  convert_heightmap(cloud_array);
  compute_traversibility(heightmap, traversibility_map);

  // Publishes heightmap to visualize with RVIZ
  // publish_image(heightmap, heightmap_in_bytes);

  // Publishes heightmap to visualize with RVIZ
  publish_image(traversibility_map, traversibility_in_bytes, 63.0f);

  auto time_end = std::chrono::steady_clock::now();
  RCLCPP_INFO(this->get_logger(), "Time (s) per Tick: " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count() / 1000.0f));
}

void JetLegPointCloudProc::load_data(std::vector<std::array<float, 3>> &data, const unsigned int step_size, const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // Stores byte array representing single floating-point value
  uchar* bytes = new uchar[step_size];

  // Step taken in the data field
  unsigned int data_step;

  // Number of fields for each point (e.g. xyz_ --> 4)
  unsigned int point_offset = 4;

  // Iterate through points
  for(unsigned int i = 0; i < data.size(); i += 4) {
    data_step = i * point_offset;

    // Iterates through fields per point
    for(unsigned int j = 0; j < 3; j++) {

      // Iterate through bytes per float
      for(unsigned int k = 0; k < step_size; k++) {
        bytes[k] = msg->data[data_step + step_size * j + k];
      }

      // Convert from byte array to float
      memcpy(&data[i][j], bytes, step_size);
    }
  }

  delete[] bytes;
}

void JetLegPointCloudProc::convert_heightmap(std::vector<std::array<float, 3>> cloud_array) {

  // Minimum and maximum value for X found in restricted point cloud
  float x_min;
  float x_max;

  // Minimum and maximum value for Y found in restricted point cloud
  float y_min;
  float y_max;

  std::vector<std::array<float, 3>> filtered_cloud;
  for(unsigned int i = 0; i < cloud_array.size(); i++) {

    // Applies X restrictions
    if(cloud_array[i][X] <= X_MAX && cloud_array[i][X] > X_MIN) {

      // Applies Y restrictions
      if(cloud_array[i][Y] <= Y_MAX && cloud_array[i][Y] >= Y_MIN) {

        // Applies Z restriction
        if(cloud_array[i][Z] <= cloud_array[i][X] * tan(Z_MAX)) {

          filtered_cloud.push_back(cloud_array[i]);

          if(filtered_cloud.size() == 1) {
            x_min = filtered_cloud[0][X];
            x_max = filtered_cloud[0][X];

            y_min = filtered_cloud[0][Y];
            y_max = filtered_cloud[0][Y];

            continue;
          }

          update_minmax(filtered_cloud.back(), x_min, x_max, y_min, y_max);
        }
      }
    }
  }

  int idx_x = 0;
  int idx_y = 0;

  float x_range = (x_max - x_min) / (MAP_ROWS - 1);
  float y_range = (y_max - y_min) / (MAP_COLS - 1);

  const float FLOATING_POINT_THRESHOLD = 0.0001f;

  for(unsigned int i = 0; i < filtered_cloud.size(); i++) {
    idx_x = (filtered_cloud[i][X] - x_min) / x_range;
    idx_y = (filtered_cloud[i][Y] - y_min) / y_range;

    if(abs(heightmap.at<float>(idx_x, idx_y)) < FLOATING_POINT_THRESHOLD) {
      heightmap.at<float>(idx_x, idx_y) = filtered_cloud[i][Z];
    } else {
      heightmap.at<float>(idx_x, idx_y) = std::max(filtered_cloud[i][Z], heightmap.at<float>(idx_x, idx_y));
    }
  }

  const unsigned int KERNEL_SIZE = 5;

  // Kernel used to process raw heightmap
  cv::Mat kernel(KERNEL_SIZE, KERNEL_SIZE, CV_8UC1);

  kernel.col(0) = (cv::Mat_<uint8_t>(1, KERNEL_SIZE) << 0, 0, 1, 0, 0);
  kernel.col(1) = (cv::Mat_<uint8_t>(1, KERNEL_SIZE) << 1, 1, 1, 1, 1);
  kernel.col(2) = (cv::Mat_<uint8_t>(1, KERNEL_SIZE) << 1, 1, 1, 1, 1);
  kernel.col(3) = (cv::Mat_<uint8_t>(1, KERNEL_SIZE) << 1, 1, 1, 1, 1);
  kernel.col(4) = (cv::Mat_<uint8_t>(1, KERNEL_SIZE) << 0, 0, 1, 0, 0);

  // Applies image processing
  cv::erode(heightmap, heightmap, kernel);
  cv::dilate(heightmap, heightmap, kernel);
} 

void JetLegPointCloudProc::publish_image(const cv::Mat &src, cv::Mat &out, float scale) {

  // Scales image using "scale" variable
  if(src.type() == CV_32FC1) {
    for(int i = 0; i < src.size().height; i++) {
      for(int j = 0; j < src.size().width; j++) {
        out.at<uchar>(i, j) = src.at<float>(i, j) * scale;
      }
    }
  }

  if(src.type() == CV_8UC1) {
    for(int i = 0; i < src.size().height; i++) {
      for(int j = 0; j < src.size().width; j++) {
        out.at<uchar>(i, j) = src.at<uchar>(i, j) * scale;
      }
    }
  }

  // Generates Image msg from cv::Mat
  cv_bridge::CvImage img_bridge = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_8UC1, out);
  sensor_msgs::msg::Image imgmsg;

  // Publishes to topic
  img_bridge.toImageMsg(imgmsg);
  publisher->publish(imgmsg);
}

void JetLegPointCloudProc::update_minmax(std::array<float, 3> point, float &x_min, float &x_max, float &y_min, float &y_max) {
  if(point[X] < x_min) {
    x_min = point[X];
  }

  if(point[X] > x_max) {
    x_max = point[X];
  }

  if(point[Y] < y_min) {
    y_min = point[Y];
  }

  if(point[Y] > y_max) {
    y_max = point[Y];
  }
}

void JetLegPointCloudProc::compute_traversibility(cv::Mat &heightmap, cv::Mat &traversibility_map) {
  const float FLOATING_POINT_THRESHOLD = 0.0001f;
  
  cv::Mat sobel_x;
  cv::Mat sobel_y;

  cv::Sobel(heightmap, sobel_x, CV_64F, 1, 0, 5);
  cv::Sobel(heightmap, sobel_y, CV_64F, 0, 1, 5);

  for(int i = 0; i < MAP_ROWS; i++) {
    for(int j = 0; j < MAP_COLS; j++) {
      float x_grad = sobel_x.at<double>(i, j);
      float y_grad = sobel_y.at<double>(i, j);

      float max_grad = std::max(std::abs(x_grad), std::abs(y_grad));
      gradient_map.at<float>(i, j) = max_grad;
    }
  }

  for(int i = 0; i < MAP_ROWS; i++) {
    for(int j = 0; j < MAP_COLS; j++) {
      if(abs(heightmap.at<float>(i, j)) < FLOATING_POINT_THRESHOLD) {
        traversibility_map.at<uchar>(i, j) = (uchar)4;

      } else if(gradient_map.at<float>(i, j) < 5) {
        traversibility_map.at<uchar>(i, j) = (uchar)1;

      } else if(gradient_map.at<float>(i, j) < 20) {
        traversibility_map.at<uchar>(i, j) = (uchar)2;

      } else if(gradient_map.at<float>(i, j) < 35) {
        traversibility_map.at<uchar>(i, j) = (uchar)4;

      } else if(gradient_map.at<float>(i, j) < 50) {
        traversibility_map.at<uchar>(i, j) = (uchar)4;

      } else {
        traversibility_map.at<uchar>(i, j) = (uchar)0;
      }
    }
  }
}

void JetLegPointCloudProc::print_info(std::string msg) {
  RCLCPP_INFO(this->get_logger(), msg);
}