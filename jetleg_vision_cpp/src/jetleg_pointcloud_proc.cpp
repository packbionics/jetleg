#include "jetleg_pointcloud_proc.hpp"

JetLegPointCloudProc::JetLegPointCloudProc() : rclcpp::Node("jetleg_pointcloud_proc"),
                                               X(0),
                                               Y(1),
                                               Z(2),
                                               PI(3.141592f),
                                               X_MIN(0.0f),
                                               X_MAX(1.5f),
                                               Y_MIN(-0.4f),
                                               Y_MAX(0.4f),
                                               Z_MAX(55.0 * PI / 180.0f),
                                               MAP_ROWS((X_MAX - X_MIN) * 42),
                                               MAP_COLS((Y_MAX - Y_MIN) * 42) {
  RCLCPP_INFO(this->get_logger(), "jetleg_pointcloud_proc node has been created...");

  cloud_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>("/zed2i/zed_node/point_cloud/cloud_registered", 
                                                                        10, std::bind(&JetLegPointCloudProc::cloud_callback, this, std::placeholders::_1));
  pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("camera/state",
                                                                               10, std::bind(&JetLegPointCloudProc::pose_callback, this, std::placeholders::_1));

  publisher = this->create_publisher<sensor_msgs::msg::Image>("/heightmap", 10);

  // Stores output as floats
  heightmap = cv::Mat(MAP_ROWS, MAP_COLS, CV_32FC1);
  processed_heightmap = cv::Mat(MAP_ROWS, MAP_COLS, CV_32FC1);
  traversibility_map = cv::Mat(MAP_ROWS, MAP_COLS, CV_8UC1);
  gradient_map = cv::Mat(MAP_ROWS, MAP_COLS, CV_32FC1);

  // Stores outputs as bytes to be compatible with RVIZ
  heightmap_in_bytes = cv::Mat(MAP_ROWS, MAP_COLS, CV_8UC1);
  traversibility_in_bytes = cv::Mat(MAP_ROWS, MAP_COLS, CV_8UC1);
}

JetLegPointCloudProc::~JetLegPointCloudProc() {
  
}

void JetLegPointCloudProc::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  auto time_start = std::chrono::steady_clock::now();

  // Bytes per field (e.g. sizeof(float) = 4)
  const unsigned int step_size = sizeof(float);
  std::vector<glm::vec3> cloud_array(msg->data.size() / step_size);
    
  // Convert from byte array to float array of structure XYZ
  load_data(cloud_array, step_size, msg);

  convert_heightmap(cloud_array);
  compute_traversibility(processed_heightmap, traversibility_map);

  // Publishes heightmap to visualize with RVIZ
  publish_image(processed_heightmap, heightmap_in_bytes);

  // Publishes heightmap to visualize with RVIZ
  // publish_image(traversibility_map, traversibility_in_bytes, 63.0f);

  auto time_end = std::chrono::steady_clock::now();
  RCLCPP_INFO(this->get_logger(), "Time (s) per Tick: " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count() / 1000.0f));
}

void JetLegPointCloudProc::load_data(std::vector<glm::vec3> &data, const unsigned int step_size, const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // Stores byte array representing single floating-point value
  uchar* bytes = new uchar[step_size];

  // Step taken in the data field
  unsigned int data_step;

  // Number of fields for each point (e.g. xyz_ --> 4)
  unsigned int point_offset = 4;

  // Iterate through points
  for(unsigned int i = 0; i < data.size(); i += 4) {
    data_step = i * point_offset;

      for(unsigned int j = 0; j < step_size; j++) {
        bytes[j] = msg->data[data_step + step_size * X + j];
      }

      // Convert from byte array to float
      memcpy(&data[i].x, bytes, step_size);

      for(unsigned int j = 0; j < step_size; j++) {
        bytes[j] = msg->data[data_step + step_size * Y + j];
      }

      // Convert from byte array to float
      memcpy(&data[i].y, bytes, step_size);

      for(unsigned int j = 0; j < step_size; j++) {
        bytes[j] = msg->data[data_step + step_size * Z + j];
      }

      // Convert from byte array to float
      memcpy(&data[i].z, bytes, step_size);
      convertToWorldFramePoint(data, i);
  }

  delete[] bytes;
}

void JetLegPointCloudProc::convert_heightmap(std::vector<glm::vec3> cloud_array) {

  float floor_height = Z_MAX;

  std::vector<glm::vec3> filtered_cloud;
  for(unsigned int i = 0; i < cloud_array.size(); i++) {

    // Applies X restrictions
    if(cloud_array[i].x <= X_MAX && cloud_array[i].x > X_MIN) {

      // Applies Y restrictions
      if(cloud_array[i].y <= Y_MAX && cloud_array[i].y >= Y_MIN) {

        // Applies Z restriction
        if(cloud_array[i].z <= cloud_array[i].x* tan(Z_MAX)) {

          // Add points that fit all restrictions
          filtered_cloud.push_back(cloud_array[i]);
        }
      }
    }
  }

  int idx_x = 0;
  int idx_y = 0;

  float x_range = (X_MAX - X_MIN) / (MAP_ROWS - 1);
  float y_range = (Y_MAX - Y_MIN) / (MAP_COLS - 1);

  const float FLOATING_POINT_THRESHOLD = 0.0001f;

  for(int i = 0; i < MAP_ROWS; i++) {
    for(int j = 0; j < MAP_COLS; j++) {
      heightmap.at<float>(i, j) = 0.0f;
    }
  }

  for(unsigned int i = 0; i < filtered_cloud.size(); i++) {
    idx_x = (filtered_cloud[i].x - X_MIN) / x_range;
    idx_y = (filtered_cloud[i].y - Y_MIN) / y_range;

    if(close_to(heightmap.at<float>(idx_x, idx_y), 0.0f, FLOATING_POINT_THRESHOLD)) {
      heightmap.at<float>(idx_x, idx_y) = filtered_cloud[i].z;

      if(heightmap.at<float>(idx_x, idx_y) < floor_height) {
        floor_height = heightmap.at<float>(idx_x, idx_y);
      }
    } else {
      heightmap.at<float>(idx_x, idx_y) = std::max(filtered_cloud[i].z, heightmap.at<float>(idx_x, idx_y));

      if(heightmap.at<float>(idx_x, idx_y) < floor_height) {
        floor_height = heightmap.at<float>(idx_x, idx_y);
      }
    }
  }

  for(int i = 0; i < MAP_ROWS; i++) {
    for(int j = 0; j < MAP_COLS; j++) {
      if(close_to(heightmap.at<float>(i, j), 0.0f, FLOATING_POINT_THRESHOLD)) {
        heightmap.at<float>(i, j) = -10.0f;
        continue;
      }

      heightmap.at<float>(i, j) = heightmap.at<float>(i, j) - floor_height;
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
  cv::dilate(heightmap, processed_heightmap, kernel, cv::Point(-1, -1), 2);
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

bool JetLegPointCloudProc::close_to(float a, float b, float threshold) {
  if(std::abs(a - b) < threshold) {
    return true;
  }

  return false;
}

/**
 * Converts point from camera space to world space using camera orientation and translation
 * 
 * @param index index of the point in the flatData vector
 */
void JetLegPointCloudProc::convertToWorldFramePoint(std::vector<glm::vec3> &cloud_array, unsigned int index) {
    cloud_array[index] = glm::rotateZ(cloud_array[index], orientation.z) - position;
}

/**
 * Handles subscription to camera state topic and 
 * updates camera translation and orientation at each call
 * 
 * @param msg pointer to PoseStamped message
 */
void JetLegPointCloudProc::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  
  //Stores rotation and translation transformations of simulated camera
  position.x = msg->pose.position.x;
  position.y = msg->pose.position.y;
  position.z = msg->pose.position.z;

  orientation.x = 0.0f;
  orientation.y = 0.0f;
  orientation.z = -msg->pose.orientation.z;
  orientation.w = msg->pose.orientation.w;
}