#include "jetleg_pointcloud_proc.hpp"

JetLegPointCloudProc::JetLegPointCloudProc() : rclcpp::Node("jetleg_pointcloud_proc") {
  RCLCPP_INFO(this->get_logger(), "jetleg_pointcloud_proc node has been created...");

  // Generates wrapper for the pointcloud generation callback
  auto cloudWrap = std::bind(&JetLegPointCloudProc::pointcloudSubCallback, this, std::placeholders::_1);
  // Generates wrapper for the pose update callback
  auto poseWrap = std::bind(&JetLegPointCloudProc::poseStampedSubCallback, this, std::placeholders::_1);

  // Creates subscribers and publishers
  subPointCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>("pointcloud", rclcpp::SensorDataQoS(), cloudWrap);
  subPoseStamped = this->create_subscription<geometry_msgs::msg::PoseStamped>("camera_state", QOS, poseWrap);

  heightmapPub = this->create_publisher<sensor_msgs::msg::Image>("heightmap", QOS);
  traversePub = this->create_publisher<sensor_msgs::msg::Image>("traversibility", QOS);

  // Stores output as floats
  heightmap = cv::Mat(MAP_ROWS, MAP_COLS, CV_32FC1);
  processedHeightmap = cv::Mat(MAP_ROWS, MAP_COLS, CV_32FC1);
  traversibilityMap = cv::Mat(MAP_ROWS, MAP_COLS, CV_8UC1);
  gradientMap = cv::Mat(MAP_ROWS, MAP_COLS, CV_32FC1);

  // Stores outputs as bytes to be compatible with RVIZ
  heightmapInBytes = cv::Mat(MAP_ROWS, MAP_COLS, CV_8UC1);
  traversibilityInBytes = cv::Mat(MAP_ROWS, MAP_COLS, CV_8UC1);
}

JetLegPointCloudProc::~JetLegPointCloudProc() {
  
}

void JetLegPointCloudProc::pointcloudSubCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // auto timeStart = std::chrono::steady_clock::now();

  std::vector<glm::vec4> cloudArray(msg->data.size() / STEP_SIZE);
    
  // Convert from byte array to float array of structure XYZ
  loadData(cloudArray, msg);

  //Generate heightmap
  convertHeightmap(cloudArray);
  //Generate traversibility map
  computeTraversibility(processedHeightmap, traversibilityMap);

  // Publishes heightmap to visualize with RVIZ
  publishImage(heightmapPub, processedHeightmap, heightmapInBytes, Y_MIN, Y_MAX);
  // Publishes traversibility map to visualize with RVIZ
  publishImage(traversePub, traversibilityMap, traversibilityInBytes, 0.0f, 4.0f, 63.9);

  // auto timeEnd = std::chrono::steady_clock::now();
  // RCLCPP_INFO(this->get_logger(), "Time (s) per Tick: " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(timeEnd - timeStart).count() / 1000.0f));
}

void JetLegPointCloudProc::loadData(std::vector<glm::vec4> &data, const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

  //Copy byte data into vector of glm::vec4 float data
  memcpy(&data[0], &msg->data[0], msg->data.size());

  // Transforms points
  // for(unsigned int i = 0; i < data.size(); i++) {    
  //   convertToWorldFramePoint(data, i);
  // }
}

void JetLegPointCloudProc::convertHeightmap(const std::vector<glm::vec4> &cloud_array) {

  // Set initial floor height to maximum possible value
  float floor_height = Z_MAX;

  std::vector<glm::vec4> filtered_cloud;
  for(unsigned int i = 0; i < cloud_array.size(); i++) {

    // Applies X restrictions
    if(cloud_array[i].x < X_MAX && cloud_array[i].x > X_MIN) {

      // Applies Y restrictions
      if(cloud_array[i].y < Y_MAX && cloud_array[i].y > Y_MIN) {

        // Applies Z restriction
        if(cloud_array[i].z < Z_MAX && cloud_array[i].z > Z_MIN) {

          // Add points that fit all restrictions
          filtered_cloud.push_back(cloud_array[i]);
        }
      }
    }
  }

  int idx_x = 0;
  int idx_y = 0;

  float x_range = (X_MAX - X_MIN) / (MAP_COLS - 1);
  float z_range = (Z_MAX - 0) / (MAP_ROWS - 1);

  for(unsigned int i = 0; i < MAP_ROWS; i++) {
    for(unsigned int j = 0; j < MAP_COLS; j++) {
      heightmap.at<float>(i, j) = Y_MIN;
    }
  }

  for(unsigned int i = 0; i < filtered_cloud.size(); i++) {
    idx_x = (filtered_cloud[i].x - X_MIN) / x_range;
    idx_y = (filtered_cloud[i].z - Z_MIN) / z_range;

    heightmap.at<float>(idx_y, idx_x) = std::max(filtered_cloud[i].y, heightmap.at<float>(idx_y, idx_x));

    if(heightmap.at<float>(idx_y, idx_x) < floor_height) {
      floor_height = heightmap.at<float>(idx_y, idx_x);
    }
  }

  for(unsigned int i = 0; i < MAP_ROWS; i++) {
    for(unsigned int j = 0; j < MAP_COLS; j++) {

      // heightmap.at<float>(i, j) = heightmap.at<float>(i, j) - floor_height;
    }
  }

  // Kernel used to process raw heightmap
  cv::Mat kernel(KERNEL_SIZE, KERNEL_SIZE, CV_8UC1);

  kernel.col(0) = (cv::Mat_<uint8_t>(1, KERNEL_SIZE) << 0, 0, 1, 0, 0);
  kernel.col(1) = (cv::Mat_<uint8_t>(1, KERNEL_SIZE) << 1, 1, 1, 1, 1);
  kernel.col(2) = (cv::Mat_<uint8_t>(1, KERNEL_SIZE) << 1, 1, 1, 1, 1);
  kernel.col(3) = (cv::Mat_<uint8_t>(1, KERNEL_SIZE) << 1, 1, 1, 1, 1);
  kernel.col(4) = (cv::Mat_<uint8_t>(1, KERNEL_SIZE) << 0, 0, 1, 0, 0);

  // Applies image processing
  cv::dilate(heightmap, processedHeightmap, kernel, cv::Point(-1, -1), 2);
  cv::flip(processedHeightmap, processedHeightmap, 0);
} 

void JetLegPointCloudProc::publishImage(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub, const cv::Mat &src, cv::Mat &out, float min, float max, float scale) {

  float range = max - min;

  // Scales image using "scale" variable
  if(src.type() == CV_32FC1) {
    for(int i = 0; i < src.size().height; i++) {
      for(int j = 0; j < src.size().width; j++) {
        out.at<uchar>(i, j) = (src.at<float>(i, j) - min) / range * scale;
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
  cv_bridge::CvImage imgBridge = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_8UC1, out);
  sensor_msgs::msg::Image imgMsg;

  // Publishes to topic
  imgBridge.toImageMsg(imgMsg);
  pub->publish(imgMsg);
}

void JetLegPointCloudProc::computeTraversibility(cv::Mat &heightmap, cv::Mat &traversibility_map) {  
  cv::Mat sobel_x;
  cv::Mat sobel_y;

  cv::Sobel(heightmap, sobel_x, CV_64F, 1, 0, 5);
  cv::Sobel(heightmap, sobel_y, CV_64F, 0, 1, 5);

  for(unsigned int i = 0; i < MAP_ROWS; i++) {
    for(unsigned int j = 0; j < MAP_COLS; j++) {
      float x_grad = sobel_x.at<double>(i, j);
      float y_grad = sobel_y.at<double>(i, j);

      float max_grad = std::max(std::abs(x_grad), std::abs(y_grad));
      gradientMap.at<float>(i, j) = max_grad;
    }
  }

  for(unsigned int i = 0; i < MAP_ROWS; i++) {
    for(unsigned int j = 0; j < MAP_COLS; j++) {
      if(abs(heightmap.at<float>(i, j)) < FLOATING_POINT_THRESHOLD) {
        traversibility_map.at<uchar>(i, j) = (uchar)4;

      } else if(gradientMap.at<float>(i, j) < 5) {
        traversibility_map.at<uchar>(i, j) = (uchar)1;

      } else if(gradientMap.at<float>(i, j) < 20) {
        traversibility_map.at<uchar>(i, j) = (uchar)2;

      } else if(gradientMap.at<float>(i, j) < 35) {
        traversibility_map.at<uchar>(i, j) = (uchar)4;

      } else if(gradientMap.at<float>(i, j) < 50) {
        traversibility_map.at<uchar>(i, j) = (uchar)4;

      } else {
        traversibility_map.at<uchar>(i, j) = (uchar)0;
      }
    }
  }
}

void JetLegPointCloudProc::printInfo(std::string msg) {
  RCLCPP_INFO(this->get_logger(), msg.c_str());
}

bool JetLegPointCloudProc::closeTo(float a, float b, float threshold) {

  if(std::abs(a - b) < threshold) {
    return true;
  }

  return false;
}

/**
 * @brief Converts point from camera space to world space using camera orientation and translation
 * 
 * @param index index of the point in the flatData vector
 */
void JetLegPointCloudProc::convertToWorldFramePoint(std::vector<glm::vec4> &cloudArray, unsigned int index) {
    glm::rotateZ(cloudArray[index], eulerAngles.z);
    cloudArray[index] -= position;
}

/**
 * @brief Handles subscription to camera state topic and 
 *        updates camera translation and orientation at each call
 * 
 * @param msg pointer to PoseStamped message
 */
void JetLegPointCloudProc::poseStampedSubCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  
  //Stores rotation and translation transformations of simulated camera
  position.x = msg->pose.position.x;
  position.y = msg->pose.position.y;
  position.z = msg->pose.position.z;

  orientation.x = msg->pose.orientation.x;
  orientation.y = msg->pose.orientation.y;
  orientation.z = msg->pose.orientation.z;
  orientation.w = msg->pose.orientation.w;

  eulerAngles = glm::eulerAngles(orientation);
  eulerAngles.x = 0.0f;
  eulerAngles.y = 0.0f;
  eulerAngles.z = -eulerAngles.z;
}