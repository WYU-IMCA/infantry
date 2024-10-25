// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include "buff_detector/detector_node.hpp"

namespace rm_buff
{
BuffDetectorNode::BuffDetectorNode(const rclcpp::NodeOptions & options)
: Node("buff_detector", options)
{
  blades_publisher_ =
    this->create_publisher<buff_interfaces::msg::BladeArray>("detector/blade_array", 10);
  debug_blades_publisher_ =
    this->create_publisher<buff_interfaces::msg::DebugBladeArray>("debug/blade_array", 10);
  latency_publisher_ = this->create_publisher<std_msgs::msg::String>("inference/latency", 10);
  result_img_pub_ = image_transport::create_publisher(this, "detector/buff_result_img");
  bin_img_pub_ = image_transport::create_publisher(this, "detector/buff_bin_img");
  debug_img_pub_ = image_transport::create_publisher(this, "debug/buff_debug_img");

  auto pkg_path = ament_index_cpp::get_package_share_directory("buff_detector");
  std::string model_name = this->declare_parameter("detector.model", "buff_quantized.xml");
  auto model_path = pkg_path + "/models/" + model_name;
  detector_ = std::make_unique<Detector>(model_path);
  RCLCPP_INFO(this->get_logger(), "Model loaded: %s", model_path.c_str());

  // Task subscriber
  is_buff_task_ = false;

  set_rune_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
    "rune_detector/set_mode",
    std::bind(
      &BuffDetectorNode::setModeCallback, this, std::placeholders::_1, std::placeholders::_2));

  // param
  detector_->nms_threshold = this->declare_parameter("detector.nms_threshold", 0.05);
  detector_->conf_threshold = this->declare_parameter("detector.conf_threshold", 0.70);
  detector_->image_size = this->declare_parameter("detector.image_size", 640);
  detector_->bin_threshold = this->declare_parameter("detector.bin_threshold", 70.0);
  detector_->fault_tolerance_ratio =
    this->declare_parameter("detector.fault_tolerance_ratio", 0.065);

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
      cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
      pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
      cam_info_sub_.reset();
    });

  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", rclcpp::SensorDataQoS(),
    std::bind(&BuffDetectorNode::imageCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Detector node initialized");
}

void BuffDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  if (pnp_solver_ != nullptr && is_buff_task_) {
    auto blades = DetectBlades(msg);
    buff_interfaces::msg::BladeArray blade_array;
    blade_array.header = msg->header;
    blade_array.header.frame_id = "camera_optical_frame";
    buff_interfaces::msg::DebugBladeArray debug_blade_array;

    // publish blade array
    for (auto & blade : blades) {
      buff_interfaces::msg::DebugBlade debug_blade_msg;
      debug_blade_msg.x = blade.rect.x;
      debug_blade_msg.y = blade.rect.y;
      debug_blade_msg.width = blade.rect.width;
      debug_blade_msg.height = blade.rect.height;
      debug_blade_msg.label = blade.label;
      debug_blade_msg.prob = blade.prob;
      for (auto & kpt : blade.kpt) {
        geometry_msgs::msg::Point point;
        point.x = kpt.x;
        point.y = kpt.y;
        debug_blade_msg.kpt.emplace_back(point);
      }
      // solve pnp
      cv::Mat rvec, tvec;
      if (pnp_solver_->solvePnP(blade, rvec, tvec)) {
        buff_interfaces::msg::Blade blade_msg;
        blade_msg.pose.position.x = tvec.at<double>(0);
        blade_msg.pose.position.y = tvec.at<double>(1);
        blade_msg.pose.position.z = tvec.at<double>(2);

        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);

        tf2::Matrix3x3 tf_rotation_matrix(
          rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
          rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
          rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
          rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
          rotation_matrix.at<double>(2, 2));
        tf2::Quaternion tf_quaternion;
        tf_rotation_matrix.getRotation(tf_quaternion);
        blade_msg.pose.orientation = tf2::toMsg(tf_quaternion);
        blade_msg.label = blade.label;
        blade_msg.prob = blade.prob;
        debug_blade_msg.pose = blade_msg.pose;
        blade_array.blades.emplace_back(blade_msg);

        geometry_msgs::msg::Point center;
        center.x = blade.kpt[2].x;
        center.y = blade.kpt[2].y;
        debug_blade_msg.center = center;
      } else {
        RCLCPP_WARN(this->get_logger(), "PnP failed");
      }
      debug_blade_array.blades.emplace_back(debug_blade_msg);
    }
    blades_publisher_->publish(blade_array);
    debug_blades_publisher_->publish(debug_blade_array);
  }
}

std::vector<Blade> BuffDetectorNode::DetectBlades(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{
  auto img = cv_bridge::toCvShare(image_msg, "rgb8")->image;

  // copy
  cv::Mat img_copy = img.clone();

  // start time
  auto start = std::chrono::steady_clock::now();
  std::vector<Blade> result = detector_->Detect(img_copy);

  // end time
  auto end = std::chrono::steady_clock::now();

  auto final_time = this->now();
  auto latency = (final_time - image_msg->header.stamp).seconds() * 1000;

  // publish
  auto time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  std_msgs::msg::String latency_msg;
  latency_msg.data = std::to_string(time.count());
  latency_publisher_->publish(latency_msg);
  if (result.size() == 0) {
    RCLCPP_DEBUG(this->get_logger(), "No blade detected");
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Blade detected");
  }
  // Draw camera center
  cv::circle(img_copy, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
  // draw blade
  detector_->draw_blade(img_copy);
  // Draw latency
  std::stringstream latency_ss;
  latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
  auto latency_s = latency_ss.str();
  cv::putText(
    img_copy, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0),
    2);
  result_img_pub_.publish(cv_bridge::CvImage(image_msg->header, "rgb8", img_copy).toImageMsg());

  bin_img_pub_.publish(
    cv_bridge::CvImage(image_msg->header, "mono8", detector_->binary_img).toImageMsg());
  debug_img_pub_.publish(
    cv_bridge::CvImage(image_msg->header, "rgb8", detector_->debug_img).toImageMsg());

  return result;
}

void BuffDetectorNode::setModeCallback(
  const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
  std::shared_ptr<rm_interfaces::srv::SetMode::Response> response)
{
  response->success = true;

  imca::VisionMode mode = static_cast<imca::VisionMode>(request->mode);
  std::string mode_name = imca::visionModeToString(mode);
  if (mode_name == "UNKNOWN") {
    return;
  }
  switch (mode) {
    case imca::VisionMode::SMALL_RUNE: {
      is_buff_task_ = true;
      RCLCPP_INFO(get_logger(), "Create buff_detector");
      break;
    }
    case imca::VisionMode::BIG_RUNE: {
      is_buff_task_ = true;
      RCLCPP_INFO(get_logger(), "Create buff_detector");
      break;
    }
    default: {
      is_buff_task_ = false;
      break;
    }
  }
}

}  // namespace rm_buff

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_buff::BuffDetectorNode)