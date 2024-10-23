#include "buff_tracker/gimbal_solver.hpp"
namespace rm_buff
{
BuffSolver::BuffSolver(std::weak_ptr<rclcpp::Node> n) : node_(n)
{
  auto node = node_.lock();
  prediction_delay_ = node->declare_parameter("solver.prediction_delay", 0.0);

  std::string compenstator_type = node->declare_parameter("solver.compensator_type", "ideal");
  trajectory_compensator_ = imca::CompensatorFactory::createCompensator(compenstator_type);
  trajectory_compensator_->iteration_times = node->declare_parameter("solver.iteration_times", 20);
  trajectory_compensator_->velocity = node->declare_parameter("solver.bullet_speed", 20.0);
  trajectory_compensator_->gravity = node->declare_parameter("solver.gravity", 9.8);
  trajectory_compensator_->resistance = node->declare_parameter("solver.resistance", 0.001);
}

rm_interfaces::msg::GimbalCmd BuffSolver::solve(
  const buff_interfaces::msg::Rune & target, const rclcpp::Time current_time,
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_)
{
  try {
    auto node = node_.lock();
    controller_delay_ = node->get_parameter("solver.controller_delay").as_double();
    prediction_delay_ = node->get_parameter("solver.prediction_delay").as_double();

    RCLCPP_INFO(node->get_logger(),"getting params");
    node.reset();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(rclcpp::get_logger("buff_gimbal_error"), "%s", e.what());
  }
 

  //predict twice
  Eigen::Vector3d target_position(target.position.x, target.position.y, target.position.z);
 
  double flying_time = trajectory_compensator_->getFlyingTime(target_position);

  const double t0 = rclcpp::Time(target.header.stamp).seconds();
  const double t1 = current_time.seconds() + flying_time + prediction_delay_;
  predict_timestamp_ = current_time + rclcpp::Duration::from_seconds(flying_time) +
                                   rclcpp::Duration::from_seconds(prediction_delay_);


  const double & r = target.r;
  double theta = target.theta;
  //small buff
  if (target.w == 0) {
    theta += target.b * (t1 - t0);
  }
  //large buff
  else {
    theta += target.a / target.w * (cos(target.w * t0) - cos(target.w * t1)) + target.b * (t1 - t0);
  }

  const double & xc = target.position.x;
  const double & yc = target.position.y;
  const double & zc = target.position.z;

  double target_x = xc + r * (sin(theta) * yc / sqrt(pow(xc, 2) + pow(yc, 2)));
  double target_y = yc + r * (-sin(theta) * xc / sqrt(pow(xc, 2) + pow(yc, 2)));
  double target_z = zc + r * cos(theta);

  geometry_msgs::msg::PointStamped target_msg;  //predict pose
  target_msg.point.x = target_x;
  target_msg.point.y = target_y;
  target_msg.point.z = target_z;
  target_msg.header.stamp = predict_timestamp_;

  auto target_msg_pub =
    node_.lock()->create_publisher<geometry_msgs::msg::PointStamped>("buff/predict_point", 10);
  target_msg_pub->publish(target_msg);

  //solve gimbal cmd

  predict_target_=Eigen::Vector3d(target_x, target_y, target_z);
  //const Eigen::Vector3d predict_target(target_x, target_y, target_z);

  rm_interfaces::msg::GimbalCmd control_msg;
  double current_yaw = 0.0, current_pitch = 0.0;
  try {
    auto gimbal_tf = tf2_buffer_->lookupTransform("odom", "gimbal_link", tf2::TimePointZero);
    auto msg_q = gimbal_tf.transform.rotation;
    tf2::Quaternion tf_q;
    tf2::fromMsg(msg_q, tf_q);
    double roll;
    tf2::Matrix3x3(tf_q).getRPY(roll, current_pitch, current_yaw);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("buff_gimbal_solver"), "%s", ex.what());
    throw ex;
  }

  // Calculate yaw and pitch

  double yaw = atan2(predict_target_.y(), predict_target_.x());
  double pitch = atan2(predict_target_.z(), predict_target_.head(2).norm());

  if (double temp_pitch = pitch; trajectory_compensator_->compensate(predict_target_, temp_pitch)) {
    pitch = temp_pitch;
  }
  double distance = predict_target_.norm();
  control_msg.yaw = yaw * 180 / M_PI;
  control_msg.pitch = pitch * 180 / M_PI;
  control_msg.yaw_diff = (yaw - current_yaw) * 180 / M_PI;
  control_msg.pitch_diff = (-pitch - current_pitch) * 180 / M_PI;
  control_msg.distance = distance;
  control_msg.header=target_msg.header;

  // Judge whether to shoot
  constexpr double TARGET_RADIUS = 0.308;
  double shooting_range_yaw = std::abs(atan2(TARGET_RADIUS / 2, distance)) * 180 / M_PI;
  double shooting_range_pitch = std::abs(atan2(TARGET_RADIUS / 2, distance)) * 180 / M_PI;

  // Limit the shooting area to 1 degree to avoid not shooting when distance is
  // too large
  shooting_range_yaw = std::max(shooting_range_yaw, 1.0);
  shooting_range_pitch = std::max(shooting_range_pitch, 1.0);
  std::cout<<"shooting_range_yaw:   "<<shooting_range_yaw<<"shooting_range_pitch:   "<<shooting_range_pitch<<std::endl;
  if (
    std::abs(control_msg.yaw_diff) < shooting_range_yaw &&
    std::abs(control_msg.pitch_diff) < shooting_range_pitch) {
    control_msg.fire_advice = true;
    RCLCPP_DEBUG(rclcpp::get_logger("buff_gimbal_solver"), "You Can Fire!");
  } else {
    control_msg.fire_advice = false;
  }

  return control_msg;
}

void BuffSolver::calcYawAndPitch(
  const Eigen::Vector3d & p, const std::array<double, 3> rpy, double & yaw,
  double & pitch) const noexcept
{
  // Calculate yaw and pitch
  yaw = atan2(p.y(), p.x());
  pitch = atan2(p.z(), p.head(2).norm());

  if (double temp_pitch = pitch; trajectory_compensator_->compensate(p, temp_pitch)) {
    pitch = temp_pitch;
  }
}
}  // namespace rm_buff
