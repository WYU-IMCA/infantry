#ifndef GIMBAL_SOLVER_HPP_
#define GIMBAL_SOLVER_HPP_

//c++ std
#include <memory>

//ros2
#include <tf2_ros/buffer.h>

#include <rclcpp/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
//msg
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "buff_interfaces/msg/rune.hpp"

// 3rd party
#include <Eigen/Dense>

//tools
#include <rm_utils/math/trajectory_compensator.hpp>

namespace rm_buff
{
class BuffSolver
{
public:
  explicit BuffSolver(std::weak_ptr<rclcpp::Node> n);
  ~BuffSolver() = default;
  rm_interfaces::msg::GimbalCmd solve(
    const buff_interfaces::msg::Rune & msg, const rclcpp::Time current_time,
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer);
  std::vector<std::pair<double, double>> getTrajectory(double distance, double angle) const noexcept
  {
    return trajectory_compensator_->getTrajectory(distance, angle);
  }
  Eigen::Vector3d getPridictTarget(){
    return predict_target_;
  }
  rclcpp::Time getPredictTimestamp(){
    return predict_timestamp_;
  }

private:
  std::unique_ptr<fyt::TrajectoryCompensator> trajectory_compensator_;
  void calcYawAndPitch(
    const Eigen::Vector3d & p, const std::array<double, 3> rpy, double & yaw,
    double & pitch) const noexcept;
  double shooting_range_pitch_;
  double shooting_range_yaw_;
  double prediction_delay_;
  double controller_delay_;
  std::weak_ptr<rclcpp::Node> node_;
  Eigen::Vector3d predict_target_;
  rclcpp::Time predict_timestamp_;
};

}  // namespace rm_buff
#endif