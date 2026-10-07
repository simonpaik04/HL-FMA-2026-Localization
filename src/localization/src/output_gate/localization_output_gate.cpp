#include "localization_output_gate.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "common/message_validation.hpp"

namespace mando_localization {
namespace {

template <typename T>
T requireParameter(const ros::NodeHandle& node, const std::string& name) {
  T value;
  if (!node.getParam(name, value)) {
    throw std::runtime_error("필수 파라미터가 없습니다: " + node.resolveName(name));
  }
  return value;
}

}  // namespace

LocalizationOutputGate::LocalizationOutputGate(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(std::move(nh)), private_nh_(std::move(private_nh)) {
  const std::string input_topic =
      requireParameter<std::string>(private_nh_, "topics/global_odometry");
  const std::string valid_topic = requireParameter<std::string>(private_nh_, "topics/valid");
  const std::string output_topic =
      requireParameter<std::string>(private_nh_, "topics/output_odometry");
  map_frame_ = requireParameter<std::string>(private_nh_, "frames/map");
  base_frame_ = requireParameter<std::string>(private_nh_, "frames/base_link");
  max_odometry_age_sec_ =
      requireParameter<double>(private_nh_, "output_gate/max_odometry_age_sec");
  max_valid_age_sec_ =
      requireParameter<double>(private_nh_, "output_gate/max_valid_age_sec");
  max_future_stamp_sec_ =
      requireParameter<double>(private_nh_, "output_gate/max_future_stamp_sec");
  max_position_variance_m2_ =
      requireParameter<double>(private_nh_, "quality/max_position_variance_m2");
  max_quaternion_error_ = requireParameter<double>(
      private_nh_, "output_gate/max_quaternion_normalization_error");
  max_covariance_diagonal_ = requireParameter<double>(
      private_nh_, "output_gate/max_covariance_diagonal");
  const bool require_finite_values = requireParameter<bool>(
      private_nh_, "output_gate/require_finite_values");
  const bool require_map_frame = requireParameter<bool>(
      private_nh_, "output_gate/require_map_frame");
  const bool publish_last_pose_when_invalid = requireParameter<bool>(
      private_nh_, "output_gate/publish_last_pose_when_invalid");
  private_nh_.param("output_gate/allow_unbounded_position_variance",
                    allow_unbounded_position_variance_, false);
  private_nh_.param("initialization/required", initialization_required_, false);
  private_nh_.param("initialization/ready_timeout_sec",
                    initialization_ready_timeout_sec_, 0.5);
  if (map_frame_.empty() || base_frame_.empty() || !std::isfinite(max_odometry_age_sec_) ||
      max_odometry_age_sec_ <= 0.0 || !std::isfinite(max_valid_age_sec_) ||
      max_valid_age_sec_ <= 0.0 || !std::isfinite(max_future_stamp_sec_) ||
      max_future_stamp_sec_ < 0.0 || !std::isfinite(max_position_variance_m2_) ||
      max_position_variance_m2_ <= 0.0 || !std::isfinite(max_quaternion_error_) ||
      max_quaternion_error_ <= 0.0 || !std::isfinite(max_covariance_diagonal_) ||
      max_covariance_diagonal_ <= 0.0 || !require_finite_values ||
      !require_map_frame || publish_last_pose_when_invalid ||
      !std::isfinite(initialization_ready_timeout_sec_) ||
      initialization_ready_timeout_sec_ <= 0.0) {
    throw std::runtime_error("Output Gate frame 또는 timeout 설정이 유효하지 않습니다.");
  }

  valid_subscriber_ =
      nh_.subscribe(valid_topic, 10, &LocalizationOutputGate::validCallback, this);
  odometry_subscriber_ =
      nh_.subscribe(input_topic, 50, &LocalizationOutputGate::odometryCallback, this);
  output_publisher_ = nh_.advertise<nav_msgs::Odometry>(output_topic, 20);
  if (initialization_required_) {
    initialization_ready_subscriber_ = nh_.subscribe(
        "/mando_localization/internal/initialization/ready", 10,
        &LocalizationOutputGate::initializationReadyCallback, this);
  }

  ROS_INFO_STREAM("LocalizationOutputGate 설정: " << input_topic << " -> " << output_topic
                  << ", frame=" << map_frame_ << "/" << base_frame_);
}

bool LocalizationOutputGate::validateOdometry(const nav_msgs::Odometry& message,
                                              const std::string& expected_parent_frame,
                                              const std::string& expected_child_frame,
                                              const ros::Time& now,
                                              const double max_age_sec) {
  return validateOdometry(message, expected_parent_frame, expected_child_frame,
                          now, max_age_sec, 0.1, 1.0e12, 1.0e-3,
                          1.0e12);
}

bool LocalizationOutputGate::validateOdometry(
    const nav_msgs::Odometry& message,
    const std::string& expected_parent_frame,
    const std::string& expected_child_frame, const ros::Time& now,
    const double max_age_sec, const double max_future_sec,
    const double max_position_variance_m2,
    const double max_quaternion_error,
    const double max_covariance_diagonal) {
  if (message.header.stamp.isZero() ||
      message.header.frame_id != expected_parent_frame ||
      message.child_frame_id != expected_child_frame ||
      !std::isfinite(message.pose.pose.position.x) ||
      !std::isfinite(message.pose.pose.position.y) ||
      !std::isfinite(message.pose.pose.position.z)) {
    return false;
  }
  const double age = (now - message.header.stamp).toSec();
  if (!std::isfinite(age) || age < -max_future_sec || age > max_age_sec) {
    return false;
  }
  geometry_msgs::Quaternion normalized;
  std::string reason;
  if (!MessageValidation::normalizeQuaternion(
          message.pose.pose.orientation, 1.0e-12, max_quaternion_error,
          &normalized, &reason) ||
      !MessageValidation::validateCovariance(
          message.pose.covariance.data(), 6, false,
          max_covariance_diagonal, &reason) ||
      message.pose.covariance[0] > max_position_variance_m2 ||
      message.pose.covariance[7] > max_position_variance_m2 ||
      !MessageValidation::validateCovariance(
          message.twist.covariance.data(), 6, false,
          max_covariance_diagonal, &reason)) {
    return false;
  }
  const geometry_msgs::Twist& twist = message.twist.twist;
  const double twist_values[] = {
      twist.linear.x,  twist.linear.y,  twist.linear.z,
      twist.angular.x, twist.angular.y, twist.angular.z};
  for (const double value : twist_values) {
    if (!std::isfinite(value)) return false;
  }
  return true;
}

void LocalizationOutputGate::validCallback(const std_msgs::BoolConstPtr& message) {
  valid_ = message->data && initializationReady(ros::Time::now());
  valid_receipt_time_ = ros::Time::now();
}

void LocalizationOutputGate::initializationReadyCallback(
    const std_msgs::BoolConstPtr& message) {
  initialization_ready_ = message->data;
  initialization_ready_receipt_time_ = ros::Time::now();
  initialization_ready_wall_time_ = ros::WallTime::now();
  if (!message->data) {
    valid_ = false;
    valid_receipt_time_ = ros::Time();
  }
}

bool LocalizationOutputGate::initializationReady(const ros::Time& now) const {
  if (!initialization_required_) return true;
  if (!initialization_ready_ || initialization_ready_receipt_time_.isZero() ||
      initialization_ready_wall_time_.isZero()) return false;
  const double age = (now - initialization_ready_receipt_time_).toSec();
  const double wall_age = (ros::WallTime::now() - initialization_ready_wall_time_).toSec();
  return age >= 0.0 && age <= initialization_ready_timeout_sec_ &&
         wall_age >= 0.0 && wall_age <= initialization_ready_timeout_sec_;
}

void LocalizationOutputGate::odometryCallback(const nav_msgs::OdometryConstPtr& message) {
  const ros::Time now = ros::Time::now();
  if (!initializationReady(now)) {
    ROS_WARN_THROTTLE(2.0, "LocalizationOutputGate: RDDF 초기화 승인을 기다립니다.");
    return;
  }
  const double valid_age = (now - valid_receipt_time_).toSec();
  if (!valid_ || valid_receipt_time_.isZero() ||
      !std::isfinite(valid_age) || valid_age < 0.0 ||
      valid_age > max_valid_age_sec_) {
    ROS_WARN_THROTTLE(2.0, "LocalizationOutputGate: 상태 승인이 없어 출력을 차단합니다.");
    return;
  }
  if (!validateOdometry(*message, map_frame_, base_frame_, now,
                        max_odometry_age_sec_, max_future_stamp_sec_,
                        allow_unbounded_position_variance_
                            ? max_covariance_diagonal_
                            : max_position_variance_m2_,
                        max_quaternion_error_,
                        max_covariance_diagonal_)) {
    ROS_ERROR_THROTTLE(2.0, "LocalizationOutputGate: Global Odometry 계약 위반입니다.");
    return;
  }
  output_publisher_.publish(message);
}

}  // namespace mando_localization
