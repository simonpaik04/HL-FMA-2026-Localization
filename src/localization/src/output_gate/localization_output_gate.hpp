#pragma once

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <string>

namespace mando_localization {

/**
 * @brief 상태 관리자가 승인한 최신 Global Odometry만 최종 토픽으로 전달한다.
 *
 * 무효 상태에서 마지막 위치를 재발행하거나 0 위치를 만들지 않는다. 따라서
 * downstream watchdog은 실제 위치 공급 중단을 감지할 수 있다.
 */
class LocalizationOutputGate {
 public:
  LocalizationOutputGate(ros::NodeHandle nh, ros::NodeHandle private_nh);

  static bool validateOdometry(const nav_msgs::Odometry& message,
                               const std::string& expected_parent_frame,
                               const std::string& expected_child_frame,
                               const ros::Time& now, double max_age_sec);
  static bool validateOdometry(const nav_msgs::Odometry& message,
                               const std::string& expected_parent_frame,
                               const std::string& expected_child_frame,
                               const ros::Time& now, double max_age_sec,
                               double max_future_sec,
                               double max_position_variance_m2,
                               double max_quaternion_error,
                               double max_covariance_diagonal);

 private:
  void validCallback(const std_msgs::BoolConstPtr& message);
  void initializationReadyCallback(const std_msgs::BoolConstPtr& message);
  bool initializationReady(const ros::Time& now) const;
  void odometryCallback(const nav_msgs::OdometryConstPtr& message);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber valid_subscriber_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber initialization_ready_subscriber_;
  ros::Publisher output_publisher_;

  bool valid_{false};
  bool allow_unbounded_position_variance_{false};
  bool initialization_required_{false};
  bool initialization_ready_{false};
  double initialization_ready_timeout_sec_{0.5};
  ros::Time initialization_ready_receipt_time_;
  ros::WallTime initialization_ready_wall_time_;
  ros::Time valid_receipt_time_;
  std::string map_frame_;
  std::string base_frame_;
  double max_odometry_age_sec_{0.25};
  double max_valid_age_sec_{0.25};
  double max_future_stamp_sec_{0.5};
  double max_position_variance_m2_{25.0};
  double max_quaternion_error_{0.001};
  double max_covariance_diagonal_{1000000.0};
};

}  // namespace mando_localization
