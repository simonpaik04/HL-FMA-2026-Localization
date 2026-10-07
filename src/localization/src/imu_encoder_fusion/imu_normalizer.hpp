/*
imu_normalizer.hpp
- 역할: IMU의 timestamp, imu_link frame, quaternion과 covariance를 검증하고
        안전하게 정규화한 sensor_msgs/Imu만 relay한다.
- 단위/frame: 입력·출력 모두 rad/s, m/s^2, configured imu frame이다.
- 실패 경로: frame 이름을 좌표 변환 없이 바꾸지 않으며 잘못된 sample은 폐기한다.
*/
#pragma once

#include <array>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace mando_localization {

class ImuNormalizer {
 public:
  ImuNormalizer(const ros::NodeHandle& node,
                const ros::NodeHandle& private_node);

 private:
  void load_configuration();
  void imu_callback(const sensor_msgs::Imu::ConstPtr& message);
  bool validate_and_normalize(const sensor_msgs::Imu& input,
                              sensor_msgs::Imu* output,
                              std::string* reason) const;

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Publisher imu_publisher_;
  ros::Subscriber imu_subscriber_;
  std::string input_topic_;
  std::string output_topic_;
  std::string imu_frame_;
  double max_message_age_sec_ = 0.0;
  double max_future_stamp_sec_ = 0.0;
  double min_quaternion_norm_ = 0.0;
  double max_quaternion_normalization_error_ = 0.0;
  double max_covariance_diagonal_ = 0.0;
  bool require_positive_covariance_diagonal_ = true;
  bool covariance_override_enabled_ = false;
  std::array<double, 3> acceleration_stddev_{{0.0, 0.0, 0.0}};
  std::array<double, 3> angular_velocity_stddev_{{0.0, 0.0, 0.0}};
  std::array<double, 3> orientation_stddev_{{0.0, 0.0, 0.0}};
  bool have_last_stamp_ = false;
  ros::Time last_stamp_;
};

}  // namespace mando_localization
