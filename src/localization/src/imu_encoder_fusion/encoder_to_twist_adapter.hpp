/*
encoder_to_twist_adapter.hpp
- 역할: 차량 엔코더 속도를 robot_localization이 소비할 수 있는
        geometry_msgs/TwistWithCovarianceStamped로 변환한다.
- 입력 단위: SerialFeedBack.speed는 m/s, encoder는 최근 100 ms 증분이다.
- 출력 단위/frame: linear.x m/s, base_link frame. 활성 시 비홀로노믹
                   linear.y=0 제약을 함께 제공하고 yaw rate는 IMU가 담당한다.
- 실패 경로: alive counter 정체, NaN, 과도한 속도나 encoder delta는 발행하지 않는다.
*/
#pragma once

#include <cstdint>
#include <string>

#include <erp42_msgs/SerialFeedBack.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <ros/ros.h>

namespace mando_localization {

class EncoderToTwistAdapter {
 public:
  EncoderToTwistAdapter(const ros::NodeHandle& node,
                        const ros::NodeHandle& private_node);
  static bool hasUsableSpeed(const erp42_msgs::SerialFeedBack& message);
  static bool aliveCounterAdvanced(uint8_t current, bool have_previous,
                                   uint8_t previous);
  static geometry_msgs::TwistWithCovarianceStamped convertSpeedToTwist(
      const erp42_msgs::SerialFeedBack& message, const ros::Time& receipt_stamp,
      const std::string& base_link_frame,
      double speed_scale, int direction_sign, double speed_variance,
      double lateral_velocity_variance, double unobserved_variance);

 private:
  void load_configuration();
  void encoder_callback(const erp42_msgs::SerialFeedBack::ConstPtr& message);
  bool validate_measurement(const erp42_msgs::SerialFeedBack& message,
                            std::string* reason) const;
  geometry_msgs::TwistWithCovarianceStamped make_twist(
      const erp42_msgs::SerialFeedBack& message,
      const ros::Time& receipt_stamp) const;

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Publisher twist_publisher_;
  ros::Subscriber encoder_subscriber_;

  std::string encoder_topic_;
  std::string twist_topic_;
  std::string base_link_frame_;
  double speed_scale_ = 0.0;
  int direction_sign_ = 0;
  double max_abs_speed_mps_ = 0.0;
  int max_abs_encoder_delta_100ms_ = 0;
  double speed_variance_m2ps2_ = 0.0;
  double lateral_velocity_variance_m2ps2_ = 0.0;
  std::string lateral_velocity_calibration_state_;
  std::string lateral_velocity_source_;
  double unobserved_variance_ = 0.0;
  bool require_alive_counter_change_ = true;
  std::string calibration_state_;
  double encoder_delta_window_sec_ = 0.0;
  bool meter_per_tick_enabled_ = false;
  double meter_per_tick_m_ = 0.0;
  double delta_consistency_tolerance_mps_ = 0.0;
  bool steering_enabled_ = false;
  int steering_adc_center_ = 0;
  double steering_adc_per_rad_ = 0.0;
  int steering_adc_min_ = 0;
  int steering_adc_max_ = 0;
  bool have_last_alive_counter_ = false;
  uint8_t last_alive_counter_ = 0U;
};

}  // namespace mando_localization
