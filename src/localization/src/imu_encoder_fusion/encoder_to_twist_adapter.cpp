/*
encoder_to_twist_adapter.cpp
- 역할: Arduino SerialFeedBack 검증, 전진 속도 전달과 공분산 구성을 구현한다.
- 좌표계: +X는 base_link 전방, +Z yaw는 반시계 방향이다.
*/
#include "imu_encoder_fusion/encoder_to_twist_adapter.hpp"

#include <algorithm>
#include <cstdint>
#include <cmath>

#include "common/message_validation.hpp"
#include "common/parameter_utils.hpp"

namespace mando_localization {

bool EncoderToTwistAdapter::hasUsableSpeed(
    const erp42_msgs::SerialFeedBack& message) {
  return std::isfinite(message.speed);
}

bool EncoderToTwistAdapter::aliveCounterAdvanced(
    const uint8_t current, const bool have_previous, const uint8_t previous) {
  return !have_previous || current != previous;
}

geometry_msgs::TwistWithCovarianceStamped
EncoderToTwistAdapter::convertSpeedToTwist(
    const erp42_msgs::SerialFeedBack& message,
    const ros::Time& receipt_stamp, const std::string& base_link_frame,
    const double speed_scale, const int direction_sign,
    const double speed_variance, const double lateral_velocity_variance,
    const double unobserved_variance) {
  geometry_msgs::TwistWithCovarianceStamped output;
  output.header.stamp = receipt_stamp;
  output.header.frame_id = base_link_frame;
  output.twist.twist.linear.x =
      message.speed * speed_scale * direction_sign;
  output.twist.twist.angular.z = 0.0;
  std::fill(output.twist.covariance.begin(), output.twist.covariance.end(),
            0.0);
  output.twist.covariance[0] = speed_variance;
  output.twist.covariance[7] = lateral_velocity_variance;
  output.twist.covariance[14] = unobserved_variance;
  output.twist.covariance[21] = unobserved_variance;
  output.twist.covariance[28] = unobserved_variance;
  output.twist.covariance[35] = unobserved_variance;
  return output;
}

EncoderToTwistAdapter::EncoderToTwistAdapter(
    const ros::NodeHandle& node, const ros::NodeHandle& private_node)
    : node_(node), private_node_(private_node) {
  load_configuration();
  twist_publisher_ =
      node_.advertise<geometry_msgs::TwistWithCovarianceStamped>(
          twist_topic_, 20, false);
  encoder_subscriber_ = node_.subscribe(
      encoder_topic_, 50, &EncoderToTwistAdapter::encoder_callback, this);
  ROS_INFO_STREAM("EncoderToTwistAdapter 설정: " << encoder_topic_ << " -> "
                  << twist_topic_ << ", max_speed=" << max_abs_speed_mps_
                  << " m/s, calibration=" << calibration_state_);
}

// 함수이름: load_configuration
// 기능: 엔코더 기하, 시간 제한, 토픽과 frame 파라미터를 읽고 검증한다.
// 인자: 없음
// 반환값: 없음
void EncoderToTwistAdapter::load_configuration() {
  encoder_topic_ =
      requireParameter<std::string>(private_node_, "topics/encoder_state");
  twist_topic_ =
      requireParameter<std::string>(private_node_, "topics/encoder_twist");
  base_link_frame_ =
      requireParameter<std::string>(private_node_, "frames/base_link");
  speed_scale_ =
      requireParameter<double>(private_node_, "encoder/speed_scale");
  direction_sign_ =
      requireParameter<int>(private_node_, "encoder/direction_sign");
  max_abs_speed_mps_ =
      requireParameter<double>(private_node_, "encoder/max_abs_speed_mps");
  max_abs_encoder_delta_100ms_ = requireParameter<int>(
      private_node_, "encoder/max_abs_encoder_delta_100ms");
  speed_variance_m2ps2_ = requireParameter<double>(
      private_node_, "encoder/speed_variance_m2ps2");
  lateral_velocity_variance_m2ps2_ = requireParameter<double>(
      private_node_, "encoder/lateral_velocity_constraint/variance_m2ps2");
  lateral_velocity_calibration_state_ = requireParameter<std::string>(
      private_node_, "encoder/lateral_velocity_constraint/calibration_state");
  lateral_velocity_source_ = requireParameter<std::string>(
      private_node_, "encoder/lateral_velocity_constraint/source");
  unobserved_variance_ = requireParameter<double>(
      private_node_, "encoder/unobserved_variance");
  require_alive_counter_change_ = requireParameter<bool>(
      private_node_, "encoder/require_alive_counter_change");
  calibration_state_ = requireParameter<std::string>(
      private_node_, "encoder/calibration_state");
  encoder_delta_window_sec_ = requireParameter<double>(
      private_node_, "encoder/encoder_delta_window_sec");
  meter_per_tick_enabled_ = requireParameter<bool>(
      private_node_, "encoder/meter_per_tick_enabled");
  meter_per_tick_m_ = requireParameter<double>(
      private_node_, "encoder/meter_per_tick_m");
  delta_consistency_tolerance_mps_ = requireParameter<double>(
      private_node_, "encoder/delta_consistency_tolerance_mps");
  steering_enabled_ = requireParameter<bool>(
      private_node_, "encoder/steering/enabled");
  steering_adc_center_ = requireParameter<int>(
      private_node_, "encoder/steering/adc_center");
  steering_adc_per_rad_ = requireParameter<double>(
      private_node_, "encoder/steering/adc_per_rad");
  steering_adc_min_ = requireParameter<int>(
      private_node_, "encoder/steering/min_adc");
  steering_adc_max_ = requireParameter<int>(
      private_node_, "encoder/steering/max_adc");

  requireAbsoluteRosName(encoder_topic_, "topics/encoder_state");
  requireAbsoluteRosName(twist_topic_, "topics/encoder_twist");
  if (encoder_topic_ == twist_topic_) {
    throw std::runtime_error("encoder input and twist output topics must differ");
  }
  requireNonEmpty(base_link_frame_, "frames/base_link");
  requireFinitePositive(speed_scale_, "encoder/speed_scale");
  if (direction_sign_ != -1 && direction_sign_ != 1) {
    throw std::runtime_error("encoder/direction_sign must be -1 or 1");
  }
  requireFinitePositive(max_abs_speed_mps_, "encoder/max_abs_speed_mps");
  if (max_abs_encoder_delta_100ms_ <= 0) {
    throw std::runtime_error(
        "encoder/max_abs_encoder_delta_100ms must be positive");
  }
  requireProbabilityVariance(speed_variance_m2ps2_,
                             "encoder/speed_variance_m2ps2");
  requireProbabilityVariance(
      lateral_velocity_variance_m2ps2_,
      "encoder/lateral_velocity_constraint/variance_m2ps2");
  if (lateral_velocity_calibration_state_ != "unmeasured" &&
      lateral_velocity_calibration_state_ != "model_assumption" &&
      lateral_velocity_calibration_state_ != "measured" &&
      lateral_velocity_calibration_state_ != "verified") {
    throw std::runtime_error(
        "encoder/lateral_velocity_constraint/calibration_state is invalid");
  }
  if (lateral_velocity_calibration_state_ == "unmeasured" ||
      lateral_velocity_source_.empty()) {
    throw std::runtime_error(
        "lateral velocity constraint requires state and source");
  }
  requireFinitePositive(unobserved_variance_,
                        "encoder/unobserved_variance");
  if (calibration_state_ != "unmeasured" &&
      calibration_state_ != "measured" &&
      calibration_state_ != "verified") {
    throw std::runtime_error("encoder/calibration_state is invalid");
  }
  requireFinitePositive(encoder_delta_window_sec_,
                        "encoder/encoder_delta_window_sec");
  requireFiniteNonnegative(meter_per_tick_m_,
                           "encoder/meter_per_tick_m");
  requireFinitePositive(delta_consistency_tolerance_mps_,
                        "encoder/delta_consistency_tolerance_mps");
  if (meter_per_tick_enabled_ &&
      (calibration_state_ == "unmeasured" || meter_per_tick_m_ <= 0.0)) {
    throw std::runtime_error(
        "meter-per-tick 사용에는 measured 이상 보정값이 필요합니다");
  }
  if (steering_adc_min_ < 0 || steering_adc_max_ > 65535 ||
      steering_adc_min_ >= steering_adc_max_ ||
      steering_adc_center_ < steering_adc_min_ ||
      steering_adc_center_ > steering_adc_max_ ||
      !std::isfinite(steering_adc_per_rad_) || steering_adc_per_rad_ < 0.0) {
    throw std::runtime_error("encoder/steering ADC 설정이 유효하지 않습니다");
  }
  if (steering_enabled_ &&
      (calibration_state_ == "unmeasured" || steering_adc_per_rad_ <= 0.0)) {
    throw std::runtime_error(
        "조향 보정 사용에는 measured 이상 adc_per_rad가 필요합니다");
  }
}

// 함수이름: validate_measurement
// 기능: 엔코더 alive counter, 속도, 조향 ADC와 delta 범위를 검사한다.
// 인자: message, reason
// 반환값: Twist 변환에 사용할 수 있으면 true
bool EncoderToTwistAdapter::validate_measurement(
    const erp42_msgs::SerialFeedBack& message, std::string* reason) const {
  if (!hasUsableSpeed(message)) {
    *reason = "encoder_speed_nonfinite";
    return false;
  }
  if (require_alive_counter_change_ &&
      !aliveCounterAdvanced(message.alive, have_last_alive_counter_,
                            last_alive_counter_)) {
    *reason = "encoder_alive_counter_stalled";
    return false;
  }
  const double calibrated_speed_mps =
      message.speed * speed_scale_ * direction_sign_;
  if (!std::isfinite(calibrated_speed_mps) ||
      std::abs(calibrated_speed_mps) > max_abs_speed_mps_) {
    *reason = "encoder_velocity_out_of_range";
    return false;
  }
  const std::int64_t encoder_delta = message.encoder;
  const std::int64_t encoder_delta_limit = max_abs_encoder_delta_100ms_;
  if (encoder_delta > encoder_delta_limit ||
      encoder_delta < -encoder_delta_limit) {
    *reason = "encoder_delta_out_of_range";
    return false;
  }
  if (!std::isfinite(message.steer) || message.steer < steering_adc_min_ ||
      message.steer > steering_adc_max_) {
    *reason = "steering_adc_out_of_range";
    return false;
  }
  if (meter_per_tick_enabled_) {
    const double delta_speed_mps =
        static_cast<double>(message.encoder) *
        meter_per_tick_m_ / encoder_delta_window_sec_;
    if (!std::isfinite(delta_speed_mps) ||
        std::abs(delta_speed_mps - calibrated_speed_mps) >
            delta_consistency_tolerance_mps_) {
      *reason = "encoder_speed_delta_inconsistent";
      return false;
    }
  }
  reason->clear();
  return true;
}

// 함수이름: make_twist
// 기능: 승인된 speed_mps와 비홀로노믹 횡속도 0 제약을 전달한다.
// 인자: message
// 반환값: base_link frame TwistWithCovarianceStamped
geometry_msgs::TwistWithCovarianceStamped
EncoderToTwistAdapter::make_twist(
    const erp42_msgs::SerialFeedBack& message,
    const ros::Time& receipt_stamp) const {
  // steering_adc만으로 정확한 yaw rate를 만들지 않는다. Z 각속도는 IMU가 제공한다.
  return convertSpeedToTwist(message, receipt_stamp, base_link_frame_, speed_scale_,
                             direction_sign_, speed_variance_m2ps2_,
                             lateral_velocity_variance_m2ps2_,
                             unobserved_variance_);
}

// 함수이름: encoder_callback
// 기능: 승인된 엔코더만 변환하여 발행하고 마지막 timestamp를 갱신한다.
// 인자: message
// 반환값: 없음
void EncoderToTwistAdapter::encoder_callback(
    const erp42_msgs::SerialFeedBack::ConstPtr& message) {
  std::string reason;
  if (!validate_measurement(*message, &reason)) {
    ROS_WARN_THROTTLE(1.0, "Rejecting encoder measurement: %s",
                      reason.c_str());
    return;
  }
  const ros::Time receipt_stamp = ros::Time::now();
  twist_publisher_.publish(make_twist(*message, receipt_stamp));
  last_alive_counter_ = message->alive;
  have_last_alive_counter_ = true;
}

}  // namespace mando_localization
