/*
imu_normalizer.cpp
- 역할: robot_localization 앞단에서 IMU 형식 오류와 stale sample을 차단한다.
- 좌표계: 축 변환은 static TF의 책임이며 이 클래스는 imu_link 데이터를 재표기하지 않는다.
*/
#include "imu_encoder_fusion/imu_normalizer.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "common/message_validation.hpp"
#include "common/parameter_utils.hpp"

namespace mando_localization {

ImuNormalizer::ImuNormalizer(const ros::NodeHandle& node,
                             const ros::NodeHandle& private_node)
    : node_(node), private_node_(private_node) {
  load_configuration();
  imu_publisher_ =
      node_.advertise<sensor_msgs::Imu>(output_topic_, 30, false);
  imu_subscriber_ = node_.subscribe(
      input_topic_, 50, &ImuNormalizer::imu_callback, this);
  ROS_INFO_STREAM("ImuNormalizer 설정: " << input_topic_ << " -> "
                  << output_topic_ << ", frame=" << imu_frame_
                  << ", covariance_override="
                  << (covariance_override_enabled_ ? "true" : "false")
                  << ", positive_diagonal_required="
                  << (require_positive_covariance_diagonal_ ? "true" : "false"));
}

// 함수이름: load_configuration
// 기능: IMU 입출력 토픽, frame과 검증 허용값을 읽는다.
// 인자: 없음
// 반환값: 없음
void ImuNormalizer::load_configuration() {
  input_topic_ =
      requireParameter<std::string>(private_node_, "topics/imu_data");
  output_topic_ =
      requireParameter<std::string>(private_node_, "topics/imu_normalized");
  imu_frame_ = requireParameter<std::string>(private_node_, "frames/imu");
  max_message_age_sec_ =
      requireParameter<double>(private_node_, "imu/max_message_age_sec");
  max_future_stamp_sec_ =
      requireParameter<double>(private_node_, "imu/max_future_stamp_sec");
  min_quaternion_norm_ =
      requireParameter<double>(private_node_, "imu/min_quaternion_norm");
  max_quaternion_normalization_error_ = requireParameter<double>(
      private_node_, "imu/max_quaternion_normalization_error");
  max_covariance_diagonal_ = requireParameter<double>(
      private_node_, "imu/max_covariance_diagonal");
  require_positive_covariance_diagonal_ = requireParameter<bool>(
      private_node_, "imu/require_positive_covariance_diagonal");
  covariance_override_enabled_ = requireParameter<bool>(
      private_node_, "covariance_override/enabled");
  const std::string covariance_calibration_state =
      requireParameter<std::string>(
          private_node_, "covariance_override/calibration_state");
  const std::string covariance_measured_at = requireParameter<std::string>(
      private_node_, "covariance_override/measured_at");
  const std::string covariance_source = requireParameter<std::string>(
      private_node_, "covariance_override/source");
  const std::vector<double> acceleration_stddev =
      requireParameter<std::vector<double>>(
          private_node_,
          "covariance_override/linear_acceleration_stddev_mps2");
  const std::vector<double> angular_velocity_stddev =
      requireParameter<std::vector<double>>(
          private_node_,
          "covariance_override/angular_velocity_stddev_radps");
  const std::vector<double> orientation_stddev =
      requireParameter<std::vector<double>>(
          private_node_, "covariance_override/orientation_stddev_rad");

  requireAbsoluteRosName(input_topic_, "topics/imu_data");
  requireAbsoluteRosName(output_topic_, "topics/imu_normalized");
  if (input_topic_ == output_topic_) {
    throw std::runtime_error("IMU input and normalized output topics must differ");
  }
  requireNonEmpty(imu_frame_, "frames/imu");
  requireFinitePositive(max_message_age_sec_, "imu/max_message_age_sec");
  requireFiniteNonnegative(max_future_stamp_sec_,
                           "imu/max_future_stamp_sec");
  requireFinitePositive(min_quaternion_norm_, "imu/min_quaternion_norm");
  requireFinitePositive(max_quaternion_normalization_error_,
                        "imu/max_quaternion_normalization_error");
  requireFinitePositive(max_covariance_diagonal_,
                        "imu/max_covariance_diagonal");
  if (acceleration_stddev.size() != 3 || angular_velocity_stddev.size() != 3 ||
      orientation_stddev.size() != 3) {
    throw std::runtime_error("IMU covariance override 배열은 길이 3이어야 합니다");
  }
  if (covariance_calibration_state != "unmeasured" &&
      covariance_calibration_state != "measured" &&
      covariance_calibration_state != "verified") {
    throw std::runtime_error("IMU covariance calibration_state가 유효하지 않습니다");
  }
  if (covariance_override_enabled_ &&
      (covariance_calibration_state == "unmeasured" ||
       covariance_measured_at.empty() || covariance_source.empty())) {
    throw std::runtime_error(
        "IMU covariance override에는 실측 상태, 시각과 출처가 필요합니다");
  }
  for (std::size_t index = 0; index < 3; ++index) {
    acceleration_stddev_[index] = acceleration_stddev[index];
    angular_velocity_stddev_[index] = angular_velocity_stddev[index];
    orientation_stddev_[index] = orientation_stddev[index];
    const double values[] = {acceleration_stddev_[index],
                             angular_velocity_stddev_[index],
                             orientation_stddev_[index]};
    for (const double value : values) {
      if (!std::isfinite(value) || value < 0.0 ||
          (covariance_override_enabled_ && value <= 0.0)) {
        throw std::runtime_error(
            "활성 IMU covariance override 표준편차는 유한한 양수여야 합니다");
      }
    }
  }
}

// 함수이름: validate_and_normalize
// 기능: IMU 핵심 측정값을 검사하고 orientation만 단위 quaternion으로 정규화한다.
// 인자: input, output, reason
// 반환값: relay 가능한 메시지를 만들었으면 true
bool ImuNormalizer::validate_and_normalize(const sensor_msgs::Imu& input,
                                           sensor_msgs::Imu* output,
                                           std::string* reason) const {
  if (output == nullptr || reason == nullptr) {
    return false;
  }
  if (input.header.frame_id != imu_frame_) {
    *reason = "imu_frame_mismatch";
    return false;
  }
  if (!MessageValidation::validateStamp(
          input.header.stamp, ros::Time::now(), max_message_age_sec_,
          max_future_stamp_sec_, have_last_stamp_, last_stamp_, reason)) {
    return false;
  }
  geometry_msgs::Quaternion normalized;
  if (!MessageValidation::normalizeQuaternion(
          input.orientation, min_quaternion_norm_,
          max_quaternion_normalization_error_, &normalized, reason)) {
    return false;
  }
  if (!std::isfinite(input.angular_velocity.x) ||
      !std::isfinite(input.angular_velocity.y) ||
      !std::isfinite(input.angular_velocity.z) ||
      !std::isfinite(input.linear_acceleration.x) ||
      !std::isfinite(input.linear_acceleration.y) ||
      !std::isfinite(input.linear_acceleration.z)) {
    *reason = "imu_vector_nonfinite";
    return false;
  }
  if (!covariance_override_enabled_) {
    if (!MessageValidation::validateCovariance(
            input.orientation_covariance.data(), 3, true,
            max_covariance_diagonal_, reason)) {
      *reason = "imu_orientation_" + *reason;
      return false;
    }
    if (!MessageValidation::validateCovariance(
            input.angular_velocity_covariance.data(), 3, true,
            max_covariance_diagonal_, reason)) {
      *reason = "imu_angular_velocity_" + *reason;
      return false;
    }
    if (!MessageValidation::validateCovariance(
            input.linear_acceleration_covariance.data(), 3, true,
            max_covariance_diagonal_, reason)) {
      *reason = "imu_linear_acceleration_" + *reason;
      return false;
    }
    if (require_positive_covariance_diagonal_) {
      const auto has_positive_diagonal = [](const boost::array<double, 9>& covariance) {
        return covariance[0] > 0.0 && covariance[4] > 0.0 &&
               covariance[8] > 0.0;
      };
      if (!has_positive_diagonal(input.orientation_covariance) ||
          !has_positive_diagonal(input.angular_velocity_covariance) ||
          !has_positive_diagonal(input.linear_acceleration_covariance)) {
        *reason = "imu_covariance_not_measured";
        return false;
      }
    }
  }
  *output = input;
  output->orientation = normalized;
  if (covariance_override_enabled_) {
    std::fill(output->orientation_covariance.begin(),
              output->orientation_covariance.end(), 0.0);
    std::fill(output->angular_velocity_covariance.begin(),
              output->angular_velocity_covariance.end(), 0.0);
    std::fill(output->linear_acceleration_covariance.begin(),
              output->linear_acceleration_covariance.end(), 0.0);
    for (std::size_t index = 0; index < 3; ++index) {
      const std::size_t diagonal = index * 3 + index;
      output->orientation_covariance[diagonal] =
          orientation_stddev_[index] * orientation_stddev_[index];
      output->angular_velocity_covariance[diagonal] =
          angular_velocity_stddev_[index] *
          angular_velocity_stddev_[index];
      output->linear_acceleration_covariance[diagonal] =
          acceleration_stddev_[index] * acceleration_stddev_[index];
    }
  }
  reason->clear();
  return true;
}

// 함수이름: imu_callback
// 기능: 검증된 IMU만 normalized 토픽으로 발행한다.
// 인자: message
// 반환값: 없음
void ImuNormalizer::imu_callback(const sensor_msgs::Imu::ConstPtr& message) {
  sensor_msgs::Imu normalized;
  std::string reason;
  if (!validate_and_normalize(*message, &normalized, &reason)) {
    ROS_WARN_THROTTLE(1.0, "Rejecting IMU measurement: %s", reason.c_str());
    return;
  }
  imu_publisher_.publish(normalized);
  last_stamp_ = message->header.stamp;
  have_last_stamp_ = true;
}

}  // namespace mando_localization
