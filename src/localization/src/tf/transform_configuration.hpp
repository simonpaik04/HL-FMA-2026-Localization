#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>

#include <string>
#include <vector>

namespace mando_localization {

struct StaticTransformSpec {
  std::string parent_frame;
  std::string child_frame;
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
  double roll_deg{0.0};
  double pitch_deg{0.0};
  double yaw_deg{0.0};
  bool enabled{false};
  std::string calibration_state;
  std::string measured_at;
  std::string source;
};

struct DynamicTransformSpec {
  std::string parent_frame;
  std::string child_frame;
  std::string owner_node;
  std::string source;
};

/**
 * @brief tf_configuration.yaml의 정적 TF를 읽고 구조적 오류를 검사한다.
 *
 * 측정되지 않은 센서 TF는 정상적인 설정 항목이지만 발행 대상은 아니다.
 * 중복 child와 순환 구조는 TF 트리를 모호하게 만들기 때문에 시작을 거부한다.
 */
class TransformConfiguration {
 public:
  static std::vector<StaticTransformSpec> load(const ros::NodeHandle& node,
                                               const std::string& parameter_name);
  static std::vector<DynamicTransformSpec> loadDynamic(
      const ros::NodeHandle& node, const std::string& parameter_name);
  static void validate(const std::vector<StaticTransformSpec>& transforms);
  static void validateTree(
      const std::vector<StaticTransformSpec>& static_transforms,
      const std::vector<DynamicTransformSpec>& dynamic_transforms);
  static bool shouldPublish(const StaticTransformSpec& transform);
  static geometry_msgs::TransformStamped toMessage(const StaticTransformSpec& transform,
                                                    const ros::Time& stamp);
};

}  // namespace mando_localization
