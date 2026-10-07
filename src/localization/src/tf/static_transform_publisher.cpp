#include "static_transform_publisher.hpp"

#include <stdexcept>
#include <string>
#include <utility>

namespace mando_localization {

StaticTransformPublisher::StaticTransformPublisher(ros::NodeHandle private_nh) {
  int schema_version = 0;
  bool strict_validation = false;
  if (!private_nh.getParam("schema_version", schema_version) ||
      schema_version != 1 ||
      !private_nh.getParam("strict_validation", strict_validation) ||
      !strict_validation) {
    throw std::runtime_error("TF schema_version=1 및 strict_validation=true가 필요합니다.");
  }
  std::string parameter_name;
  if (!private_nh.getParam("static_transforms_parameter", parameter_name) ||
      parameter_name.empty()) {
    throw std::runtime_error("static_transforms_parameter가 필요합니다.");
  }
  const std::vector<StaticTransformSpec> configured =
      TransformConfiguration::load(private_nh, parameter_name);
  const std::vector<DynamicTransformSpec> dynamic =
      TransformConfiguration::loadDynamic(private_nh, "dynamic_transforms");
  TransformConfiguration::validateTree(configured, dynamic);

  const auto required_string = [&private_nh](const std::string& name) {
    std::string value;
    if (!private_nh.getParam(name, value) || value.empty()) {
      throw std::runtime_error("필수 TF 문자열 파라미터가 없습니다: " + name);
    }
    return value;
  };
  const std::string map_frame = required_string("frames/map");
  const std::string odom_frame = required_string("frames/odom");
  const std::string base_frame = required_string("frames/base_link");
  const std::string base_origin = required_string("base_link_reference/origin");
  const std::string x_axis = required_string("base_link_reference/x_axis");
  const std::string y_axis = required_string("base_link_reference/y_axis");
  const std::string z_axis = required_string("base_link_reference/z_axis");
  if (x_axis != "forward" || y_axis != "left" || z_axis != "up") {
    throw std::runtime_error(
        "base_link 축은 REP-103의 forward/left/up이어야 합니다.");
  }
  const std::string expected_global_owner =
      required_string("expected_global_owner");
  const std::string expected_local_owner =
      required_string("expected_local_owner");
  bool found_global = false;
  bool found_local = false;
  for (const DynamicTransformSpec& transform : dynamic) {
    if (transform.parent_frame == map_frame &&
        transform.child_frame == odom_frame &&
        transform.owner_node == expected_global_owner) {
      found_global = true;
    }
    if (transform.parent_frame == odom_frame &&
        transform.child_frame == base_frame &&
        transform.owner_node == expected_local_owner) {
      found_local = true;
    }
  }
  if (!found_global || !found_local) {
    throw std::runtime_error(
        "동적 TF owner_node가 launch의 Local/Global EKF 이름과 다릅니다.");
  }
  ROS_INFO_STREAM("base_link 기준: origin=" << base_origin
                  << ", axes=" << x_axis << "/" << y_axis << "/" << z_axis);
  const ros::Time stamp = ros::Time::now();
  for (const StaticTransformSpec& transform : configured) {
    if (TransformConfiguration::shouldPublish(transform)) {
      published_transforms_.push_back(TransformConfiguration::toMessage(transform, stamp));
      ROS_INFO_STREAM("정적 TF 발행: " << transform.parent_frame << " -> "
                      << transform.child_frame << " (" << transform.calibration_state << ")");
    } else {
      ROS_WARN_STREAM("미측정 또는 비활성 TF는 발행하지 않습니다: "
                      << transform.parent_frame << " -> " << transform.child_frame);
    }
  }
  if (!published_transforms_.empty()) {
    broadcaster_.sendTransform(published_transforms_);
  } else {
    ROS_WARN("발행 가능한 정적 TF가 없습니다. 실측 후 tf_configuration.yaml을 갱신하세요.");
  }
}

}  // namespace mando_localization
