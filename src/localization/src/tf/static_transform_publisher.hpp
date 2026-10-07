#pragma once

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <vector>

#include "transform_configuration.hpp"

namespace mando_localization {

/** @brief 실측 완료로 명시된 정적 TF만 한 번 발행한다. */
class StaticTransformPublisher {
 public:
  explicit StaticTransformPublisher(ros::NodeHandle private_nh);

 private:
  tf2_ros::StaticTransformBroadcaster broadcaster_;
  std::vector<geometry_msgs::TransformStamped> published_transforms_;
};

}  // namespace mando_localization
