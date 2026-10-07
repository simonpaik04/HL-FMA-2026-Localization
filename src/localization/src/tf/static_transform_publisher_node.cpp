#include <ros/ros.h>

#include <exception>

#include "static_transform_publisher.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "mando_static_transform_publisher");
  try {
    mando_localization::StaticTransformPublisher publisher(ros::NodeHandle("~"));
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("정적 TF 발행 노드 시작 실패: " << error.what());
    return 1;
  }
  return 0;
}
