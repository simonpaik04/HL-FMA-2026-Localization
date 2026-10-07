/*
imu_normalizer_node.cpp
- 역할: ImuNormalizer 클래스의 ROS 실행 진입점만 제공한다.
*/
#include <exception>

#include <ros/ros.h>

#include "imu_encoder_fusion/imu_normalizer.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_normalizer");
  try {
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");
    mando_localization::ImuNormalizer normalizer(node, private_node);
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("IMU normalizer startup failed: %s", exception.what());
    return 1;
  }
  return 0;
}
