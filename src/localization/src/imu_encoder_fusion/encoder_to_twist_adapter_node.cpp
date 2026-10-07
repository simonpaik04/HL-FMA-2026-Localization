/*
encoder_to_twist_adapter_node.cpp
- 역할: EncoderToTwistAdapter 클래스의 ROS 실행 진입점만 제공한다.
*/
#include <exception>

#include <ros/ros.h>

#include "imu_encoder_fusion/encoder_to_twist_adapter.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "encoder_to_twist_adapter");
  try {
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");
    mando_localization::EncoderToTwistAdapter adapter(node, private_node);
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("Encoder adapter startup failed: %s", exception.what());
    return 1;
  }
  return 0;
}
