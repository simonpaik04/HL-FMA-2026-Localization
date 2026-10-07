/*
odometry_gps_fusion_node.cpp
- 역할: OdometryGpsFusion 클래스의 ROS 실행 진입점만 제공한다.
*/
#include <exception>

#include <ros/ros.h>

#include "odometry_gps_fusion/odometry_gps_fusion.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry_gps_fusion");
  try {
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");
    mando_localization::OdometryGpsFusion fusion(node, private_node);
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("GPS fusion startup failed: %s", exception.what());
    return 1;
  }
  return 0;
}
