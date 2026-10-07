#include <ros/ros.h>

#include <exception>

#include "localization_output_gate.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "localization_output_gate");
  try {
    mando_localization::LocalizationOutputGate gate(ros::NodeHandle(), ros::NodeHandle("~"));
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("LocalizationOutputGate 시작 실패: " << error.what());
    return 1;
  }
  return 0;
}
