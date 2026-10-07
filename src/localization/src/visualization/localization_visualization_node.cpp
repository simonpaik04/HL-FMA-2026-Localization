#include <ros/ros.h>

#include <exception>

#include "localization_visualization.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "localization_visualization");
  try {
    mando_localization::LocalizationVisualization visualization(ros::NodeHandle(),
                                                               ros::NodeHandle("~"));
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("LocalizationVisualization 시작 실패: " << error.what());
    return 1;
  }
  return 0;
}
