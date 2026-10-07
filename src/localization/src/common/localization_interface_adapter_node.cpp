#include <exception>

#include <ros/ros.h>

#include "common/localization_interface_adapter.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "localization_interface_adapter");
  try {
    mando_localization::LocalizationInterfaceAdapter adapter(
        ros::NodeHandle(), ros::NodeHandle("~"));
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("LocalizationInterfaceAdapter 시작 실패: %s", error.what());
    return 1;
  }
  return 0;
}
