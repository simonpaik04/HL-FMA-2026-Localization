#include <ros/ros.h>

#include <exception>

#include "relocalization/relocalization_coordinator.hpp"
#include "status_manager/localization_status_manager.hpp"
#include "supervisor/localization_supervisor.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "localization_supervisor");
  try {
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");
    mando_localization::LocalizationStatusManager status_manager(
        node, private_node);
    mando_localization::RelocalizationCoordinator recovery_coordinator(
        node, private_node);
    mando_localization::LocalizationSupervisor supervisor(node, private_node);
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("LocalizationSupervisor 시작 실패: " << error.what());
    return 1;
  }
  return 0;
}
