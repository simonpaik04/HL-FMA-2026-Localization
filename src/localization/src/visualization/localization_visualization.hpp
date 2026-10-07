#pragma once

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>

#include <cstddef>
#include <string>
#include <vector>

namespace mando_localization {

/** @brief 승인된 최종 위치만 누적해 RViz용 주행 경로를 발행한다. */
class LocalizationVisualization {
 public:
  LocalizationVisualization(ros::NodeHandle nh, ros::NodeHandle private_nh);

 private:
  void odometryCallback(const nav_msgs::OdometryConstPtr& message);
  void validCallback(const std_msgs::BoolConstPtr& message);
  void gpsPoseCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& message);
  void publishTimerCallback(const ros::TimerEvent& event);
  visualization_msgs::Marker makePoseMarker(
      int id, const std::string& name, const geometry_msgs::Pose& pose,
      double scale, const std::vector<double>& color) const;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber valid_subscriber_;
  ros::Subscriber gps_pose_subscriber_;
  ros::Publisher path_publisher_;
  ros::Publisher marker_publisher_;
  ros::Timer publish_timer_;
  nav_msgs::Path path_;
  std::size_t max_path_points_{2000U};
  bool clear_on_invalid_{true};
  bool valid_{false};
  bool have_odometry_{false};
  bool have_gps_pose_{false};
  nav_msgs::Odometry odometry_;
  geometry_msgs::PoseWithCovarianceStamped gps_pose_;
  double min_translation_m_{0.05};
  double min_rotation_rad_{0.02};
  double path_line_width_m_{0.05};
  std::vector<double> path_color_;
  std::vector<double> vehicle_scale_;
  double gps_scale_m_{0.35};
  std::vector<double> gps_color_;
  std::vector<double> fault_color_;
  std::string map_frame_;
};

}  // namespace mando_localization
