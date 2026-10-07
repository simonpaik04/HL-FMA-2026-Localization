#pragma once

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <topic_tools/shape_shifter.h>

namespace mando_localization {

/**
 * @brief 차량별 공개 토픽과 패키지 내부 고정 필터 토픽을 연결한다.
 *
 * 메시지를 계산하거나 판정하지 않고 그대로 전달한다. 따라서 공개 토픽 변경은
 * localization_interfaces.yaml 한 곳에서 끝나며 robot_localization 설정은 차량마다
 * 복제하지 않는다.
 */
class LocalizationInterfaceAdapter {
 public:
  LocalizationInterfaceAdapter(const ros::NodeHandle& node,
                               const ros::NodeHandle& private_node);

 private:
  void imuDriverCallback(const sensor_msgs::ImuConstPtr& message);
  void gpsDriverCallback(const sensor_msgs::NavSatFixConstPtr& message);
  void gpsNavPvtDriverCallback(
      const topic_tools::ShapeShifter::ConstPtr& message);
  void imuCallback(const sensor_msgs::ImuConstPtr& message);
  void twistCallback(
      const geometry_msgs::TwistWithCovarianceStampedConstPtr& message);
  void gpsPoseCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& message);
  void localOdometryCallback(const nav_msgs::OdometryConstPtr& message);
  void globalOdometryCallback(const nav_msgs::OdometryConstPtr& message);

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber imu_driver_subscriber_;
  ros::Subscriber gps_driver_subscriber_;
  ros::Subscriber gps_navpvt_driver_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber twist_subscriber_;
  ros::Subscriber gps_pose_subscriber_;
  ros::Subscriber local_odometry_subscriber_;
  ros::Subscriber global_odometry_subscriber_;
  ros::Publisher public_imu_publisher_;
  ros::Publisher public_gps_publisher_;
  ros::Publisher public_gps_navpvt_publisher_;
  ros::Publisher internal_imu_publisher_;
  ros::Publisher internal_twist_publisher_;
  ros::Publisher internal_gps_pose_publisher_;
  ros::Publisher public_local_odometry_publisher_;
  ros::Publisher public_global_odometry_publisher_;
  std::string public_gps_navpvt_topic_;
};

}  // namespace mando_localization
