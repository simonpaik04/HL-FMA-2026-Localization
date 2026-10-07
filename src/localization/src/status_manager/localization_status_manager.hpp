#pragma once

#include <diagnostic_msgs/DiagnosticArray.h>
#include <erp42_msgs/SerialFeedBack.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <string>
#include <cstdint>
#include <vector>

#include "localization_state_evaluator.hpp"

namespace mando_localization {

/**
 * @brief 각 센서와 필터 출력의 건강 상태만 판정하는 관리자.
 *
 * EKF 계산, 위치 보정, TF 발행, Odometry relay는 수행하지 않는다. 메시지
 * 도착 시각과 header stamp를 분리해서 검사하며, fail-closed 판단 결과를
 * 별도 Output Gate가 소비하도록 발행한다.
 */
class LocalizationStatusManager {
 public:
  LocalizationStatusManager(ros::NodeHandle nh, ros::NodeHandle private_nh);

 private:
  struct StreamStatus {
    bool seen{false};
    bool payload_valid{false};
    ros::Time receipt_time;
    ros::Time stamp;
    std::string frame_id;
    std::string reason{"not_received"};
  };

  void imuCallback(const sensor_msgs::ImuConstPtr& message);
  void encoderCallback(const erp42_msgs::SerialFeedBackConstPtr& message);
  void twistCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr& message);
  void localOdometryCallback(const nav_msgs::OdometryConstPtr& message);
  void gpsPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& message);
  void globalOdometryCallback(const nav_msgs::OdometryConstPtr& message);
  void gpsRelocalizingCallback(const std_msgs::BoolConstPtr& message);
  void initializationReadyCallback(const std_msgs::BoolConstPtr& message);
  void initializationPoseCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& message);
  void timerCallback(const ros::TimerEvent& event);

  bool initializationReady(const ros::Time& now) const;
  void applyInitializationAnchor(const ros::Time& now);
  void clearInitializationAnchor();

  bool isFresh(const StreamStatus& stream, double timeout_sec,
               double max_future_sec, const ros::Time& now) const;
  bool validatePose(const geometry_msgs::Pose& pose) const;
  bool validatePoseCovariance(const boost::array<double, 36>& covariance,
                              bool enforce_position_limit = true) const;
  void markAbsoluteMeasurement(const ros::Time& receipt_time);
  diagnostic_msgs::DiagnosticStatus makeStreamDiagnostic(
      const std::string& name, const StreamStatus& stream, double timeout_sec,
      double max_future_sec, const ros::Time& now, bool enabled) const;
  diagnostic_msgs::DiagnosticStatus makeDeviceDiagnostic(
      const std::string& name, const std::string& path, bool required) const;
  diagnostic_msgs::DiagnosticStatus makeDriverDiagnostic(
      const std::string& name, const std::string& expected_node, bool required,
      const std::vector<std::string>& running_nodes) const;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber encoder_subscriber_;
  ros::Subscriber twist_subscriber_;
  ros::Subscriber local_odometry_subscriber_;
  ros::Subscriber gps_pose_subscriber_;
  ros::Subscriber global_odometry_subscriber_;
  ros::Subscriber gps_relocalizing_subscriber_;
  ros::Subscriber initialization_ready_subscriber_;
  ros::Subscriber initialization_pose_subscriber_;
  ros::Publisher diagnostics_publisher_;
  ros::Publisher state_publisher_;
  ros::Publisher valid_publisher_;
  ros::Timer timer_;

  StreamStatus imu_status_;
  StreamStatus encoder_status_;
  StreamStatus twist_status_;
  StreamStatus local_odometry_status_;
  StreamStatus gps_status_;
  StreamStatus global_odometry_status_;
  StreamStatus initialization_pose_status_;
  bool initialization_required_{false};
  bool initialization_ready_{false};
  bool initialization_anchor_applied_{false};
  double initialization_ready_timeout_sec_{0.5};
  double initialization_pose_max_age_sec_{1.0};
  ros::Time initialization_ready_receipt_time_;
  ros::WallTime initialization_ready_wall_time_;
  ros::WallTime initialization_pose_wall_time_;
  uint16_t steering_adc_{0};
  int32_t encoder_delta_100ms_{0};
  bool brake_{false};
  bool have_encoder_alive_counter_{false};
  uint8_t encoder_alive_counter_{0U};

  LocalizationStateEvaluator evaluator_;
  ros::Time start_time_;
  ros::Time last_absolute_time_;
  bool anchor_seen_{false};
  bool gps_relocalizing_{false};
  bool dead_reckoning_active_{false};
  double dead_reckoning_distance_m_{0.0};
  bool have_previous_global_position_{false};
  geometry_msgs::Point previous_global_position_;

  bool enable_gps_{true};
  double imu_timeout_sec_{0.2};
  double imu_max_future_sec_{0.05};
  double encoder_timeout_sec_{0.3};
  double encoder_max_future_sec_{0.05};
  double local_odometry_timeout_sec_{0.25};
  double local_odometry_max_future_sec_{0.05};
  double gps_timeout_sec_{1.5};
  double gps_max_future_sec_{0.5};
  double global_odometry_timeout_sec_{0.25};
  double global_odometry_max_future_sec_{0.5};
  double status_publish_rate_hz_{10.0};
  double max_position_variance_m2_{100.0};
  double max_global_consistency_distance_m_{10.0};
  geometry_msgs::Point gps_position_;
  geometry_msgs::Point global_position_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_device_path_;
  std::string gps_device_path_;
  std::string imu_driver_node_;
  std::string gps_driver_node_;
};

}  // namespace mando_localization
