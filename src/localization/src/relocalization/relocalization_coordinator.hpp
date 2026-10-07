#pragma once

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <mando_localization/GpsGateReanchor.h>
#include <nav_msgs/Odometry.h>
#include <robot_localization/SetPose.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <cstdint>
#include <string>

#include "relocalization/relocalization_policy.hpp"

namespace mando_localization {

/**
 * @brief 승인 전 GPS 후보를 이용해 장기 단절 재정합을 조정한다.
 *
 * 정상/짧은 단절은 기존 GPS gate 출력을 그대로 전달한다. 장기 단절에서는
 * Global EKF
 * reset은 정지·후보 안정성·명시적 설정을 모두 만족할 때만 요청한다.
 */
class RelocalizationCoordinator {
 public:
  RelocalizationCoordinator(ros::NodeHandle nh, ros::NodeHandle private_nh);

 private:

  struct TimedTwist {
    bool seen{false};
    bool valid{false};
    ros::Time receipt_time;
    geometry_msgs::TwistWithCovarianceStamped message;
  };

  struct TimedOdometry {
    bool seen{false};
    bool valid{false};
    ros::Time receipt_time;
    nav_msgs::Odometry message;
  };

  void gpsCandidateCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& message);
  void gpsGatePoseCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& message);
  void twistCallback(
      const geometry_msgs::TwistWithCovarianceStampedConstPtr& message);
  void globalOdometryCallback(const nav_msgs::OdometryConstPtr& message);
  void gateRelocalizingCallback(const std_msgs::BoolConstPtr& message);
  void reanchorAcceptedCallback(
      const mando_localization::GpsGateReanchorConstPtr& message);
  void timerCallback(const ros::TimerEvent& event);

  bool validatePose(const geometry_msgs::PoseWithCovarianceStamped& message,
                    const ros::Time& now) const;
  bool twistIsFresh(const ros::Time& now) const;
  bool globalOdometryIsFresh(const ros::Time& now) const;
  void updateCandidateCluster(
      const geometry_msgs::PoseWithCovarianceStamped& candidate);
  void resetCandidateCluster();
  bool requestGpsOnlyReset(
      const geometry_msgs::PoseWithCovarianceStamped& candidate,
      const ros::Time& now);
  void requestGateReanchor(
      const geometry_msgs::PoseWithCovarianceStamped& candidate,
      const std::string& completion_state);
  void finishRecovery(
      const geometry_msgs::PoseWithCovarianceStamped& candidate,
      const std::string& state);
  void publishRecoveryStatus(const std::string& state, bool active);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber gps_candidate_subscriber_;
  ros::Subscriber gps_gate_pose_subscriber_;
  ros::Subscriber twist_subscriber_;
  ros::Subscriber global_odometry_subscriber_;
  ros::Subscriber gate_relocalizing_subscriber_;
  ros::Subscriber reanchor_accepted_subscriber_;
  ros::Publisher gps_pose_publisher_;
  ros::Publisher gps_relocalizing_publisher_;
  ros::Publisher gps_reanchor_publisher_;
  ros::Publisher recovery_state_publisher_;
  ros::Publisher recovery_active_publisher_;
  ros::ServiceClient set_pose_client_;
  ros::Timer timer_;

  RelocalizationPolicy policy_;
  std::string map_frame_;
  double long_outage_sec_{2.0};
  double candidate_max_message_age_sec_{1.0};
  double candidate_max_future_stamp_sec_{0.5};
  double candidate_cluster_radius_m_{2.0};
  double twist_timeout_sec_{0.2};
  double global_odometry_timeout_sec_{0.2};
  double set_pose_service_wait_timeout_sec_{0.3};
  double global_confirmation_timeout_sec_{1.0};
  double max_global_confirmation_distance_m_{2.0};
  double reanchor_ack_timeout_sec_{1.0};
  int required_global_confirmations_{3};

  bool have_prior_anchor_{false};
  ros::Time last_gate_pose_receipt_time_;
  bool gate_relocalizing_{false};
  bool recovery_active_{false};
  std::string recovery_state_{"IDLE"};
  bool reset_pending_{false};
  ros::Time reset_request_time_;
  geometry_msgs::PoseWithCovarianceStamped reset_target_;
  int consecutive_global_confirmations_{0};
  ros::Time last_global_confirmation_stamp_;
  bool reanchor_pending_{false};
  ros::Time reanchor_request_time_;
  geometry_msgs::PoseWithCovarianceStamped reanchor_target_;
  std::string pending_completion_state_;
  std::uint64_t pending_reanchor_transaction_id_{0};
  std::uint64_t next_reanchor_transaction_id_{1};
  int consecutive_candidates_{0};
  bool have_candidate_cluster_anchor_{false};
  ros::Time last_candidate_stamp_;
  geometry_msgs::Point candidate_cluster_anchor_position_;
  TimedTwist twist_;
  TimedOdometry global_odometry_;
};

}  // namespace mando_localization
