#include "relocalization/relocalization_coordinator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "common/parameter_utils.hpp"

namespace mando_localization {
namespace {

bool finiteQuaternion(const geometry_msgs::Quaternion& quaternion) {
  const double norm_squared = quaternion.x * quaternion.x +
                              quaternion.y * quaternion.y +
                              quaternion.z * quaternion.z +
                              quaternion.w * quaternion.w;
  return std::isfinite(norm_squared) && norm_squared > 1.0e-12;
}

double planarDistance(const geometry_msgs::Point& lhs,
                      const geometry_msgs::Point& rhs) {
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

}  // namespace

RelocalizationCoordinator::RelocalizationCoordinator(
    ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(std::move(nh)),
      private_nh_(std::move(private_nh)),
      policy_([&]() {
        RelocalizationPolicyConfig config;
        config.max_stationary_speed_mps = requireParameter<double>(
            private_nh_,
            "relocalization/gps_only/max_stationary_speed_mps");
        config.gps_only_required_consecutive_candidates = requireParameter<int>(
            private_nh_,
            "relocalization/gps_only/required_consecutive_candidates");
        config.automatic_gps_only_reset_enabled = requireParameter<bool>(
            private_nh_,
            "relocalization/gps_only/automatic_reset_enabled");
        config.require_measured_datum = requireParameter<bool>(
            private_nh_,
            "relocalization/gps_only/require_measured_datum");
        config.gps_datum_measured = requireParameter<bool>(
            private_nh_, "reference/measured");
        return config;
      }()) {
  const std::string gps_candidate_topic = requireParameter<std::string>(
      private_nh_, "internal_topics/gps_candidate_pose");
  const std::string gps_gate_pose_topic = requireParameter<std::string>(
      private_nh_, "internal_topics/gps_gate_pose");
  const std::string gate_relocalizing_topic = requireParameter<std::string>(
      private_nh_, "internal_topics/gps_gate_relocalizing");
  const std::string gps_reanchor_topic = requireParameter<std::string>(
      private_nh_, "internal_topics/gps_reanchor_pose");
  const std::string gps_reanchor_accepted_topic =
      requireParameter<std::string>(
          private_nh_, "internal_topics/gps_reanchor_accepted");
  const std::string recovery_active_topic = requireParameter<std::string>(
      private_nh_, "internal_topics/recovery_active");
  const std::string gps_pose_topic = requireParameter<std::string>(
      private_nh_, "topics/gps_map_pose");
  const std::string gps_relocalizing_topic = requireParameter<std::string>(
      private_nh_, "topics/gps_relocalizing");
  const std::string twist_topic = requireParameter<std::string>(
      private_nh_, "topics/encoder_twist");
  const std::string global_odometry_topic = requireParameter<std::string>(
      private_nh_, "topics/global_odometry");
  const std::string recovery_state_topic = requireParameter<std::string>(
      private_nh_, "topics/recovery_state");
  const std::string set_pose_service = requireParameter<std::string>(
      private_nh_, "services/global_ekf_set_pose");

  map_frame_ = requireParameter<std::string>(private_nh_, "frames/map");
  long_outage_sec_ = requireParameter<double>(
      private_nh_, "relocalization/long_outage_sec");
  candidate_max_message_age_sec_ = requireParameter<double>(
      private_nh_, "relocalization/candidate_max_message_age_sec");
  candidate_max_future_stamp_sec_ = requireParameter<double>(
      private_nh_, "relocalization/candidate_max_future_stamp_sec");
  candidate_cluster_radius_m_ = requireParameter<double>(
      private_nh_, "relocalization/candidate_cluster_radius_m");
  twist_timeout_sec_ = requireParameter<double>(
      private_nh_, "sources/encoder/timeout_sec");
  global_odometry_timeout_sec_ = requireParameter<double>(
      private_nh_, "sources/global_odometry/timeout_sec");
  set_pose_service_wait_timeout_sec_ = requireParameter<double>(
      private_nh_,
      "relocalization/gps_only/set_pose_service_wait_timeout_sec");
  global_confirmation_timeout_sec_ = requireParameter<double>(
      private_nh_,
      "relocalization/gps_only/global_confirmation_timeout_sec");
  max_global_confirmation_distance_m_ = requireParameter<double>(
      private_nh_,
      "relocalization/gps_only/max_global_confirmation_distance_m");
  required_global_confirmations_ = requireParameter<int>(
      private_nh_,
      "relocalization/gps_only/required_global_confirmations");
  reanchor_ack_timeout_sec_ = requireParameter<double>(
      private_nh_, "relocalization/reanchor_ack_timeout_sec");

  const double positive_values[] = {
      long_outage_sec_, candidate_max_message_age_sec_,
      candidate_cluster_radius_m_, twist_timeout_sec_,
      global_odometry_timeout_sec_, set_pose_service_wait_timeout_sec_,
      global_confirmation_timeout_sec_, max_global_confirmation_distance_m_,
      reanchor_ack_timeout_sec_};
  if (map_frame_.empty() ||
      std::any_of(std::begin(positive_values), std::end(positive_values),
                  [](const double value) {
                    return !std::isfinite(value) || value <= 0.0;
                  }) ||
      !std::isfinite(candidate_max_future_stamp_sec_) ||
      candidate_max_future_stamp_sec_ < 0.0 ||
      required_global_confirmations_ <= 0) {
    throw std::runtime_error("재정합 Coordinator 설정이 유효하지 않습니다.");
  }

  gps_pose_publisher_ =
      nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
          gps_pose_topic, 10, false);
  gps_relocalizing_publisher_ =
      nh_.advertise<std_msgs::Bool>(gps_relocalizing_topic, 1, true);
  gps_reanchor_publisher_ =
      nh_.advertise<mando_localization::GpsGateReanchor>(
          gps_reanchor_topic, 1, false);
  recovery_state_publisher_ =
      nh_.advertise<std_msgs::String>(recovery_state_topic, 1, true);
  recovery_active_publisher_ =
      nh_.advertise<std_msgs::Bool>(recovery_active_topic, 1, true);

  gps_candidate_subscriber_ = nh_.subscribe(
      gps_candidate_topic, 20,
      &RelocalizationCoordinator::gpsCandidateCallback, this);
  gps_gate_pose_subscriber_ = nh_.subscribe(
      gps_gate_pose_topic, 20,
      &RelocalizationCoordinator::gpsGatePoseCallback, this);
  twist_subscriber_ = nh_.subscribe(
      twist_topic, 50, &RelocalizationCoordinator::twistCallback, this);
  global_odometry_subscriber_ = nh_.subscribe(
      global_odometry_topic, 50,
      &RelocalizationCoordinator::globalOdometryCallback, this);
  gate_relocalizing_subscriber_ = nh_.subscribe(
      gate_relocalizing_topic, 10,
      &RelocalizationCoordinator::gateRelocalizingCallback, this);
  reanchor_accepted_subscriber_ = nh_.subscribe(
      gps_reanchor_accepted_topic, 5,
      &RelocalizationCoordinator::reanchorAcceptedCallback, this);
  set_pose_client_ = nh_.serviceClient<robot_localization::SetPose>(
      set_pose_service, false);
  timer_ = nh_.createTimer(ros::Duration(0.05),
                           &RelocalizationCoordinator::timerCallback, this);
  publishRecoveryStatus("IDLE", false);

  ROS_INFO_STREAM("RelocalizationCoordinator 설정: long outage="
                  << long_outage_sec_ << " s"
                  << ", GPS-only reset="
                  << policy_.config().automatic_gps_only_reset_enabled);
}

void RelocalizationCoordinator::gpsCandidateCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& message) {
  const ros::Time now = ros::Time::now();
  if (!validatePose(*message, now) || reset_pending_ || reanchor_pending_) {
    return;
  }
  if (!have_prior_anchor_ || last_gate_pose_receipt_time_.isZero() ||
      (now - last_gate_pose_receipt_time_).toSec() <= long_outage_sec_) {
    return;
  }

  recovery_active_ = true;
  const bool twist_healthy = twistIsFresh(now);
  const double speed_mps = twist_healthy
      ? twist_.message.twist.twist.linear.x
      : 0.0;
  const bool gps_only_candidate_is_stationary = twist_healthy &&
      std::abs(speed_mps) <= policy_.config().max_stationary_speed_mps;
  const bool new_candidate_epoch = last_candidate_stamp_.isZero() ||
      message->header.stamp > last_candidate_stamp_;

  // 정지 중 수신한 서로 다른 GPS epoch만 후보로 센다.
  if (!new_candidate_epoch) {
    resetCandidateCluster();

  } else if (gps_only_candidate_is_stationary) {
    last_candidate_stamp_ = message->header.stamp;
    updateCandidateCluster(*message);
  } else {
    last_candidate_stamp_ = message->header.stamp;
    resetCandidateCluster();
  }

  RelocalizationObservation observation;
  observation.have_prior_anchor = true;
  observation.long_outage = true;
  observation.twist_healthy = twist_healthy;
  observation.speed_mps = speed_mps;
  observation.global_odometry_healthy = globalOdometryIsFresh(now);
  observation.consecutive_candidates = consecutive_candidates_;

  const RelocalizationDecision decision = policy_.evaluate(observation);
  publishRecoveryStatus(decision.state, decision.relocalizing);
  if (decision.action == RecoveryAction::RESET_WITH_GPS &&
             requestGpsOnlyReset(*message, now)) {
    publishRecoveryStatus("WAITING_FOR_GLOBAL_CONFIRMATION", true);
  }
}

void RelocalizationCoordinator::gpsGatePoseCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& message) {
  const ros::Time now = ros::Time::now();
  if (!validatePose(*message, now)) {
    return;
  }
  // 재앵커 handshake 중에는 gate 적용 ack가 공개 승인보다 먼저 와야 한다.
  if (reanchor_pending_) {
    return;
  }
  gps_pose_publisher_.publish(message);
  have_prior_anchor_ = true;
  last_gate_pose_receipt_time_ = now;
  resetCandidateCluster();
  last_candidate_stamp_ = ros::Time(0);
  reset_pending_ = false;
  consecutive_global_confirmations_ = 0;
  recovery_active_ = false;
  publishRecoveryStatus("TRACKING", false);
}

void RelocalizationCoordinator::twistCallback(
    const geometry_msgs::TwistWithCovarianceStampedConstPtr& message) {
  twist_.seen = true;
  twist_.receipt_time = ros::Time::now();
  twist_.message = *message;
  twist_.valid = !message->header.stamp.isZero() &&
                 std::isfinite(message->twist.twist.linear.x);
}

void RelocalizationCoordinator::globalOdometryCallback(
    const nav_msgs::OdometryConstPtr& message) {
  const ros::Time now = ros::Time::now();
  global_odometry_.seen = true;
  global_odometry_.receipt_time = now;
  global_odometry_.message = *message;
  global_odometry_.valid = !message->header.stamp.isZero() &&
      message->header.frame_id == map_frame_ &&
      std::isfinite(message->pose.pose.position.x) &&
      std::isfinite(message->pose.pose.position.y) &&
      finiteQuaternion(message->pose.pose.orientation);

  if (reset_pending_) {
    const bool post_reset_output =
        message->header.stamp >= reset_request_time_;
    const bool matches_target = globalOdometryIsFresh(now) &&
        planarDistance(message->pose.pose.position,
                       reset_target_.pose.pose.position) <=
            max_global_confirmation_distance_m_;
    const bool new_confirmation_stamp =
        last_global_confirmation_stamp_.isZero() ||
        message->header.stamp > last_global_confirmation_stamp_;
    if (post_reset_output && matches_target) {
      if (new_confirmation_stamp) {
        ++consecutive_global_confirmations_;
        last_global_confirmation_stamp_ = message->header.stamp;
        if (consecutive_global_confirmations_ >=
            required_global_confirmations_) {
          reset_pending_ = false;
          consecutive_global_confirmations_ = 0;
          requestGateReanchor(reset_target_, "RECOVERED_WITH_GPS_RESET");
        }
      }
    } else {
      consecutive_global_confirmations_ = 0;
      last_global_confirmation_stamp_ = ros::Time(0);
    }
  }
}

void RelocalizationCoordinator::gateRelocalizingCallback(
    const std_msgs::BoolConstPtr& message) {
  gate_relocalizing_ = message->data;
  if (!recovery_active_ && !reset_pending_ && !reanchor_pending_) {
    publishRecoveryStatus(gate_relocalizing_ ? "SHORT_RECOVERY" : "TRACKING",
                          gate_relocalizing_);
  }
}

void RelocalizationCoordinator::reanchorAcceptedCallback(
    const mando_localization::GpsGateReanchorConstPtr& message) {
  const ros::Time now = ros::Time::now();
  if (!reanchor_pending_) {
    return;
  }
  if (!validatePose(message->pose, now)) {
    ROS_WARN("Ignoring invalid or stale GPS gate reanchor ack");
    return;
  }
  const double stamp_gap = std::abs(
      (message->pose.header.stamp - reanchor_target_.header.stamp).toSec());
  const double position_gap = planarDistance(
      message->pose.pose.pose.position,
      reanchor_target_.pose.pose.position);
  if (message->transaction_id != pending_reanchor_transaction_id_ ||
      stamp_gap > 1.0e-6 || position_gap > 1.0e-6) {
    ROS_WARN_STREAM("Ignoring mismatched GPS gate reanchor ack: transaction "
                    << message->transaction_id << "/"
                    << pending_reanchor_transaction_id_ << ", stamp gap "
                    << stamp_gap << " s, position gap " << position_gap
                    << " m");
    return;
  }
  ROS_INFO_STREAM("GPS gate reanchor ack accepted: transaction "
                  << message->transaction_id);
  finishRecovery(reanchor_target_, pending_completion_state_);
}

void RelocalizationCoordinator::timerCallback(const ros::TimerEvent&) {
  if (reset_pending_) {
    const double age = (ros::Time::now() - reset_request_time_).toSec();
    if (!std::isfinite(age) || age > global_confirmation_timeout_sec_) {
      reset_pending_ = false;
      recovery_active_ = true;
      consecutive_global_confirmations_ = 0;
      last_global_confirmation_stamp_ = ros::Time(0);
      resetCandidateCluster();
      recovery_state_ = "GLOBAL_CONFIRMATION_TIMEOUT";
    }
  }
  if (reanchor_pending_) {
    const double age =
        (ros::Time::now() - reanchor_request_time_).toSec();
    if (!std::isfinite(age) || age > reanchor_ack_timeout_sec_) {
      reanchor_pending_ = false;
      pending_reanchor_transaction_id_ = 0U;
      recovery_active_ = true;
      resetCandidateCluster();
      recovery_state_ = "GPS_GATE_REANCHOR_TIMEOUT";
      ROS_WARN("GPS gate reanchor ack timed out; recovery remains fail-closed");
    }
  }
  // Supervisor가 GPS callback 중단을 곧바로 stale fault로 오인하지 않도록
  // Coordinator 자체의 상태 heartbeat는 센서 수신과 독립적으로 유지한다.
  publishRecoveryStatus(recovery_state_, recovery_active_);
}

bool RelocalizationCoordinator::validatePose(
    const geometry_msgs::PoseWithCovarianceStamped& message,
    const ros::Time& now) const {
  if (message.header.stamp.isZero() ||
      message.header.frame_id != map_frame_ ||
      !std::isfinite(message.pose.pose.position.x) ||
      !std::isfinite(message.pose.pose.position.y) ||
      !std::isfinite(message.pose.pose.position.z) ||
      !finiteQuaternion(message.pose.pose.orientation)) {
    return false;
  }
  const double age = (now - message.header.stamp).toSec();
  return std::isfinite(age) && age >= -candidate_max_future_stamp_sec_ &&
         age <= candidate_max_message_age_sec_ &&
         std::isfinite(message.pose.covariance[0]) &&
         std::isfinite(message.pose.covariance[7]) &&
         message.pose.covariance[0] >= 0.0 &&
         message.pose.covariance[7] >= 0.0;
}

bool RelocalizationCoordinator::twistIsFresh(const ros::Time& now) const {
  if (!twist_.seen || !twist_.valid || twist_.receipt_time.isZero()) {
    return false;
  }
  const double receipt_age = (now - twist_.receipt_time).toSec();
  const double stamp_age = (now - twist_.message.header.stamp).toSec();
  return std::isfinite(receipt_age) && std::isfinite(stamp_age) &&
         receipt_age >= 0.0 && receipt_age <= twist_timeout_sec_ &&
         stamp_age >= -candidate_max_future_stamp_sec_ &&
         stamp_age <= twist_timeout_sec_;
}

bool RelocalizationCoordinator::globalOdometryIsFresh(
    const ros::Time& now) const {
  if (!global_odometry_.seen || !global_odometry_.valid ||
      global_odometry_.receipt_time.isZero()) {
    return false;
  }
  const double receipt_age = (now - global_odometry_.receipt_time).toSec();
  const double stamp_age = (now - global_odometry_.message.header.stamp).toSec();
  return std::isfinite(receipt_age) && std::isfinite(stamp_age) &&
         receipt_age >= 0.0 && receipt_age <= global_odometry_timeout_sec_ &&
         stamp_age >= -candidate_max_future_stamp_sec_ &&
         stamp_age <= global_odometry_timeout_sec_;
}

void RelocalizationCoordinator::updateCandidateCluster(
    const geometry_msgs::PoseWithCovarianceStamped& candidate) {
  if (!have_candidate_cluster_anchor_ ||
      planarDistance(candidate.pose.pose.position,
                     candidate_cluster_anchor_position_) >
          candidate_cluster_radius_m_) {
    consecutive_candidates_ = 1;
    candidate_cluster_anchor_position_ = candidate.pose.pose.position;
    have_candidate_cluster_anchor_ = true;
  } else {
    ++consecutive_candidates_;
  }
}

void RelocalizationCoordinator::resetCandidateCluster() {
  consecutive_candidates_ = 0;
  have_candidate_cluster_anchor_ = false;
}

bool RelocalizationCoordinator::requestGpsOnlyReset(
    const geometry_msgs::PoseWithCovarianceStamped& candidate,
    const ros::Time& now) {
  if (!set_pose_client_.waitForExistence(
          ros::Duration(set_pose_service_wait_timeout_sec_))) {
    publishRecoveryStatus("SET_POSE_SERVICE_UNAVAILABLE", true);
    return false;
  }

  robot_localization::SetPose service;
  service.request.pose.header.stamp = now;
  service.request.pose.header.frame_id = map_frame_;
  service.request.pose.pose = global_odometry_.message.pose;
  service.request.pose.pose.pose.position.x = candidate.pose.pose.position.x;
  service.request.pose.pose.pose.position.y = candidate.pose.pose.position.y;
  service.request.pose.pose.covariance[0] = candidate.pose.covariance[0];
  service.request.pose.pose.covariance[1] = candidate.pose.covariance[1];
  service.request.pose.pose.covariance[6] = candidate.pose.covariance[6];
  service.request.pose.pose.covariance[7] = candidate.pose.covariance[7];
  if (!set_pose_client_.call(service)) {
    publishRecoveryStatus("SET_POSE_CALL_FAILED", true);
    return false;
  }

  reset_target_ = candidate;
  reset_request_time_ = now;
  reset_pending_ = true;
  consecutive_global_confirmations_ = 0;
  last_global_confirmation_stamp_ = ros::Time(0);
  recovery_active_ = true;
  return true;
}

void RelocalizationCoordinator::requestGateReanchor(
    const geometry_msgs::PoseWithCovarianceStamped& candidate,
    const std::string& completion_state) {
  reanchor_target_ = candidate;
  pending_reanchor_transaction_id_ = next_reanchor_transaction_id_++;
  if (next_reanchor_transaction_id_ == 0U) {
    next_reanchor_transaction_id_ = 1U;
  }
  pending_completion_state_ = completion_state;
  reanchor_request_time_ = ros::Time::now();
  reanchor_pending_ = true;
  recovery_active_ = true;
  mando_localization::GpsGateReanchor command;
  command.transaction_id = pending_reanchor_transaction_id_;
  command.pose = reanchor_target_;
  gps_reanchor_publisher_.publish(command);
  publishRecoveryStatus("WAITING_FOR_GPS_GATE_REANCHOR", true);
}

void RelocalizationCoordinator::finishRecovery(
      const geometry_msgs::PoseWithCovarianceStamped& candidate,
      const std::string& state) {
  gps_pose_publisher_.publish(candidate);
  have_prior_anchor_ = true;
  last_gate_pose_receipt_time_ = ros::Time::now();
  reset_pending_ = false;
  reanchor_pending_ = false;
  pending_reanchor_transaction_id_ = 0U;
  recovery_active_ = false;
  gate_relocalizing_ = false;
  consecutive_global_confirmations_ = 0;
  last_global_confirmation_stamp_ = ros::Time(0);
  resetCandidateCluster();
  last_candidate_stamp_ = ros::Time(0);
  publishRecoveryStatus(state, false);
}

void RelocalizationCoordinator::publishRecoveryStatus(
    const std::string& state, const bool active) {
  recovery_state_ = state;
  recovery_active_ = active;
  std_msgs::String state_message;
  state_message.data = state;
  recovery_state_publisher_.publish(state_message);
  std_msgs::Bool active_message;
  active_message.data = active;
  recovery_active_publisher_.publish(active_message);
  gps_relocalizing_publisher_.publish(active_message);
}

}  // namespace mando_localization
