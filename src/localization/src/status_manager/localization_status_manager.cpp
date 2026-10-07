#include "localization_status_manager.hpp"

#include <ros/master.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace mando_localization {
namespace {

template <typename T>
T requireParameter(const ros::NodeHandle& node, const std::string& name) {
  T value;
  if (!node.getParam(name, value)) {
    throw std::runtime_error("필수 파라미터가 없습니다: " + node.resolveName(name));
  }
  return value;
}

bool finiteQuaternion(const geometry_msgs::Quaternion& quaternion) {
  const double norm_squared = quaternion.x * quaternion.x + quaternion.y * quaternion.y +
                              quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  return std::isfinite(norm_squared) && norm_squared > 1.0e-12;
}

diagnostic_msgs::KeyValue keyValue(const std::string& key, const std::string& value) {
  diagnostic_msgs::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

std::string booleanText(const bool value) { return value ? "true" : "false"; }

}  // namespace

LocalizationStatusManager::LocalizationStatusManager(ros::NodeHandle nh,
                                                     ros::NodeHandle private_nh)
    : nh_(std::move(nh)),
      private_nh_(std::move(private_nh)),
      evaluator_([&]() {
        StatePolicy policy;
        policy.startup_grace_sec =
            requireParameter<double>(private_nh_, "startup_grace_sec");
        policy.dead_reckoning_max_sec =
            requireParameter<double>(private_nh_, "dead_reckoning/max_duration_sec");
        policy.dead_reckoning_max_distance_m =
            requireParameter<double>(private_nh_, "dead_reckoning/max_distance_m");
        return policy;
      }()) {
  const std::string imu_topic =
      requireParameter<std::string>(private_nh_, "topics/imu_calibrated");
  const std::string encoder_topic =
      requireParameter<std::string>(private_nh_, "topics/encoder_state");
  const std::string twist_topic =
      requireParameter<std::string>(private_nh_, "topics/encoder_twist");
  const std::string local_odometry_topic =
      requireParameter<std::string>(private_nh_, "topics/local_odometry");
  const std::string gps_pose_topic =
      requireParameter<std::string>(private_nh_, "topics/gps_map_pose");
  const std::string global_odometry_topic =
      requireParameter<std::string>(private_nh_, "topics/global_odometry");
  const std::string gps_relocalizing_topic =
      requireParameter<std::string>(private_nh_, "topics/gps_relocalizing");
  const std::string diagnostics_topic = requireParameter<std::string>(
      private_nh_, "internal_topics/evaluated_status");
  const std::string state_topic = requireParameter<std::string>(
      private_nh_, "internal_topics/evaluated_state");
  const std::string valid_topic = requireParameter<std::string>(
      private_nh_, "internal_topics/evaluated_valid");

  enable_gps_ = requireParameter<bool>(private_nh_, "absolute_sources/gps_enabled");
  imu_timeout_sec_ = requireParameter<double>(private_nh_, "sources/imu/timeout_sec");
  imu_max_future_sec_ =
      requireParameter<double>(private_nh_, "sources/imu/max_future_stamp_sec");
  encoder_timeout_sec_ =
      requireParameter<double>(private_nh_, "sources/encoder/timeout_sec");
  encoder_max_future_sec_ = requireParameter<double>(
      private_nh_, "sources/encoder/max_future_stamp_sec");
  local_odometry_timeout_sec_ =
      requireParameter<double>(private_nh_, "sources/local_odometry/timeout_sec");
  local_odometry_max_future_sec_ = requireParameter<double>(
      private_nh_, "sources/local_odometry/max_future_stamp_sec");
  gps_timeout_sec_ = requireParameter<double>(private_nh_, "sources/gps/timeout_sec");
  gps_max_future_sec_ =
      requireParameter<double>(private_nh_, "sources/gps/max_future_stamp_sec");
  global_odometry_timeout_sec_ =
      requireParameter<double>(private_nh_, "sources/global_odometry/timeout_sec");
  global_odometry_max_future_sec_ = requireParameter<double>(
      private_nh_, "sources/global_odometry/max_future_stamp_sec");
  status_publish_rate_hz_ = requireParameter<double>(private_nh_, "manager_rate_hz");
  max_position_variance_m2_ =
      requireParameter<double>(private_nh_, "quality/max_position_variance_m2");
  max_global_consistency_distance_m_ = requireParameter<double>(
      private_nh_, "absolute_sources/max_global_consistency_distance_m");
  const std::string dead_reckoning_limit_policy = requireParameter<std::string>(
      private_nh_, "dead_reckoning/limit_policy");
  map_frame_ = requireParameter<std::string>(private_nh_, "frames/map");
  odom_frame_ = requireParameter<std::string>(private_nh_, "frames/odom");
  base_frame_ = requireParameter<std::string>(private_nh_, "frames/base_link");
  imu_device_path_ = requireParameter<std::string>(private_nh_, "devices/imu");
  gps_device_path_ = requireParameter<std::string>(private_nh_, "devices/gps");
  imu_driver_node_ = requireParameter<std::string>(private_nh_, "nodes/imu_driver");
  gps_driver_node_ = requireParameter<std::string>(private_nh_, "nodes/gps_driver");
  private_nh_.param("initialization/required", initialization_required_, false);
  private_nh_.param("initialization/ready_timeout_sec",
                    initialization_ready_timeout_sec_, 0.5);
  private_nh_.param("initialization/max_pose_age_sec",
                    initialization_pose_max_age_sec_, 1.0);

  const std::vector<double> positive_values = {
      imu_timeout_sec_, encoder_timeout_sec_, local_odometry_timeout_sec_,
      gps_timeout_sec_, global_odometry_timeout_sec_,
      status_publish_rate_hz_, max_position_variance_m2_,
      max_global_consistency_distance_m_,
      initialization_ready_timeout_sec_, initialization_pose_max_age_sec_};
  const std::vector<double> nonnegative_values = {
      imu_max_future_sec_, encoder_max_future_sec_,
      local_odometry_max_future_sec_, gps_max_future_sec_,
      global_odometry_max_future_sec_};
  if (map_frame_.empty() || odom_frame_.empty() || base_frame_.empty() ||
      std::any_of(positive_values.begin(), positive_values.end(),
                  [](double value) { return !std::isfinite(value) || value <= 0.0; }) ||
      std::any_of(nonnegative_values.begin(), nonnegative_values.end(),
                  [](double value) { return !std::isfinite(value) || value < 0.0; }) ||
      dead_reckoning_limit_policy != "first_exceeded") {
    throw std::runtime_error("상태 관리자 frame 또는 양수 파라미터가 유효하지 않습니다.");
  }

  imu_subscriber_ = nh_.subscribe(imu_topic, 50, &LocalizationStatusManager::imuCallback, this);
  encoder_subscriber_ =
      nh_.subscribe(encoder_topic, 50, &LocalizationStatusManager::encoderCallback, this);
  twist_subscriber_ =
      nh_.subscribe(twist_topic, 50, &LocalizationStatusManager::twistCallback, this);
  local_odometry_subscriber_ = nh_.subscribe(
      local_odometry_topic, 50, &LocalizationStatusManager::localOdometryCallback, this);
  if (enable_gps_) {
    gps_pose_subscriber_ = nh_.subscribe(gps_pose_topic, 10,
                                        &LocalizationStatusManager::gpsPoseCallback, this);
  }
  global_odometry_subscriber_ = nh_.subscribe(
      global_odometry_topic, 50, &LocalizationStatusManager::globalOdometryCallback, this);
  gps_relocalizing_subscriber_ = nh_.subscribe(
      gps_relocalizing_topic, 10,
      &LocalizationStatusManager::gpsRelocalizingCallback, this);
  if (initialization_required_) {
    initialization_ready_subscriber_ = nh_.subscribe(
        "/mando_localization/internal/initialization/ready", 10,
        &LocalizationStatusManager::initializationReadyCallback, this);
    initialization_pose_subscriber_ = nh_.subscribe(
        "/mando_localization/internal/initialization/committed_pose", 10,
        &LocalizationStatusManager::initializationPoseCallback, this);
  }

  diagnostics_publisher_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic, 10);
  state_publisher_ = nh_.advertise<std_msgs::String>(state_topic, 1, true);
  valid_publisher_ = nh_.advertise<std_msgs::Bool>(valid_topic, 1, true);
  timer_ = nh_.createTimer(ros::Duration(1.0 / status_publish_rate_hz_),
                           &LocalizationStatusManager::timerCallback, this);
  start_time_ = ros::Time::now();

  ROS_INFO_STREAM("LocalizationStatusManager 설정: GPS=" << booleanText(enable_gps_)
                  << ", DR=" << evaluator_.policy().dead_reckoning_max_sec << " s/"
                  << evaluator_.policy().dead_reckoning_max_distance_m << " m");
}

void LocalizationStatusManager::imuCallback(const sensor_msgs::ImuConstPtr& message) {
  imu_status_.seen = true;
  imu_status_.receipt_time = ros::Time::now();
  imu_status_.stamp = message->header.stamp;
  imu_status_.frame_id = message->header.frame_id;
  const bool finite_angular = std::isfinite(message->angular_velocity.x) &&
                              std::isfinite(message->angular_velocity.y) &&
                              std::isfinite(message->angular_velocity.z);
  imu_status_.payload_valid = !message->header.stamp.isZero() &&
                              !message->header.frame_id.empty() && finite_angular &&
                              finiteQuaternion(message->orientation);
  imu_status_.reason = imu_status_.payload_valid ? "ok" : "invalid_stamp_frame_or_value";
}

void LocalizationStatusManager::encoderCallback(
    const erp42_msgs::SerialFeedBackConstPtr& message) {
  const ros::Time receipt_time = ros::Time::now();
  const bool alive_advanced = !have_encoder_alive_counter_ ||
                              message->alive != encoder_alive_counter_;
  encoder_status_.seen = true;
  encoder_status_.receipt_time = receipt_time;
  // SerialFeedBack has no Header. Receipt time and base_link are the explicit
  // host-side contract used for freshness diagnostics.
  encoder_status_.stamp = receipt_time;
  encoder_status_.frame_id = base_frame_;
  encoder_status_.payload_valid = alive_advanced &&
                                  std::isfinite(message->speed) &&
                                  std::isfinite(message->steer);
  if (std::isfinite(message->steer) && message->steer >= 0.0 &&
      message->steer <= 65535.0) {
    steering_adc_ = static_cast<uint16_t>(std::lround(message->steer));
  }
  encoder_delta_100ms_ = message->encoder;
  brake_ = message->brake != 0;
  encoder_alive_counter_ = message->alive;
  have_encoder_alive_counter_ = true;
  encoder_status_.reason = encoder_status_.payload_valid
                               ? "ok"
                               : "alive_counter_stalled_or_invalid_value";
}

void LocalizationStatusManager::twistCallback(
    const geometry_msgs::TwistWithCovarianceStampedConstPtr& message) {
  twist_status_.seen = true;
  twist_status_.receipt_time = ros::Time::now();
  twist_status_.stamp = message->header.stamp;
  twist_status_.frame_id = message->header.frame_id;
  bool finite_covariance = true;
  for (const double value : message->twist.covariance) {
    finite_covariance = finite_covariance && std::isfinite(value);
  }
  twist_status_.payload_valid = !message->header.stamp.isZero() &&
                                message->header.frame_id == base_frame_ &&
                                std::isfinite(message->twist.twist.linear.x) &&
                                finite_covariance;
  twist_status_.reason = twist_status_.payload_valid ? "ok" : "invalid_twist";
}

void LocalizationStatusManager::localOdometryCallback(
    const nav_msgs::OdometryConstPtr& message) {
  local_odometry_status_.seen = true;
  local_odometry_status_.receipt_time = ros::Time::now();
  local_odometry_status_.stamp = message->header.stamp;
  local_odometry_status_.frame_id = message->header.frame_id;
  local_odometry_status_.payload_valid =
      !message->header.stamp.isZero() && message->header.frame_id == odom_frame_ &&
      message->child_frame_id == base_frame_ && validatePose(message->pose.pose) &&
      validatePoseCovariance(message->pose.covariance, false);
  local_odometry_status_.reason =
      local_odometry_status_.payload_valid ? "ok" : "invalid_local_odometry";
}

void LocalizationStatusManager::gpsPoseCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& message) {
  gps_status_.seen = true;
  gps_status_.receipt_time = ros::Time::now();
  gps_status_.stamp = message->header.stamp;
  gps_status_.frame_id = message->header.frame_id;
  gps_status_.payload_valid = !message->header.stamp.isZero() &&
                              message->header.frame_id == map_frame_ &&
                              validatePose(message->pose.pose) &&
                              validatePoseCovariance(message->pose.covariance);
  gps_status_.reason = gps_status_.payload_valid ? "ok" : "invalid_map_pose";
  gps_position_ = message->pose.pose.position;
  if (isFresh(gps_status_, gps_timeout_sec_, gps_max_future_sec_,
              gps_status_.receipt_time) &&
      (!initialization_required_ ||
       (initializationReady(gps_status_.receipt_time) && initialization_anchor_applied_))) {
    markAbsoluteMeasurement(gps_status_.receipt_time);
  }
}

void LocalizationStatusManager::globalOdometryCallback(const nav_msgs::OdometryConstPtr& message) {
  global_odometry_status_.seen = true;
  global_odometry_status_.receipt_time = ros::Time::now();
  global_odometry_status_.stamp = message->header.stamp;
  global_odometry_status_.frame_id = message->header.frame_id;
  global_odometry_status_.payload_valid =
      !message->header.stamp.isZero() && message->header.frame_id == map_frame_ &&
      message->child_frame_id == base_frame_ && validatePose(message->pose.pose) &&
      validatePoseCovariance(message->pose.covariance);
  global_odometry_status_.reason =
      global_odometry_status_.payload_valid ? "ok" : "invalid_global_odometry";
  global_position_ = message->pose.pose.position;

  if (!global_odometry_status_.payload_valid) {
    return;
  }
  const geometry_msgs::Point& current = message->pose.pose.position;
  if (dead_reckoning_active_ && have_previous_global_position_) {
    const double dx = current.x - previous_global_position_.x;
    const double dy = current.y - previous_global_position_.y;
    const double increment = std::hypot(dx, dy);
    if (std::isfinite(increment)) {
      dead_reckoning_distance_m_ += increment;
    }
  }
  previous_global_position_ = current;
  have_previous_global_position_ = true;
}

void LocalizationStatusManager::gpsRelocalizingCallback(
    const std_msgs::BoolConstPtr& message) {
  gps_relocalizing_ = message->data;
}

bool LocalizationStatusManager::initializationReady(const ros::Time& now) const {
  if (!initialization_required_) return true;
  if (!initialization_ready_ || initialization_ready_receipt_time_.isZero() ||
      initialization_ready_wall_time_.isZero()) return false;
  const double age = (now - initialization_ready_receipt_time_).toSec();
  const double wall_age = (ros::WallTime::now() - initialization_ready_wall_time_).toSec();
  return age >= 0.0 && age <= initialization_ready_timeout_sec_ &&
         wall_age >= 0.0 && wall_age <= initialization_ready_timeout_sec_;
}

void LocalizationStatusManager::clearInitializationAnchor() {
  initialization_pose_status_ = StreamStatus();
  initialization_pose_wall_time_ = ros::WallTime();
  initialization_anchor_applied_ = false;
  anchor_seen_ = false;
  last_absolute_time_ = ros::Time();
  dead_reckoning_active_ = false;
  dead_reckoning_distance_m_ = 0.0;
  have_previous_global_position_ = false;
  gps_status_ = StreamStatus();
}

void LocalizationStatusManager::applyInitializationAnchor(const ros::Time& now) {
  if (!initialization_required_ || initialization_anchor_applied_ ||
      !initializationReady(now) ||
      !isFresh(initialization_pose_status_, initialization_pose_max_age_sec_,
               global_odometry_max_future_sec_, now) ||
      initialization_pose_wall_time_.isZero()) return;
  const double wall_age = (ros::WallTime::now() - initialization_pose_wall_time_).toSec();
  if (wall_age < 0.0 || wall_age > initialization_pose_max_age_sec_) return;
  // A committed operator/RDDF pose is a one-time anchor, never a healthy GPS
  // stream. Heartbeats and repeated latched messages must not extend the DR budget.
  markAbsoluteMeasurement(initialization_pose_status_.stamp);
  initialization_anchor_applied_ = true;
}

void LocalizationStatusManager::initializationReadyCallback(
    const std_msgs::BoolConstPtr& message) {
  const ros::Time now = ros::Time::now();
  if (!message->data && (initialization_ready_ || initialization_anchor_applied_)) {
    clearInitializationAnchor();
  }
  initialization_ready_ = message->data;
  initialization_ready_receipt_time_ = now;
  initialization_ready_wall_time_ = ros::WallTime::now();
  applyInitializationAnchor(now);
}

void LocalizationStatusManager::initializationPoseCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& message) {
  if (initialization_anchor_applied_) return;
  const ros::Time now = ros::Time::now();
  initialization_pose_status_.seen = true;
  initialization_pose_status_.receipt_time = now;
  initialization_pose_status_.stamp = message->header.stamp;
  initialization_pose_status_.frame_id = message->header.frame_id;
  initialization_pose_wall_time_ = ros::WallTime::now();
  const auto& orientation = message->pose.pose.orientation;
  const double norm_squared = orientation.x * orientation.x + orientation.y * orientation.y +
                              orientation.z * orientation.z + orientation.w * orientation.w;
  bool covariance_valid = validatePoseCovariance(message->pose.covariance);
  for (std::size_t index = 0; index < 6; ++index) {
    covariance_valid = covariance_valid && message->pose.covariance[index * 7] >= 0.0;
  }
  initialization_pose_status_.payload_valid =
      !message->header.stamp.isZero() && message->header.frame_id == map_frame_ &&
      validatePose(message->pose.pose) && std::abs(norm_squared - 1.0) <= 0.002 &&
      covariance_valid;
  initialization_pose_status_.reason = initialization_pose_status_.payload_valid
      ? "committed_rddf_pose" : "invalid_committed_rddf_pose";
  applyInitializationAnchor(now);
}

void LocalizationStatusManager::timerCallback(const ros::TimerEvent&) {
  const ros::Time now = ros::Time::now();
  applyInitializationAnchor(now);
  const bool startup_ready = initializationReady(now) &&
      (!initialization_required_ || initialization_anchor_applied_);
  const bool imu_healthy =
      isFresh(imu_status_, imu_timeout_sec_, imu_max_future_sec_, now);
  const bool encoder_healthy = isFresh(
      encoder_status_, encoder_timeout_sec_, encoder_max_future_sec_, now);
  const bool twist_healthy = isFresh(
      twist_status_, encoder_timeout_sec_, encoder_max_future_sec_, now);
  const bool local_odometry_healthy =
      isFresh(local_odometry_status_, local_odometry_timeout_sec_,
              local_odometry_max_future_sec_, now);
  const bool global_healthy = isFresh(
      global_odometry_status_, global_odometry_timeout_sec_,
      global_odometry_max_future_sec_, now);
  const bool gps_healthy = enable_gps_ &&
      isFresh(gps_status_, gps_timeout_sec_, gps_max_future_sec_, now);

  const bool gps_unconfirmed = gps_healthy && global_healthy &&
      std::hypot(gps_position_.x - global_position_.x,
                 gps_position_.y - global_position_.y) >
          max_global_consistency_distance_m_;

  const std::size_t enabled_count = static_cast<std::size_t>(enable_gps_);
  const std::size_t healthy_count = static_cast<std::size_t>(gps_healthy);
  dead_reckoning_active_ = anchor_seen_ && healthy_count == 0U;
  if (!dead_reckoning_active_) {
    dead_reckoning_distance_m_ = 0.0;
  }

  StateInput input;
  input.uptime_sec = std::max(0.0, (now - start_time_).toSec());
  input.local_motion_healthy = imu_healthy && encoder_healthy && twist_healthy &&
                               local_odometry_healthy;
  input.global_output_healthy = global_healthy;
  input.relocalizing = (enable_gps_ && gps_relocalizing_) || gps_unconfirmed;
  input.anchor_seen = anchor_seen_;
  input.absolute_enabled_count = enabled_count;
  input.absolute_healthy_count = healthy_count;
  input.seconds_since_absolute =
      anchor_seen_ ? std::max(0.0, (now - last_absolute_time_).toSec()) : 0.0;
  input.dead_reckoning_distance_m = dead_reckoning_distance_m_;
  const StateDecision decision = startup_ready ? evaluator_.evaluate(input)
      : StateDecision{LocalizationState::INITIALIZING, false,
                      "waiting_for_rddf_initialization"};

  std_msgs::String state_message;
  state_message.data = LocalizationStateEvaluator::toString(decision.state);
  state_publisher_.publish(state_message);
  std_msgs::Bool valid_message;
  valid_message.data = decision.valid;
  valid_publisher_.publish(valid_message);

  diagnostic_msgs::DiagnosticArray diagnostics;
  diagnostics.header.stamp = now;
  diagnostics.status.push_back(
      makeStreamDiagnostic("IMU", imu_status_, imu_timeout_sec_,
                           imu_max_future_sec_, now, true));
  diagnostic_msgs::DiagnosticStatus encoder_diagnostic =
      makeStreamDiagnostic("ENCODER", encoder_status_, encoder_timeout_sec_,
                           encoder_max_future_sec_, now, true);
  encoder_diagnostic.values.push_back(
      keyValue("steering_adc", std::to_string(steering_adc_)));
  encoder_diagnostic.values.push_back(
      keyValue("encoder_delta_100ms", std::to_string(encoder_delta_100ms_)));
  encoder_diagnostic.values.push_back(keyValue("brake", booleanText(brake_)));
  diagnostics.status.push_back(encoder_diagnostic);
  diagnostics.status.push_back(
      makeStreamDiagnostic("ENCODER_TWIST", twist_status_, encoder_timeout_sec_,
                           encoder_max_future_sec_, now, true));
  diagnostics.status.push_back(makeStreamDiagnostic(
      "LOCAL_ODOMETRY", local_odometry_status_, local_odometry_timeout_sec_,
      local_odometry_max_future_sec_, now, true));
  diagnostics.status.push_back(
      makeStreamDiagnostic("GPS_POSE", gps_status_, gps_timeout_sec_,
                           gps_max_future_sec_, now, enable_gps_));
  diagnostics.status.push_back(makeStreamDiagnostic(
      "GLOBAL_ODOMETRY", global_odometry_status_, global_odometry_timeout_sec_,
      global_odometry_max_future_sec_, now, true));
  if (initialization_required_) {
    diagnostic_msgs::DiagnosticStatus startup;
    startup.name = "RDDF_INITIALIZATION";
    startup.hardware_id = "mando_localization";
    startup.level = startup_ready ? diagnostic_msgs::DiagnosticStatus::OK
                                  : diagnostic_msgs::DiagnosticStatus::WARN;
    startup.message = startup_ready ? "COMMITTED" : "WAITING_FOR_INITIALIZATION";
    startup.values.push_back(keyValue("ready_fresh", booleanText(initializationReady(now))));
    startup.values.push_back(keyValue("anchor_applied", booleanText(initialization_anchor_applied_)));
    startup.values.push_back(keyValue("pose_reason", initialization_pose_status_.reason));
    startup.values.push_back(keyValue("anchor_kind", "one_time_rddf_pose_not_gps"));
    diagnostics.status.push_back(startup);
  }

  diagnostics.status.push_back(
      makeDeviceDiagnostic("IMU_DEVICE", imu_device_path_, true));
  diagnostics.status.push_back(
      makeDeviceDiagnostic("GPS_DEVICE", gps_device_path_, enable_gps_));
  std::vector<std::string> running_nodes;
  ros::master::getNodes(running_nodes);
  diagnostics.status.push_back(
      makeDriverDiagnostic("IMU_DRIVER", imu_driver_node_, true, running_nodes));
  diagnostics.status.push_back(
      makeDriverDiagnostic("GPS_DRIVER", gps_driver_node_, enable_gps_, running_nodes));

  diagnostic_msgs::DiagnosticStatus overall;
  overall.name = "LOCALIZATION";
  overall.hardware_id = "mando_localization";
  overall.level = decision.valid ? diagnostic_msgs::DiagnosticStatus::OK
                                 : diagnostic_msgs::DiagnosticStatus::ERROR;
  if (decision.state == LocalizationState::DEGRADED ||
      decision.state == LocalizationState::DEAD_RECKONING) {
    overall.level = diagnostic_msgs::DiagnosticStatus::WARN;
  }
  overall.message = state_message.data + ":" + decision.reason;
  overall.values.push_back(keyValue("valid", booleanText(decision.valid)));
  overall.values.push_back(keyValue("reason", decision.reason));
  overall.values.push_back(
      keyValue("dead_reckoning_sec", std::to_string(input.seconds_since_absolute)));
  overall.values.push_back(
      keyValue("dead_reckoning_distance_m", std::to_string(dead_reckoning_distance_m_)));
  overall.values.push_back(keyValue("gps_healthy", booleanText(gps_healthy)));
  overall.values.push_back(
      keyValue("gps_correction_confirmed", booleanText(!gps_unconfirmed)));
  diagnostics.status.insert(diagnostics.status.begin(), overall);
  diagnostics_publisher_.publish(diagnostics);
}

bool LocalizationStatusManager::isFresh(const StreamStatus& stream,
                                        const double timeout_sec,
                                        const double max_future_sec,
                                        const ros::Time& now) const {
  if (!stream.seen || !stream.payload_valid || stream.receipt_time.isZero() ||
      stream.stamp.isZero()) {
    return false;
  }
  const double receipt_age = (now - stream.receipt_time).toSec();
  const double stamp_age = (now - stream.stamp).toSec();
  return receipt_age >= 0.0 && receipt_age <= timeout_sec &&
         stamp_age >= -max_future_sec &&
         stamp_age <= timeout_sec;
}

bool LocalizationStatusManager::validatePose(const geometry_msgs::Pose& pose) const {
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && finiteQuaternion(pose.orientation);
}

bool LocalizationStatusManager::validatePoseCovariance(
    const boost::array<double, 36>& covariance,
    const bool enforce_position_limit) const {
  for (const double value : covariance) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return covariance[0] >= 0.0 && covariance[7] >= 0.0 &&
         (!enforce_position_limit ||
          (covariance[0] <= max_position_variance_m2_ &&
           covariance[7] <= max_position_variance_m2_));
}

void LocalizationStatusManager::markAbsoluteMeasurement(const ros::Time& receipt_time) {
  anchor_seen_ = true;
  last_absolute_time_ = receipt_time;
  dead_reckoning_distance_m_ = 0.0;
  dead_reckoning_active_ = false;
}

diagnostic_msgs::DiagnosticStatus LocalizationStatusManager::makeStreamDiagnostic(
    const std::string& name, const StreamStatus& stream, const double timeout_sec,
    const double max_future_sec, const ros::Time& now, const bool enabled) const {
  diagnostic_msgs::DiagnosticStatus status;
  status.name = name;
  status.hardware_id = "mando_localization";
  if (!enabled) {
    status.level = diagnostic_msgs::DiagnosticStatus::OK;
    status.message = "DISABLED";
  } else if (isFresh(stream, timeout_sec, max_future_sec, now)) {
    status.level = diagnostic_msgs::DiagnosticStatus::OK;
    status.message = "OK";
  } else if (!stream.seen) {
    status.level = diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = "NOT_RECEIVED";
  } else {
    status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    status.message = stream.reason;
  }
  const double receipt_age = stream.seen ? (now - stream.receipt_time).toSec() : -1.0;
  const double stamp_age = stream.seen ? (now - stream.stamp).toSec() : -1.0;
  status.values.push_back(keyValue("enabled", booleanText(enabled)));
  status.values.push_back(keyValue("received", booleanText(stream.seen)));
  status.values.push_back(keyValue("payload_valid", booleanText(stream.payload_valid)));
  status.values.push_back(keyValue("receipt_age_sec", std::to_string(receipt_age)));
  status.values.push_back(keyValue("stamp_age_sec", std::to_string(stamp_age)));
  status.values.push_back(keyValue("frame_id", stream.frame_id));
  return status;
}

diagnostic_msgs::DiagnosticStatus LocalizationStatusManager::makeDeviceDiagnostic(
    const std::string& name, const std::string& path, const bool required) const {
  diagnostic_msgs::DiagnosticStatus status;
  status.name = name;
  status.hardware_id = path;
  const bool present = !path.empty() && ::access(path.c_str(), F_OK) == 0;
  status.level = (!required || present) ? diagnostic_msgs::DiagnosticStatus::OK
                                        : diagnostic_msgs::DiagnosticStatus::WARN;
  status.message = present ? "CONNECTED" : (required ? "NOT_CONNECTED" : "OPTIONAL_ABSENT");
  status.values.push_back(keyValue("path", path));
  status.values.push_back(keyValue("required", booleanText(required)));
  return status;
}

diagnostic_msgs::DiagnosticStatus LocalizationStatusManager::makeDriverDiagnostic(
    const std::string& name, const std::string& expected_node, const bool required,
    const std::vector<std::string>& running_nodes) const {
  diagnostic_msgs::DiagnosticStatus status;
  status.name = name;
  status.hardware_id = expected_node;
  const std::string resolved_node =
      (!expected_node.empty() && expected_node.front() == '/') ? expected_node : "/" + expected_node;
  const bool running = std::find(running_nodes.begin(), running_nodes.end(), resolved_node) !=
                       running_nodes.end();
  status.level = (!required || running) ? diagnostic_msgs::DiagnosticStatus::OK
                                        : diagnostic_msgs::DiagnosticStatus::WARN;
  status.message = running ? "RUNNING" : (required ? "NOT_RUNNING" : "OPTIONAL_STOPPED");
  status.values.push_back(keyValue("expected_node", resolved_node));
  return status;
}

}  // namespace mando_localization
