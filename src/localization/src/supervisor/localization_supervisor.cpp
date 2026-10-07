#include "supervisor/localization_supervisor.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "common/parameter_utils.hpp"

namespace mando_localization {
namespace {

diagnostic_msgs::KeyValue keyValue(const std::string& key,
                                   const std::string& value) {
  diagnostic_msgs::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

std::string booleanText(const bool value) { return value ? "true" : "false"; }

bool fresh(const ros::Time& receipt, const ros::Time& now,
           const double timeout_sec) {
  if (receipt.isZero()) {
    return false;
  }
  const double age = (now - receipt).toSec();
  return std::isfinite(age) && age >= 0.0 && age <= timeout_sec;
}

}  // namespace

SupervisorDecision LocalizationSupervisorArbiter::evaluate(
    const SupervisorInput& input) const {
  if (!input.evaluation_received || !input.recovery_received) {
    return {"FAULT", false, "supervisor_input_missing"};
  }
  if (!input.evaluation_fresh || !input.recovery_fresh) {
    return {"FAULT", false, "supervisor_input_stale"};
  }
  if (input.recovery_active) {
    return {"RELOCALIZING", false, "recovery_coordinator_active"};
  }
  return {input.evaluated_state, input.evaluated_valid,
          "status_evaluation_forwarded"};
}

LocalizationSupervisor::LocalizationSupervisor(ros::NodeHandle nh,
                                               ros::NodeHandle private_nh)
    : nh_(std::move(nh)), private_nh_(std::move(private_nh)) {
  const std::string evaluated_status_topic = requireParameter<std::string>(
      private_nh_, "internal_topics/evaluated_status");
  const std::string evaluated_state_topic = requireParameter<std::string>(
      private_nh_, "internal_topics/evaluated_state");
  const std::string evaluated_valid_topic = requireParameter<std::string>(
      private_nh_, "internal_topics/evaluated_valid");
  const std::string recovery_active_topic = requireParameter<std::string>(
      private_nh_, "internal_topics/recovery_active");
  const std::string recovery_state_topic = requireParameter<std::string>(
      private_nh_, "topics/recovery_state");
  const std::string status_topic = requireParameter<std::string>(
      private_nh_, "topics/status");
  const std::string state_topic = requireParameter<std::string>(
      private_nh_, "topics/state");
  const std::string valid_topic = requireParameter<std::string>(
      private_nh_, "topics/valid");
  const double publish_rate_hz = requireParameter<double>(
      private_nh_, "manager_rate_hz");
  max_evaluation_age_sec_ = requireParameter<double>(
      private_nh_, "supervisor/max_evaluation_age_sec");
  if (!std::isfinite(publish_rate_hz) || publish_rate_hz <= 0.0 ||
      !std::isfinite(max_evaluation_age_sec_) ||
      max_evaluation_age_sec_ <= 0.0) {
    throw std::runtime_error("Supervisor rate 또는 timeout이 유효하지 않습니다.");
  }

  status_publisher_ =
      nh_.advertise<diagnostic_msgs::DiagnosticArray>(status_topic, 10);
  state_publisher_ = nh_.advertise<std_msgs::String>(state_topic, 1, true);
  valid_publisher_ = nh_.advertise<std_msgs::Bool>(valid_topic, 1, true);
  evaluated_status_subscriber_ = nh_.subscribe(
      evaluated_status_topic, 10,
      &LocalizationSupervisor::evaluatedStatusCallback, this);
  evaluated_state_subscriber_ = nh_.subscribe(
      evaluated_state_topic, 10,
      &LocalizationSupervisor::evaluatedStateCallback, this);
  evaluated_valid_subscriber_ = nh_.subscribe(
      evaluated_valid_topic, 10,
      &LocalizationSupervisor::evaluatedValidCallback, this);
  recovery_active_subscriber_ = nh_.subscribe(
      recovery_active_topic, 10,
      &LocalizationSupervisor::recoveryActiveCallback, this);
  recovery_state_subscriber_ = nh_.subscribe(
      recovery_state_topic, 10,
      &LocalizationSupervisor::recoveryStateCallback, this);
  timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_hz),
                           &LocalizationSupervisor::timerCallback, this);
}

void LocalizationSupervisor::evaluatedStatusCallback(
    const diagnostic_msgs::DiagnosticArrayConstPtr& message) {
  latest_status_ = *message;
  status_receipt_time_ = ros::Time::now();
  have_status_ = true;
}

void LocalizationSupervisor::evaluatedStateCallback(
    const std_msgs::StringConstPtr& message) {
  evaluated_state_ = message->data;
  state_receipt_time_ = ros::Time::now();
  have_state_ = true;
}

void LocalizationSupervisor::evaluatedValidCallback(
    const std_msgs::BoolConstPtr& message) {
  evaluated_valid_ = message->data;
  valid_receipt_time_ = ros::Time::now();
  have_valid_ = true;
}

void LocalizationSupervisor::recoveryActiveCallback(
    const std_msgs::BoolConstPtr& message) {
  recovery_active_ = message->data;
  recovery_receipt_time_ = ros::Time::now();
  have_recovery_active_ = true;
}

void LocalizationSupervisor::recoveryStateCallback(
    const std_msgs::StringConstPtr& message) {
  recovery_state_ = message->data;
}

void LocalizationSupervisor::timerCallback(const ros::TimerEvent&) {
  const ros::Time now = ros::Time::now();
  SupervisorInput input;
  input.evaluation_received = have_state_ && have_valid_;
  input.evaluation_fresh = fresh(state_receipt_time_, now,
                                 max_evaluation_age_sec_) &&
                           fresh(valid_receipt_time_, now,
                                 max_evaluation_age_sec_);
  input.evaluated_state = evaluated_state_;
  input.evaluated_valid = evaluated_valid_;
  input.recovery_received = have_recovery_active_;
  input.recovery_fresh = fresh(recovery_receipt_time_, now,
                               max_evaluation_age_sec_);
  input.recovery_active = recovery_active_;
  const SupervisorDecision decision = arbiter_.evaluate(input);

  std_msgs::String state;
  state.data = decision.state;
  state_publisher_.publish(state);
  std_msgs::Bool valid;
  valid.data = decision.valid;
  valid_publisher_.publish(valid);

  diagnostic_msgs::DiagnosticArray diagnostics;
  if (have_status_ &&
      fresh(status_receipt_time_, now, max_evaluation_age_sec_)) {
    diagnostics = latest_status_;
  }
  diagnostics.header.stamp = now;
  diagnostic_msgs::DiagnosticStatus supervisor;
  supervisor.name = "LOCALIZATION_SUPERVISOR";
  supervisor.hardware_id = "mando_localization";
  supervisor.level = decision.valid
      ? diagnostic_msgs::DiagnosticStatus::OK
      : diagnostic_msgs::DiagnosticStatus::ERROR;
  if (decision.state == "DEGRADED" ||
      decision.state == "DEAD_RECKONING") {
    supervisor.level = diagnostic_msgs::DiagnosticStatus::WARN;
  }
  supervisor.message = decision.state + ":" + decision.reason;
  supervisor.values.push_back(keyValue("valid", booleanText(decision.valid)));
  supervisor.values.push_back(keyValue("reason", decision.reason));
  supervisor.values.push_back(keyValue("recovery_state", recovery_state_));
  supervisor.values.push_back(
      keyValue("recovery_active", booleanText(recovery_active_)));
  diagnostics.status.insert(diagnostics.status.begin(), supervisor);
  status_publisher_.publish(diagnostics);
}

}  // namespace mando_localization
