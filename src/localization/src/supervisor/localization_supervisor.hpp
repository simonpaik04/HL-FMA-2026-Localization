#pragma once

#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <string>

namespace mando_localization {

struct SupervisorInput {
  bool evaluation_received{false};
  bool evaluation_fresh{false};
  std::string evaluated_state{"INITIALIZING"};
  bool evaluated_valid{false};
  bool recovery_received{false};
  bool recovery_fresh{false};
  bool recovery_active{false};
};

struct SupervisorDecision {
  std::string state{"FAULT"};
  bool valid{false};
  std::string reason{"supervisor_input_missing"};
};

class LocalizationSupervisorArbiter {
 public:
  SupervisorDecision evaluate(const SupervisorInput& input) const;
};

/**
 * @brief StatusManager 평가와 Coordinator 복구 상태를 최종 중재한다.
 *
 * 공개 state/valid/status의 단일 소유자다. 평가나 복구 상태가 stale이면
 * fail-closed하며, 복구 중에는 StatusManager가 valid를 올려도 차단한다.
 */
class LocalizationSupervisor {
 public:
  LocalizationSupervisor(ros::NodeHandle nh, ros::NodeHandle private_nh);

 private:
  void evaluatedStatusCallback(
      const diagnostic_msgs::DiagnosticArrayConstPtr& message);
  void evaluatedStateCallback(const std_msgs::StringConstPtr& message);
  void evaluatedValidCallback(const std_msgs::BoolConstPtr& message);
  void recoveryActiveCallback(const std_msgs::BoolConstPtr& message);
  void recoveryStateCallback(const std_msgs::StringConstPtr& message);
  void timerCallback(const ros::TimerEvent& event);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber evaluated_status_subscriber_;
  ros::Subscriber evaluated_state_subscriber_;
  ros::Subscriber evaluated_valid_subscriber_;
  ros::Subscriber recovery_active_subscriber_;
  ros::Subscriber recovery_state_subscriber_;
  ros::Publisher status_publisher_;
  ros::Publisher state_publisher_;
  ros::Publisher valid_publisher_;
  ros::Timer timer_;
  LocalizationSupervisorArbiter arbiter_;

  bool have_status_{false};
  bool have_state_{false};
  bool have_valid_{false};
  bool have_recovery_active_{false};
  diagnostic_msgs::DiagnosticArray latest_status_;
  std::string evaluated_state_{"INITIALIZING"};
  bool evaluated_valid_{false};
  bool recovery_active_{false};
  std::string recovery_state_{"IDLE"};
  ros::Time status_receipt_time_;
  ros::Time state_receipt_time_;
  ros::Time valid_receipt_time_;
  ros::Time recovery_receipt_time_;
  double max_evaluation_age_sec_{0.5};
};

}  // namespace mando_localization
