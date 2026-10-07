#include "localization_state_evaluator.hpp"

#include <cmath>
#include <stdexcept>

namespace mando_localization {

LocalizationStateEvaluator::LocalizationStateEvaluator(StatePolicy policy) : policy_(policy) {
  if (!std::isfinite(policy_.startup_grace_sec) || policy_.startup_grace_sec < 0.0 ||
      !std::isfinite(policy_.dead_reckoning_max_sec) ||
      policy_.dead_reckoning_max_sec <= 0.0 ||
      !std::isfinite(policy_.dead_reckoning_max_distance_m) ||
      policy_.dead_reckoning_max_distance_m <= 0.0) {
    throw std::invalid_argument("상태 정책의 시간/거리 값이 유효하지 않습니다.");
  }
}

StateDecision LocalizationStateEvaluator::evaluate(const StateInput& input) const {
  if (!input.local_motion_healthy) {
    if (input.uptime_sec <= policy_.startup_grace_sec) {
      return {LocalizationState::INITIALIZING, false, "waiting_for_local_motion"};
    }
    return {LocalizationState::FAULT, false, "local_motion_invalid"};
  }

  if (input.relocalizing) {
    return {LocalizationState::RELOCALIZING, false, "absolute_pose_relocalizing"};
  }

  if (!input.global_output_healthy) {
    if (input.uptime_sec <= policy_.startup_grace_sec) {
      return {LocalizationState::INITIALIZING, false, "waiting_for_global_output"};
    }
    return {LocalizationState::FAULT, false, "global_output_invalid"};
  }

  if (input.absolute_healthy_count > 0U) {
    if (input.absolute_healthy_count < input.absolute_enabled_count) {
      return {LocalizationState::DEGRADED, true, "absolute_source_degraded"};
    }
    return {LocalizationState::TRACKING, true, "all_enabled_sources_healthy"};
  }

  if (!input.anchor_seen || input.absolute_enabled_count == 0U) {
    return {LocalizationState::INITIALIZING, false, "waiting_for_absolute_anchor"};
  }

  if (input.seconds_since_absolute <= policy_.dead_reckoning_max_sec &&
      input.dead_reckoning_distance_m <= policy_.dead_reckoning_max_distance_m) {
    return {LocalizationState::DEAD_RECKONING, true, "bounded_dead_reckoning"};
  }

  if (input.seconds_since_absolute > policy_.dead_reckoning_max_sec) {
    return {LocalizationState::FAULT, false, "dead_reckoning_time_exceeded"};
  }
  return {LocalizationState::FAULT, false, "dead_reckoning_distance_exceeded"};
}

const char* LocalizationStateEvaluator::toString(const LocalizationState state) {
  switch (state) {
    case LocalizationState::INITIALIZING:
      return "INITIALIZING";
    case LocalizationState::TRACKING:
      return "TRACKING";
    case LocalizationState::DEGRADED:
      return "DEGRADED";
    case LocalizationState::DEAD_RECKONING:
      return "DEAD_RECKONING";
    case LocalizationState::RELOCALIZING:
      return "RELOCALIZING";
    case LocalizationState::FAULT:
      return "FAULT";
  }
  return "FAULT";
}

}  // namespace mando_localization
