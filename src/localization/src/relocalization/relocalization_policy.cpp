#include "relocalization/relocalization_policy.hpp"

#include <cmath>
#include <stdexcept>

namespace mando_localization {

RelocalizationPolicy::RelocalizationPolicy(RelocalizationPolicyConfig config)
    : config_(config) {
  if (!std::isfinite(config_.max_stationary_speed_mps) ||
      config_.max_stationary_speed_mps < 0.0 ||
      config_.gps_only_required_consecutive_candidates <= 0) {
    throw std::invalid_argument("재정합 정책 파라미터가 유효하지 않습니다.");
  }
}

RelocalizationDecision RelocalizationPolicy::evaluate(
    const RelocalizationObservation& observation) const {
  if (!observation.have_prior_anchor || !observation.long_outage) {
    return {RecoveryAction::NONE, false, "IDLE", "no_long_outage"};
  }

  if (!config_.automatic_gps_only_reset_enabled) {
    return {RecoveryAction::NONE, true, "GPS_ONLY_BLOCKED",
            "automatic_gps_only_reset_disabled"};
  }
  if (config_.require_measured_datum && !config_.gps_datum_measured) {
    return {RecoveryAction::NONE, true, "GPS_ONLY_BLOCKED_DATUM",
            "measured_map_datum_required"};
  }
  if (!observation.twist_healthy ||
      !std::isfinite(observation.speed_mps) ||
      std::abs(observation.speed_mps) > config_.max_stationary_speed_mps) {
    return {RecoveryAction::NONE, true, "WAITING_FOR_STATIONARY",
            "vehicle_not_confirmed_stationary"};
  }
  if (!observation.global_odometry_healthy) {
    return {RecoveryAction::NONE, true, "WAITING_FOR_GLOBAL_ODOMETRY",
            "global_odometry_not_healthy"};
  }
  if (observation.consecutive_candidates <
      config_.gps_only_required_consecutive_candidates) {
    return {RecoveryAction::NONE, true, "VALIDATING_GPS_ONLY",
            "waiting_for_stable_gps_cluster"};
  }
  return {RecoveryAction::RESET_WITH_GPS, true, "GPS_ONLY_RESET_READY",
          "stable_stationary_gps_confirmed"};
}

}  // namespace mando_localization
