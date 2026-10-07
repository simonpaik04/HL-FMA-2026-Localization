#pragma once

#include <string>

namespace mando_localization {

enum class RecoveryAction {
  NONE,
  RESET_WITH_GPS,
};

struct RelocalizationPolicyConfig {
  double max_stationary_speed_mps{0.3};
  int gps_only_required_consecutive_candidates{5};
  bool automatic_gps_only_reset_enabled{false};
  bool gps_datum_measured{false};
  bool require_measured_datum{true};
};

struct RelocalizationObservation {
  bool have_prior_anchor{false};
  bool long_outage{false};
  bool twist_healthy{false};
  double speed_mps{0.0};
  bool global_odometry_healthy{false};
  int consecutive_candidates{0};
};

struct RelocalizationDecision {
  RecoveryAction action{RecoveryAction::NONE};
  bool relocalizing{false};
  std::string state{"IDLE"};
  std::string reason{"no_long_outage"};
};

/**
 * @brief 장기 GPS 단절 후보의 복구 전략만 결정하는 순수 정책.
 *
 * 명시적으로 허용된 정지 상태 GPS reset만 선택한다.
 * ROS callback, EKF service 호출과 메시지 발행은 Coordinator가 담당한다.
 */
class RelocalizationPolicy {
 public:
  explicit RelocalizationPolicy(RelocalizationPolicyConfig config);

  RelocalizationDecision evaluate(
      const RelocalizationObservation& observation) const;
  const RelocalizationPolicyConfig& config() const { return config_; }

 private:
  RelocalizationPolicyConfig config_;
};

}  // namespace mando_localization
