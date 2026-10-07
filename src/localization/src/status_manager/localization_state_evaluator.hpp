#pragma once

#include <cstddef>
#include <string>

namespace mando_localization {

enum class LocalizationState {
  INITIALIZING,
  TRACKING,
  DEGRADED,
  DEAD_RECKONING,
  RELOCALIZING,
  FAULT,
};

struct StatePolicy {
  double startup_grace_sec{3.0};
  double dead_reckoning_max_sec{2.0};
  double dead_reckoning_max_distance_m{10.0};
};

struct StateInput {
  double uptime_sec{0.0};
  bool local_motion_healthy{false};
  // Global EKF 출력은 로컬 IMU/엔코더 건전성과 별도로 판단한다.
  // 이를 분리해야 절대 위치 복구 중에는 RELOCALIZING을 표시하면서도
  // 아직 보정되지 않은 최종 위치를 fail-closed로 차단할 수 있다.
  bool global_output_healthy{false};
  bool relocalizing{false};
  bool anchor_seen{false};
  std::size_t absolute_enabled_count{0U};
  std::size_t absolute_healthy_count{0U};
  double seconds_since_absolute{0.0};
  double dead_reckoning_distance_m{0.0};
};

struct StateDecision {
  LocalizationState state{LocalizationState::INITIALIZING};
  bool valid{false};
  std::string reason{"waiting_for_inputs"};
};

/**
 * @brief 센서 상태만으로 전체 Localization 상태를 결정한다.
 *
 * 위치를 계산하거나 TF를 발행하지 않는다. 시간과 이동거리 예산은 호출자가
 * 측정해 전달하며, 이 클래스는 동일 입력에 항상 동일 결정을 반환한다.
 */
class LocalizationStateEvaluator {
 public:
  explicit LocalizationStateEvaluator(StatePolicy policy);

  StateDecision evaluate(const StateInput& input) const;
  const StatePolicy& policy() const { return policy_; }

  static const char* toString(LocalizationState state);

 private:
  StatePolicy policy_;
};

}  // namespace mando_localization
