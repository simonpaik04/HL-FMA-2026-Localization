#pragma once

#include <cstddef>

namespace mando_localization {

/**
 * @brief 절대 위치 센서의 최초 승인과 복귀 시 연속 정상 횟수를 관리한다.
 *
 * 이 클래스는 위치를 계산하지 않는다. 품질·innovation 검사를 통과했다는 호출만
 * 세며, 한 번 승인된 센서가 무효화되면 required_count회 연속 정상 전까지
 * recovering 상태를 유지한다.
 */
class ConsecutiveRecoveryGate {
 public:
  explicit ConsecutiveRecoveryGate(int required_count);

  bool observeHealthy();
  void markUnhealthy();
  void resetCandidate();
  void forceAccept();

  bool acceptedOnce() const { return accepted_once_; }
  bool recovering() const { return recovering_; }
  int consecutiveCount() const { return consecutive_count_; }
  int requiredCount() const { return required_count_; }

 private:
  int required_count_;
  int consecutive_count_{0};
  bool accepted_once_{false};
  bool recovering_{false};
};

}  // namespace mando_localization
