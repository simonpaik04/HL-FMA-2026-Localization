#include "common/consecutive_recovery_gate.hpp"

#include <stdexcept>

namespace mando_localization {

ConsecutiveRecoveryGate::ConsecutiveRecoveryGate(const int required_count)
    : required_count_(required_count) {
  if (required_count_ <= 0) {
    throw std::invalid_argument("연속 복구 횟수는 1 이상이어야 합니다.");
  }
}

bool ConsecutiveRecoveryGate::observeHealthy() {
  if (accepted_once_ && !recovering_) {
    return true;
  }

  ++consecutive_count_;
  if (consecutive_count_ < required_count_) {
    return false;
  }

  consecutive_count_ = required_count_;
  accepted_once_ = true;
  recovering_ = false;
  return true;
}

void ConsecutiveRecoveryGate::markUnhealthy() {
  consecutive_count_ = 0;
  recovering_ = accepted_once_;
}

void ConsecutiveRecoveryGate::resetCandidate() {
  consecutive_count_ = 0;
  // 최초 위치 대기 중에는 INITIALIZING이며, 한 번 승인된 뒤에만 재정합 상태다.
  recovering_ = accepted_once_;
}

void ConsecutiveRecoveryGate::forceAccept() {
  consecutive_count_ = required_count_;
  accepted_once_ = true;
  recovering_ = false;
}

}  // namespace mando_localization
