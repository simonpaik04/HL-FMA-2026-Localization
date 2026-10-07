/*
parameter_utils.cpp
- 역할: ROS 이름과 수치 파라미터의 공통 시작 시 검증을 구현한다.
- 단위: 함수에 전달되는 물리량은 호출하는 파라미터 이름에 명시된 SI 단위를 따른다.
*/
#include "common/parameter_utils.hpp"

namespace mando_localization {

// 함수이름: requireAbsoluteRosName
// 기능: 토픽 이름이 전역 ROS 이름인지 확인한다.
// 인자: value, parameter_name
// 반환값: 없음
void requireAbsoluteRosName(const std::string& value,
                            const std::string& parameter_name) {
  if (value.empty() || value.front() != '/') {
    throw std::runtime_error(parameter_name +
                             " must be a non-empty absolute ROS name");
  }
}

// 함수이름: requireNonEmpty
// 기능: frame 또는 mode 문자열이 비어 있지 않은지 확인한다.
// 인자: value, parameter_name
// 반환값: 없음
void requireNonEmpty(const std::string& value,
                     const std::string& parameter_name) {
  if (value.empty()) {
    throw std::runtime_error(parameter_name + " must not be empty");
  }
}

// 함수이름: requireFinitePositive
// 기능: 길이·주기·timeout처럼 0보다 커야 하는 값을 검증한다.
// 인자: value, parameter_name
// 반환값: 없음
void requireFinitePositive(double value, const std::string& parameter_name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error(parameter_name + " must be finite and positive");
  }
}

// 함수이름: requireFiniteNonnegative
// 기능: 분산·허용 오차처럼 음수가 될 수 없는 값을 검증한다.
// 인자: value, parameter_name
// 반환값: 없음
void requireFiniteNonnegative(double value,
                              const std::string& parameter_name) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::runtime_error(parameter_name +
                             " must be finite and nonnegative");
  }
}

// 함수이름: requireProbabilityVariance
// 기능: 공분산 대각에 사용할 유한한 비음수 분산을 검증한다.
// 인자: value, parameter_name
// 반환값: 없음
void requireProbabilityVariance(double value,
                                const std::string& parameter_name) {
  requireFiniteNonnegative(value, parameter_name);
}

}  // namespace mando_localization
