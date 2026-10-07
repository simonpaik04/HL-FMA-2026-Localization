/*
parameter_utils.hpp
- 역할: 모든 Localization 노드가 동일한 방식으로 필수 ROS 파라미터와
        물리량 범위를 검증하도록 돕는다.
- 실패 경로: 필수 파라미터 누락이나 잘못된 값은 예외로 처리하여 노드가
             불완전한 설정으로 실행되지 않게 한다.
*/
#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

#include <ros/ros.h>

namespace mando_localization {

// 함수이름: requireParameter
// 기능: 필수 private 파라미터를 읽고 누락되면 시작 실패 예외를 발생시킨다.
// 인자: node, name
// 반환값: 요청한 타입의 파라미터 값
template <typename T>
T requireParameter(const ros::NodeHandle& node, const std::string& name) {
  T value;
  if (!node.getParam(name, value)) {
    throw std::runtime_error("missing required parameter: " + name);
  }
  return value;
}

void requireAbsoluteRosName(const std::string& value,
                            const std::string& parameter_name);
void requireNonEmpty(const std::string& value,
                     const std::string& parameter_name);
void requireFinitePositive(double value, const std::string& parameter_name);
void requireFiniteNonnegative(double value,
                              const std::string& parameter_name);
void requireProbabilityVariance(double value,
                                const std::string& parameter_name);

}  // namespace mando_localization
