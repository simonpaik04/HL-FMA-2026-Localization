/*
message_validation.hpp
- 역할: timestamp, covariance, quaternion의 공통 유효성 검사를 제공한다.
- frame 책임: 좌표 변환은 수행하지 않으며 각 노드가 기대 frame을 별도로 검사한다.
- 실패 경로: 함수는 false와 기계 판독용 영문 reason을 반환하고 메시지를 수정하지 않는다.
*/
#pragma once

#include <cstddef>
#include <string>

#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>

namespace mando_localization {

class MessageValidation {
 public:
  static bool validateStamp(const ros::Time& stamp, const ros::Time& now,
                            double max_age_sec, double max_future_sec,
                            bool have_last_stamp, const ros::Time& last_stamp,
                            std::string* reason);

  static bool validateCovariance(const double* covariance,
                                 std::size_t dimension,
                                 bool reject_unavailable_marker,
                                 double max_diagonal_variance,
                                 std::string* reason);

  static bool normalizeQuaternion(const geometry_msgs::Quaternion& input,
                                  double min_norm,
                                  double max_normalization_error,
                                  geometry_msgs::Quaternion* output,
                                  std::string* reason);

  static bool finiteQuaternion(const geometry_msgs::Quaternion& quaternion);
  static double yawFromQuaternion(const geometry_msgs::Quaternion& quaternion);
  static double shortestAngularDistance(double from_rad, double to_rad);
};

}  // namespace mando_localization
