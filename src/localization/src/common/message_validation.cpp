/*
message_validation.cpp
- 역할: Localization 측정값의 공통 시간·공분산·자세 검증을 구현한다.
- 단위: 시간은 s, 각도는 rad, covariance는 각 메시지가 정의한 SI 단위의 제곱이다.
*/
#include "common/message_validation.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace mando_localization {

namespace {

void setReason(const std::string& value, std::string* reason) {
  if (reason != nullptr) {
    *reason = value;
  }
}

}  // namespace

// 함수이름: validateStamp
// 기능: ROS timestamp의 zero, 과거·미래 범위와 단조 증가를 확인한다.
// 인자: stamp, now, timeout 값과 이전 승인 timestamp
// 반환값: 모든 시간 계약을 만족하면 true
bool MessageValidation::validateStamp(
    const ros::Time& stamp, const ros::Time& now, double max_age_sec,
    double max_future_sec, bool have_last_stamp, const ros::Time& last_stamp,
    std::string* reason) {
  if (!std::isfinite(max_age_sec) || max_age_sec <= 0.0 ||
      !std::isfinite(max_future_sec) || max_future_sec < 0.0) {
    setReason("invalid_time_policy", reason);
    return false;
  }
  if (stamp.isZero() || now.isZero()) {
    setReason("zero_timestamp", reason);
    return false;
  }
  const double age_sec = (now - stamp).toSec();
  if (!std::isfinite(age_sec) || age_sec > max_age_sec ||
      age_sec < -max_future_sec) {
    setReason("timestamp_not_fresh", reason);
    return false;
  }
  // rosbag 반복 재생처럼 /clock이 뒤로 이동하면 이전 epoch의 timestamp를 버린다.
  const bool clock_moved_back =
      have_last_stamp && (now + ros::Duration(max_future_sec)) < last_stamp;
  if (have_last_stamp && !clock_moved_back && stamp <= last_stamp) {
    setReason("timestamp_not_monotonic", reason);
    return false;
  }
  setReason("", reason);
  return true;
}

// 함수이름: validateCovariance
// 기능: 정사각 covariance의 유한성, 대칭성, 분산 범위와 양의 준정부호성을 확인한다.
// 인자: covariance, 차원, unavailable marker 정책, 최대 대각 분산
// 반환값: covariance를 융합 입력으로 사용할 수 있으면 true
bool MessageValidation::validateCovariance(
    const double* covariance, std::size_t dimension,
    bool reject_unavailable_marker, double max_diagonal_variance,
    std::string* reason) {
  if (covariance == nullptr || dimension == 0 ||
      !std::isfinite(max_diagonal_variance) || max_diagonal_variance <= 0.0) {
    setReason("covariance_policy_invalid", reason);
    return false;
  }
  if (reject_unavailable_marker && covariance[0] == -1.0) {
    setReason("covariance_unavailable", reason);
    return false;
  }
  const std::size_t value_count = dimension * dimension;
  for (std::size_t index = 0; index < value_count; ++index) {
    if (!std::isfinite(covariance[index])) {
      setReason("covariance_nonfinite", reason);
      return false;
    }
  }
  for (std::size_t row = 0; row < dimension; ++row) {
    const double variance = covariance[row * dimension + row];
    if (variance < 0.0 || variance > max_diagonal_variance) {
      setReason("covariance_diagonal_out_of_range", reason);
      return false;
    }
  }

  double matrix_scale = 1.0;
  for (std::size_t index = 0; index < value_count; ++index) {
    matrix_scale = std::max(matrix_scale, std::abs(covariance[index]));
  }
  const double tolerance = 1.0e-9 * matrix_scale;
  for (std::size_t row = 0; row < dimension; ++row) {
    for (std::size_t column = row + 1; column < dimension; ++column) {
      if (std::abs(covariance[row * dimension + column] -
                   covariance[column * dimension + row]) > tolerance) {
        setReason("covariance_not_symmetric", reason);
        return false;
      }
    }
  }

  // Semidefinite 행렬의 0 pivot도 허용하는 Cholesky 형태 검사다.
  std::vector<double> lower(value_count, 0.0);
  for (std::size_t row = 0; row < dimension; ++row) {
    double diagonal = covariance[row * dimension + row];
    for (std::size_t previous = 0; previous < row; ++previous) {
      const double value = lower[row * dimension + previous];
      diagonal -= value * value;
    }
    if (diagonal < -tolerance) {
      setReason("covariance_not_positive_semidefinite", reason);
      return false;
    }
    if (diagonal <= tolerance) {
      for (std::size_t next = row + 1; next < dimension; ++next) {
        double residual = covariance[next * dimension + row];
        for (std::size_t previous = 0; previous < row; ++previous) {
          residual -= lower[next * dimension + previous] *
                      lower[row * dimension + previous];
        }
        if (std::abs(residual) > tolerance) {
          setReason("covariance_not_positive_semidefinite", reason);
          return false;
        }
      }
      continue;
    }
    lower[row * dimension + row] = std::sqrt(diagonal);
    for (std::size_t next = row + 1; next < dimension; ++next) {
      double residual = covariance[next * dimension + row];
      for (std::size_t previous = 0; previous < row; ++previous) {
        residual -= lower[next * dimension + previous] *
                    lower[row * dimension + previous];
      }
      lower[next * dimension + row] =
          residual / lower[row * dimension + row];
    }
  }
  setReason("", reason);
  return true;
}

// 함수이름: normalizeQuaternion
// 기능: 유한하고 단위 quaternion에 충분히 가까운 입력만 정규화한다.
// 인자: input, 최소 norm, 허용 정규화 오차, output
// 반환값: 안전하게 정규화했으면 true
bool MessageValidation::normalizeQuaternion(
    const geometry_msgs::Quaternion& input, double min_norm,
    double max_normalization_error, geometry_msgs::Quaternion* output,
    std::string* reason) {
  if (output == nullptr || !std::isfinite(min_norm) || min_norm <= 0.0 ||
      !std::isfinite(max_normalization_error) ||
      max_normalization_error <= 0.0) {
    setReason("quaternion_policy_invalid", reason);
    return false;
  }
  if (!finiteQuaternion(input)) {
    setReason("quaternion_nonfinite", reason);
    return false;
  }
  const double norm = std::sqrt(input.x * input.x + input.y * input.y +
                                input.z * input.z + input.w * input.w);
  if (!std::isfinite(norm) || norm < min_norm) {
    setReason("quaternion_zero", reason);
    return false;
  }
  if (std::abs(norm - 1.0) > max_normalization_error) {
    setReason("quaternion_not_unit", reason);
    return false;
  }
  output->x = input.x / norm;
  output->y = input.y / norm;
  output->z = input.z / norm;
  output->w = input.w / norm;
  setReason("", reason);
  return true;
}

// 함수이름: finiteQuaternion
// 기능: quaternion 네 성분이 모두 유한한지 확인한다.
// 인자: quaternion
// 반환값: 모두 유한하면 true
bool MessageValidation::finiteQuaternion(
    const geometry_msgs::Quaternion& quaternion) {
  return std::isfinite(quaternion.x) && std::isfinite(quaternion.y) &&
         std::isfinite(quaternion.z) && std::isfinite(quaternion.w);
}

// 함수이름: yawFromQuaternion
// 기능: 정규화된 ENU quaternion에서 Z축 yaw를 계산한다.
// 인자: quaternion
// 반환값: yaw rad
double MessageValidation::yawFromQuaternion(
    const geometry_msgs::Quaternion& quaternion) {
  const double sin_yaw =
      2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw =
      1.0 - 2.0 * (quaternion.y * quaternion.y +
                   quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

// 함수이름: shortestAngularDistance
// 기능: 두 yaw 사이의 최단 부호 각도 차이를 [-pi, pi]로 계산한다.
// 인자: from_rad, to_rad
// 반환값: to-from의 최단 각도 rad
double MessageValidation::shortestAngularDistance(double from_rad,
                                                  double to_rad) {
  return std::atan2(std::sin(to_rad - from_rad),
                    std::cos(to_rad - from_rad));
}

}  // namespace mando_localization
