#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <iterator>
#include <limits>
#include <string>

#include <nav_msgs/Odometry.h>

#include "common/message_validation.hpp"

namespace mando_localization {

// GPS 측정 시각을 양쪽 Local 표본으로 둘러쌀 때만 보간한다. 시간 epoch는
// 소유 노드가 /clock 역행을 감지하여 clear하며, 늦은 표본은 epoch로 오인하지 않는다.
class LocalOdometryHistory {
 public:
  void configure(double duration, double maximum_gap, std::size_t maximum_count,
                 const std::string& frame, const std::string& child) {
    duration_ = duration;
    maximum_gap_ = maximum_gap;
    maximum_count_ = maximum_count;
    frame_ = frame;
    child_ = child;
    clear();
  }

  void clear() { samples_.clear(); }
  std::size_t size() const { return samples_.size(); }

  bool append(const nav_msgs::Odometry& input, std::string* reason) {
    const auto& p = input.pose.pose.position;
    geometry_msgs::Quaternion orientation;
    if (input.header.stamp.isZero() || input.header.frame_id != frame_ ||
        input.child_frame_id != child_ || !std::isfinite(p.x) ||
        !std::isfinite(p.y) || !std::isfinite(p.z)) {
      *reason = "invalid_local_frame_stamp_or_position";
      return false;
    }
    if (!MessageValidation::normalizeQuaternion(input.pose.pose.orientation,
                                                1.0e-6, 0.1, &orientation, reason) ||
        !MessageValidation::validateCovariance(input.pose.covariance.data(), 6,
            false, std::numeric_limits<double>::max(), reason)) {
      return false;
    }
    if (!samples_.empty() && input.header.stamp <= samples_.back().header.stamp) {
      *reason = "local_timestamp_not_monotonic";
      return false;
    }
    samples_.push_back(input);
    samples_.back().pose.pose.orientation = orientation;
    while (samples_.size() > maximum_count_ ||
           (samples_.size() > 1 &&
            (samples_.back().header.stamp - samples_.front().header.stamp).toSec() > duration_)) {
      samples_.pop_front();
    }
    reason->clear();
    return true;
  }

  bool sample(const ros::Time& stamp, nav_msgs::Odometry* output,
              std::string* reason) const {
    if (samples_.empty()) {
      *reason = "local_history_empty";
      return false;
    }
    if (stamp < samples_.front().header.stamp) {
      *reason = "gps_before_local_history";
      return false;
    }
    if (stamp > samples_.back().header.stamp) {
      *reason = "awaiting_local_odometry";
      return false;
    }
    const auto right = std::lower_bound(samples_.begin(), samples_.end(), stamp,
        [](const nav_msgs::Odometry& sample, const ros::Time& time) {
          return sample.header.stamp < time;
        });
    if (right->header.stamp == stamp) {
      *output = *right;
      reason->clear();
      return true;
    }
    const auto left = std::prev(right);
    const double gap = (right->header.stamp - left->header.stamp).toSec();
    if (gap > maximum_gap_ + 1.0e-9) {
      *reason = "gps_local_interpolation_gap_too_large";
      return false;
    }
    const double weight = (stamp - left->header.stamp).toSec() / gap;
    *output = *left;
    output->header.stamp = stamp;
    auto& p = output->pose.pose.position;
    const auto& a = left->pose.pose.position;
    const auto& b = right->pose.pose.position;
    p.x = (1.0 - weight) * a.x + weight * b.x;
    p.y = (1.0 - weight) * a.y + weight * b.y;
    p.z = (1.0 - weight) * a.z + weight * b.z;
    // 두 PSD covariance의 convex combination은 PSD를 유지한다.
    for (std::size_t i = 0; i < output->pose.covariance.size(); ++i) {
      output->pose.covariance[i] = (1.0 - weight) * left->pose.covariance[i] +
                                  weight * right->pose.covariance[i];
    }
    const auto& qa = left->pose.pose.orientation;
    const auto& qb = right->pose.pose.orientation;
    double av[4] = {qa.x, qa.y, qa.z, qa.w};
    double bv[4] = {qb.x, qb.y, qb.z, qb.w};
    double dot = 0.0;
    for (int i = 0; i < 4; ++i) dot += av[i] * bv[i];
    if (dot < 0.0) {
      for (double& value : bv) value = -value;
      dot = -dot;
    }
    dot = std::max(0.0, std::min(1.0, dot));
    double wa = 1.0 - weight, wb = weight;
    if (dot < 0.9995) {
      const double angle = std::acos(dot);
      wa = std::sin((1.0 - weight) * angle) / std::sin(angle);
      wb = std::sin(weight * angle) / std::sin(angle);
    }
    double q[4], norm = 0.0;
    for (int i = 0; i < 4; ++i) {
      q[i] = wa * av[i] + wb * bv[i];
      norm += q[i] * q[i];
    }
    norm = std::sqrt(norm);
    auto& orientation = output->pose.pose.orientation;
    orientation.x = q[0] / norm;
    orientation.y = q[1] / norm;
    orientation.z = q[2] / norm;
    orientation.w = q[3] / norm;
    reason->clear();
    return true;
  }

 private:
  double duration_ = 2.0;
  double maximum_gap_ = 0.10;
  std::size_t maximum_count_ = 200;
  std::string frame_ = "odom";
  std::string child_ = "base_link";
  std::deque<nav_msgs::Odometry> samples_;
};

}  // namespace mando_localization
