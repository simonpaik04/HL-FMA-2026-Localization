#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "odometry_gps_fusion/local_odometry_history.hpp"

namespace {
ros::Time stamp(double value) { ros::Time t; t.fromSec(value); return t; }
nav_msgs::Odometry odom(double time, double x = 0.0, double yaw = 0.0) {
  nav_msgs::Odometry value;
  value.header.stamp = stamp(time);
  value.header.frame_id = "odom";
  value.child_frame_id = "base_link";
  value.pose.pose.position.x = x;
  value.pose.pose.orientation.z = std::sin(yaw / 2.0);
  value.pose.pose.orientation.w = std::cos(yaw / 2.0);
  for (int i : {0, 7, 14, 21, 28, 35}) value.pose.covariance[i] = 1.0;
  return value;
}
}

TEST(LocalOdometryHistory, DelayedMeasurementUsesItsOwnBracket) {
  mando_localization::LocalOdometryHistory history;
  std::string reason;
  for (int i = 0; i <= 10; ++i) ASSERT_TRUE(history.append(odom(10.0+i*.05, i*.1), &reason));
  nav_msgs::Odometry result;
  ASSERT_TRUE(history.sample(stamp(10.125), &result, &reason));
  EXPECT_NEAR(.25, result.pose.pose.position.x, 1e-10);
  EXPECT_EQ(stamp(10.125), result.header.stamp);
}

TEST(LocalOdometryHistory, QuaternionCrossesPiByShortestPath) {
  mando_localization::LocalOdometryHistory history;
  std::string reason;
  const double pi = std::acos(-1.0);
  ASSERT_TRUE(history.append(odom(10., 0., 179.*pi/180.), &reason));
  ASSERT_TRUE(history.append(odom(10.1, 0., -179.*pi/180.), &reason));
  nav_msgs::Odometry result;
  ASSERT_TRUE(history.sample(stamp(10.05), &result, &reason));
  EXPECT_NEAR(pi, std::abs(mando_localization::MessageValidation::yawFromQuaternion(
      result.pose.pose.orientation)), 1e-10);
}

TEST(LocalOdometryHistory, ConvexCovarianceStaysPositiveSemidefinite) {
  mando_localization::LocalOdometryHistory history;
  std::string reason;
  auto left = odom(10.);
  auto right = odom(10.1);
  left.pose.covariance[0] = 4.; left.pose.covariance[7] = 1.;
  left.pose.covariance[1] = left.pose.covariance[6] = 1.5;
  right.pose.covariance[0] = 1.; right.pose.covariance[7] = 4.;
  right.pose.covariance[1] = right.pose.covariance[6] = -1.5;
  ASSERT_TRUE(history.append(left, &reason)); ASSERT_TRUE(history.append(right, &reason));
  nav_msgs::Odometry result;
  ASSERT_TRUE(history.sample(stamp(10.05), &result, &reason));
  EXPECT_NEAR(2.5, result.pose.covariance[0], 1e-10);
  EXPECT_NEAR(0., result.pose.covariance[1], 1e-10);
  EXPECT_TRUE(mando_localization::MessageValidation::validateCovariance(
      result.pose.covariance.data(), 6, false, 100., &reason));
}

TEST(LocalOdometryHistory, DoesNotExtrapolateOrCrossLargeGap) {
  mando_localization::LocalOdometryHistory history;
  std::string reason;
  ASSERT_TRUE(history.append(odom(10.), &reason));
  ASSERT_TRUE(history.append(odom(10.2), &reason));
  nav_msgs::Odometry result;
  EXPECT_FALSE(history.sample(stamp(9.9), &result, &reason));
  EXPECT_EQ("gps_before_local_history", reason);
  EXPECT_FALSE(history.sample(stamp(10.3), &result, &reason));
  EXPECT_EQ("awaiting_local_odometry", reason);
  EXPECT_FALSE(history.sample(stamp(10.1), &result, &reason));
  EXPECT_EQ("gps_local_interpolation_gap_too_large", reason);
  EXPECT_TRUE(history.sample(stamp(10.2), &result, &reason));
}

TEST(LocalOdometryHistory, InvalidFrameNanAndNonPsdCannotPoisonHistory) {
  mando_localization::LocalOdometryHistory history;
  std::string reason;
  auto input = odom(10.);
  input.header.frame_id = "map";
  EXPECT_FALSE(history.append(input, &reason));
  input = odom(10.); input.pose.pose.position.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(history.append(input, &reason));
  input = odom(10.); input.pose.covariance[0] = -1.;
  EXPECT_FALSE(history.append(input, &reason));
  input = odom(10.); input.pose.pose.orientation.w = 0.;
  EXPECT_FALSE(history.append(input, &reason));
  EXPECT_EQ(0U, history.size());
}

TEST(LocalOdometryHistory, DuplicatesAndOldSamplesDoNotResetHistory) {
  mando_localization::LocalOdometryHistory history;
  std::string reason;
  ASSERT_TRUE(history.append(odom(10., 1.), &reason));
  ASSERT_TRUE(history.append(odom(10.1, 2.), &reason));
  EXPECT_FALSE(history.append(odom(10.1, 99.), &reason));
  EXPECT_FALSE(history.append(odom(9., 99.), &reason));
  nav_msgs::Odometry result;
  EXPECT_TRUE(history.sample(stamp(10.05), &result, &reason));
  EXPECT_NEAR(1.5, result.pose.pose.position.x, 1e-10);
}

TEST(LocalOdometryHistory, ExplicitEpochClearAllowsEarlierTimeline) {
  mando_localization::LocalOdometryHistory history;
  std::string reason;
  ASSERT_TRUE(history.append(odom(100.), &reason));
  history.clear();
  ASSERT_TRUE(history.append(odom(10.), &reason));
  nav_msgs::Odometry result;
  EXPECT_FALSE(history.sample(stamp(100.), &result, &reason));
  EXPECT_TRUE(history.sample(stamp(10.), &result, &reason));
}

TEST(LocalOdometryHistory, MemoryIsBoundedByAgeAndSampleCount) {
  mando_localization::LocalOdometryHistory history;
  history.configure(.2, .10, 3, "odom", "base_link");
  std::string reason;
  for (int i = 0; i < 10; ++i) ASSERT_TRUE(history.append(odom(10.+i*.05), &reason));
  EXPECT_LE(history.size(), 3U);
  nav_msgs::Odometry result;
  EXPECT_FALSE(history.sample(stamp(10.), &result, &reason));
}
