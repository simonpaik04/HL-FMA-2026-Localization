#include <gtest/gtest.h>

#include <limits>

#include "../src/output_gate/localization_output_gate.hpp"

namespace mando_localization {
namespace {

nav_msgs::Odometry validOdometry(const ros::Time& stamp) {
  nav_msgs::Odometry message;
  message.header.stamp = stamp;
  message.header.frame_id = "map";
  message.child_frame_id = "base_link";
  message.pose.pose.orientation.w = 1.0;
  return message;
}

TEST(LocalizationOutputGateTest, AcceptsFreshFiniteFrameContract) {
  const ros::Time now(100.0);
  EXPECT_TRUE(LocalizationOutputGate::validateOdometry(validOdometry(ros::Time(99.9)), "map",
                                                       "base_link", now, 0.25));
}

TEST(LocalizationOutputGateTest, RejectsStaleAndWrongFrames) {
  const ros::Time now(100.0);
  EXPECT_FALSE(LocalizationOutputGate::validateOdometry(validOdometry(ros::Time(99.0)), "map",
                                                        "base_link", now, 0.25));
  nav_msgs::Odometry message = validOdometry(ros::Time(99.9));
  message.header.frame_id = "odom";
  EXPECT_FALSE(LocalizationOutputGate::validateOdometry(message, "map", "base_link", now, 0.25));
}

TEST(LocalizationOutputGateTest, RejectsInvalidQuaternionAndNan) {
  const ros::Time now(100.0);
  nav_msgs::Odometry message = validOdometry(ros::Time(99.9));
  message.pose.pose.orientation.w = 0.0;
  EXPECT_FALSE(LocalizationOutputGate::validateOdometry(message, "map", "base_link", now, 0.25));
  message = validOdometry(ros::Time(99.9));
  message.pose.covariance[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(LocalizationOutputGate::validateOdometry(message, "map", "base_link", now, 0.25));

  message = validOdometry(ros::Time(99.9));
  message.twist.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(LocalizationOutputGate::validateOdometry(
      message, "map", "base_link", now, 0.25));
}

TEST(LocalizationOutputGateTest, RejectsFutureStampAndExcessPositionVariance) {
  const ros::Time now(100.0);
  EXPECT_FALSE(LocalizationOutputGate::validateOdometry(
      validOdometry(ros::Time(100.06)), "map", "base_link", now, 0.25,
      0.05, 25.0, 0.001, 1000000.0));

  nav_msgs::Odometry message = validOdometry(ros::Time(99.9));
  message.pose.covariance[0] = 26.0;
  EXPECT_FALSE(LocalizationOutputGate::validateOdometry(
      message, "map", "base_link", now, 0.25, 0.05, 25.0, 0.001,
      1000000.0));
}

}  // namespace
}  // namespace mando_localization
