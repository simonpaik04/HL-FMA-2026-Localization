#include <gtest/gtest.h>

#include <limits>

#include "imu_encoder_fusion/encoder_to_twist_adapter.hpp"

namespace mando_localization {
namespace {

TEST(EncoderToTwistAdapterTest, AppliesScaleAndDirectionWithoutInventingYawRate) {
  erp42_msgs::SerialFeedBack message;
  message.speed = 2.5;
  const geometry_msgs::TwistWithCovarianceStamped output =
      EncoderToTwistAdapter::convertSpeedToTwist(
          message, ros::Time(10.0), "base_link", 1.2, -1, 0.25,
          0.01, 1000000.0);
  EXPECT_EQ(ros::Time(10.0), output.header.stamp);
  EXPECT_EQ("base_link", output.header.frame_id);
  EXPECT_DOUBLE_EQ(-3.0, output.twist.twist.linear.x);
  EXPECT_DOUBLE_EQ(0.0, output.twist.twist.linear.y);
  EXPECT_DOUBLE_EQ(0.0, output.twist.twist.angular.z);
  EXPECT_DOUBLE_EQ(0.25, output.twist.covariance[0]);
  EXPECT_DOUBLE_EQ(0.01, output.twist.covariance[7]);
  EXPECT_DOUBLE_EQ(1000000.0, output.twist.covariance[35]);
}

TEST(EncoderToTwistAdapterTest, RejectsNonFiniteSpeed) {
  erp42_msgs::SerialFeedBack message;
  message.speed = 1.0;
  EXPECT_TRUE(EncoderToTwistAdapter::hasUsableSpeed(message));
  message.speed = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(EncoderToTwistAdapter::hasUsableSpeed(message));
}

TEST(EncoderToTwistAdapterTest, RequiresAliveCounterToAdvanceAndAllowsWrap) {
  EXPECT_TRUE(EncoderToTwistAdapter::aliveCounterAdvanced(7U, false, 7U));
  EXPECT_FALSE(EncoderToTwistAdapter::aliveCounterAdvanced(7U, true, 7U));
  EXPECT_TRUE(EncoderToTwistAdapter::aliveCounterAdvanced(8U, true, 7U));
  EXPECT_TRUE(EncoderToTwistAdapter::aliveCounterAdvanced(0U, true, 255U));
}

}  // namespace
}  // namespace mando_localization
