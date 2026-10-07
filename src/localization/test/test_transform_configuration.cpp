#include <gtest/gtest.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>

#include "../src/tf/transform_configuration.hpp"

namespace mando_localization {
namespace {

StaticTransformSpec transform(const std::string& parent, const std::string& child) {
  StaticTransformSpec spec;
  spec.parent_frame = parent;
  spec.child_frame = child;
  spec.calibration_state = "unmeasured";
  return spec;
}

TEST(TransformConfigurationTest, DoesNotPublishUnmeasuredTransform) {
  StaticTransformSpec spec = transform("base_link", "imu_link");
  spec.enabled = true;
  EXPECT_FALSE(TransformConfiguration::shouldPublish(spec));
  spec.calibration_state = "measured";
  EXPECT_TRUE(TransformConfiguration::shouldPublish(spec));
}

TEST(TransformConfigurationTest, RejectsDuplicateChild) {
  EXPECT_THROW(TransformConfiguration::validate(
                   {transform("base_link", "imu_link"), transform("odom", "imu_link")}),
               std::runtime_error);
}

TEST(TransformConfigurationTest, RejectsCycle) {
  EXPECT_THROW(TransformConfiguration::validate(
                   {transform("base_link", "imu_link"), transform("imu_link", "base_link")}),
               std::runtime_error);
}

TEST(TransformConfigurationTest, RejectsDuplicateChildAcrossDynamicAndStatic) {
  DynamicTransformSpec dynamic;
  dynamic.parent_frame = "odom";
  dynamic.child_frame = "base_link";
  dynamic.owner_node = "local_ekf";
  dynamic.source = "robot_localization";
  EXPECT_THROW(TransformConfiguration::validateTree(
                   {transform("map", "base_link")}, {dynamic}),
               std::runtime_error);
}

TEST(TransformConfigurationTest, ConvertsDegreesToNormalizedQuaternion) {
  StaticTransformSpec spec = transform("base_link", "gps_link");
  spec.yaw_deg = 90.0;
  const geometry_msgs::TransformStamped message =
      TransformConfiguration::toMessage(spec, ros::Time(1.0));
  EXPECT_NEAR(std::sqrt(0.5), message.transform.rotation.z, 1.0e-9);
  EXPECT_NEAR(std::sqrt(0.5), message.transform.rotation.w, 1.0e-9);
}

TEST(TransformConfigurationTest, UpsideDownRearFacingLidarPreservesLeftAxis) {
  StaticTransformSpec spec = transform("base_link", "laser_link");
  spec.x_m = 1.05;
  spec.roll_deg = 180.0;
  spec.yaw_deg = 180.0;
  const auto message = TransformConfiguration::toMessage(spec, ros::Time(1.0));
  const auto& q = message.transform.rotation;
  const tf2::Matrix3x3 rotation(tf2::Quaternion(q.x, q.y, q.z, q.w));
  // laser +X points rearward, +Y stays left, and +Z points downward.
  const double expected[] = {-1.0, 1.0, -1.0};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      EXPECT_NEAR(row == column ? expected[row] : 0.0,
                  rotation[row][column], 1.0e-9);
    }
  }
  EXPECT_DOUBLE_EQ(1.05, message.transform.translation.x);
}

}  // namespace
}  // namespace mando_localization
