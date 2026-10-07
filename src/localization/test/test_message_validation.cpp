#include <gtest/gtest.h>

#include "common/message_validation.hpp"

namespace mando_localization {
namespace {

TEST(MessageValidationTest, AcceptsNewEpochAfterClockMovesBack) {
  std::string reason;
  EXPECT_TRUE(MessageValidation::validateStamp(
      ros::Time(10.0), ros::Time(10.0), 0.2, 0.05, true,
      ros::Time(100.0), &reason));
}

TEST(MessageValidationTest, RejectsNonMonotonicStampWithinSameEpoch) {
  std::string reason;
  EXPECT_FALSE(MessageValidation::validateStamp(
      ros::Time(10.0), ros::Time(10.0), 0.2, 0.05, true,
      ros::Time(10.0), &reason));
  EXPECT_EQ("timestamp_not_monotonic", reason);
}

TEST(MessageValidationTest, HalfSecondFutureToleranceKeepsBoundary) {
  std::string reason;
  for (double lead : {0.46, 0.5}) {
    EXPECT_TRUE(MessageValidation::validateStamp(
        ros::Time(10.0 + lead), ros::Time(10.0), 1.0, 0.5,
        false, ros::Time(), &reason));
  }
  EXPECT_FALSE(MessageValidation::validateStamp(
      ros::Time(10.501), ros::Time(10.0), 1.0, 0.5,
      false, ros::Time(), &reason));
  EXPECT_EQ("timestamp_not_fresh", reason);
}

TEST(MessageValidationTest, RejectsAsymmetricAndIndefiniteCovariance) {
  std::string reason;
  double asymmetric[4] = {1.0, 0.5, 0.0, 1.0};
  EXPECT_FALSE(MessageValidation::validateCovariance(
      asymmetric, 2, false, 10.0, &reason));
  EXPECT_EQ("covariance_not_symmetric", reason);

  double indefinite[4] = {1.0, 2.0, 2.0, 1.0};
  EXPECT_FALSE(MessageValidation::validateCovariance(
      indefinite, 2, false, 10.0, &reason));
  EXPECT_EQ("covariance_not_positive_semidefinite", reason);
}

TEST(MessageValidationTest, AcceptsPositiveSemidefiniteCovariance) {
  std::string reason;
  double covariance[4] = {4.0, 2.0, 2.0, 1.0};
  EXPECT_TRUE(MessageValidation::validateCovariance(
      covariance, 2, false, 10.0, &reason));
}

}  // namespace
}  // namespace mando_localization
