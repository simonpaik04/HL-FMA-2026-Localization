#include <gtest/gtest.h>

#include "common/consecutive_recovery_gate.hpp"

namespace mando_localization {
namespace {

TEST(ConsecutiveRecoveryGateTest, RequiresThreeInitialMeasurements) {
  ConsecutiveRecoveryGate gate(3);
  EXPECT_FALSE(gate.observeHealthy());
  EXPECT_FALSE(gate.observeHealthy());
  EXPECT_TRUE(gate.observeHealthy());
  EXPECT_TRUE(gate.acceptedOnce());
  EXPECT_FALSE(gate.recovering());
}

TEST(ConsecutiveRecoveryGateTest, RequiresThreeMeasurementsAfterOutage) {
  ConsecutiveRecoveryGate gate(3);
  gate.observeHealthy();
  gate.observeHealthy();
  ASSERT_TRUE(gate.observeHealthy());

  gate.markUnhealthy();
  EXPECT_TRUE(gate.recovering());
  EXPECT_FALSE(gate.observeHealthy());
  EXPECT_FALSE(gate.observeHealthy());
  EXPECT_TRUE(gate.observeHealthy());
  EXPECT_FALSE(gate.recovering());
}

TEST(ConsecutiveRecoveryGateTest, CandidateResetDoesNotLoseRecoveryState) {
  ConsecutiveRecoveryGate gate(2);
  gate.observeHealthy();
  ASSERT_TRUE(gate.observeHealthy());
  gate.markUnhealthy();
  gate.observeHealthy();
  gate.resetCandidate();
  EXPECT_TRUE(gate.recovering());
  EXPECT_EQ(0, gate.consecutiveCount());
}

TEST(ConsecutiveRecoveryGateTest, ExplicitReanchorRestoresAcceptedState) {
  ConsecutiveRecoveryGate gate(3);
  gate.observeHealthy();
  gate.forceAccept();

  EXPECT_TRUE(gate.acceptedOnce());
  EXPECT_FALSE(gate.recovering());
  EXPECT_EQ(3, gate.consecutiveCount());
  EXPECT_TRUE(gate.observeHealthy());
}

}  // namespace
}  // namespace mando_localization
