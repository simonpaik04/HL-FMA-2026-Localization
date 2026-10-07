#include <gtest/gtest.h>

#include "../src/status_manager/localization_state_evaluator.hpp"

namespace mando_localization {
namespace {

LocalizationStateEvaluator evaluator() {
  StatePolicy policy;
  policy.startup_grace_sec = 3.0;
  policy.dead_reckoning_max_sec = 2.0;
  policy.dead_reckoning_max_distance_m = 10.0;
  return LocalizationStateEvaluator(policy);
}

TEST(LocalizationStateEvaluatorTest, WaitsForLocalMotionDuringStartup) {
  StateInput input;
  input.uptime_sec = 1.0;
  const StateDecision decision = evaluator().evaluate(input);
  EXPECT_EQ(LocalizationState::INITIALIZING, decision.state);
  EXPECT_FALSE(decision.valid);
}

TEST(LocalizationStateEvaluatorTest, FaultsWhenLocalMotionIsLostAfterGrace) {
  StateInput input;
  input.uptime_sec = 4.0;
  const StateDecision decision = evaluator().evaluate(input);
  EXPECT_EQ(LocalizationState::FAULT, decision.state);
  EXPECT_FALSE(decision.valid);
}

TEST(LocalizationStateEvaluatorTest, TracksWithAllEnabledAbsoluteSources) {
  StateInput input;
  input.uptime_sec = 4.0;
  input.local_motion_healthy = true;
  input.global_output_healthy = true;
  input.anchor_seen = true;
  input.absolute_enabled_count = 2U;
  input.absolute_healthy_count = 2U;
  const StateDecision decision = evaluator().evaluate(input);
  EXPECT_EQ(LocalizationState::TRACKING, decision.state);
  EXPECT_TRUE(decision.valid);
}

TEST(LocalizationStateEvaluatorTest, DegradesWithOneRemainingAbsoluteSource) {
  StateInput input;
  input.uptime_sec = 4.0;
  input.local_motion_healthy = true;
  input.global_output_healthy = true;
  input.anchor_seen = true;
  input.absolute_enabled_count = 2U;
  input.absolute_healthy_count = 1U;
  const StateDecision decision = evaluator().evaluate(input);
  EXPECT_EQ(LocalizationState::DEGRADED, decision.state);
  EXPECT_TRUE(decision.valid);
}

TEST(LocalizationStateEvaluatorTest, AllowsBoundedDeadReckoning) {
  StateInput input;
  input.uptime_sec = 4.0;
  input.local_motion_healthy = true;
  input.global_output_healthy = true;
  input.anchor_seen = true;
  input.absolute_enabled_count = 2U;
  input.seconds_since_absolute = 1.9;
  input.dead_reckoning_distance_m = 9.9;
  const StateDecision decision = evaluator().evaluate(input);
  EXPECT_EQ(LocalizationState::DEAD_RECKONING, decision.state);
  EXPECT_TRUE(decision.valid);
}

TEST(LocalizationStateEvaluatorTest, StopsOnEitherDeadReckoningLimit) {
  StateInput time_input;
  time_input.uptime_sec = 4.0;
  time_input.local_motion_healthy = true;
  time_input.global_output_healthy = true;
  time_input.anchor_seen = true;
  time_input.absolute_enabled_count = 1U;
  time_input.seconds_since_absolute = 2.01;
  EXPECT_EQ(LocalizationState::FAULT, evaluator().evaluate(time_input).state);

  StateInput distance_input = time_input;
  distance_input.seconds_since_absolute = 1.0;
  distance_input.dead_reckoning_distance_m = 10.01;
  EXPECT_EQ(LocalizationState::FAULT, evaluator().evaluate(distance_input).state);
}

TEST(LocalizationStateEvaluatorTest, RelocalizingIsFailClosed) {
  StateInput input;
  input.uptime_sec = 4.0;
  input.local_motion_healthy = true;
  input.global_output_healthy = false;
  input.anchor_seen = true;
  input.absolute_enabled_count = 1U;
  input.absolute_healthy_count = 1U;
  input.relocalizing = true;
  const StateDecision decision = evaluator().evaluate(input);
  EXPECT_EQ(LocalizationState::RELOCALIZING, decision.state);
  EXPECT_FALSE(decision.valid);
}

TEST(LocalizationStateEvaluatorTest, RejectsInvalidGlobalOutputAfterStartup) {
  StateInput input;
  input.uptime_sec = 4.0;
  input.local_motion_healthy = true;
  input.global_output_healthy = false;
  input.anchor_seen = true;
  input.absolute_enabled_count = 1U;
  input.absolute_healthy_count = 1U;
  const StateDecision decision = evaluator().evaluate(input);
  EXPECT_EQ(LocalizationState::FAULT, decision.state);
  EXPECT_EQ("global_output_invalid", decision.reason);
  EXPECT_FALSE(decision.valid);
}

}  // namespace
}  // namespace mando_localization
