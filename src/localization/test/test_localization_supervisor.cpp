#include <gtest/gtest.h>

#include "../src/supervisor/localization_supervisor.hpp"

namespace mando_localization {
namespace {

SupervisorInput healthyInput() {
  SupervisorInput input;
  input.evaluation_received = true;
  input.evaluation_fresh = true;
  input.evaluated_state = "TRACKING";
  input.evaluated_valid = true;
  input.recovery_received = true;
  input.recovery_fresh = true;
  return input;
}

TEST(LocalizationSupervisorTest, ForwardsHealthyEvaluation) {
  const auto decision = LocalizationSupervisorArbiter().evaluate(healthyInput());
  EXPECT_EQ("TRACKING", decision.state);
  EXPECT_TRUE(decision.valid);
}

TEST(LocalizationSupervisorTest, RecoveryAlwaysClosesPublicOutput) {
  SupervisorInput input = healthyInput();
  input.recovery_active = true;
  const auto decision = LocalizationSupervisorArbiter().evaluate(input);
  EXPECT_EQ("RELOCALIZING", decision.state);
  EXPECT_FALSE(decision.valid);
}

TEST(LocalizationSupervisorTest, MissingOrStaleInputsFailClosed) {
  SupervisorInput missing = healthyInput();
  missing.recovery_received = false;
  EXPECT_EQ("FAULT", LocalizationSupervisorArbiter().evaluate(missing).state);

  SupervisorInput stale = healthyInput();
  stale.evaluation_fresh = false;
  const auto decision = LocalizationSupervisorArbiter().evaluate(stale);
  EXPECT_EQ("FAULT", decision.state);
  EXPECT_FALSE(decision.valid);
}

TEST(LocalizationSupervisorTest, RecoveryCannotOverrideStatusFault) {
  SupervisorInput input = healthyInput();
  input.evaluated_state = "FAULT";
  input.evaluated_valid = false;
  const auto decision = LocalizationSupervisorArbiter().evaluate(input);
  EXPECT_EQ("FAULT", decision.state);
  EXPECT_FALSE(decision.valid);
}

}  // namespace
}  // namespace mando_localization
