// Copyright 2026 Arconic Labs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// About: Unit tests for ModeManager - single authority for mode determination

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "fusion_core/mode_manager.hpp"
#include "common_msgs/msg/perception_health.hpp"

namespace juppiter
{
namespace fusion
{
namespace testing
{

class ModeManagerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");
    mode_manager_ = std::make_unique<ModeManager>();
    
    // Initialize with default config
    mode_manager_->initialize(node_, "config/fusion/health_thresholds.yaml");
  }

  void TearDown() override
  {
    mode_manager_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<ModeManager> mode_manager_;
};

// Test 1: DeterminesNominalAt0_75Health
TEST_F(ModeManagerTest, DeterminesNominalAt0_75Health)
{
  common_msgs::msg::PerceptionHealth health;
  health.overall_health = 0.80f;
  health.lio_health = 0.85f;
  health.vio_health = 0.82f;
  health.kinematic_health = 0.90f;
  
  auto mode = mode_manager_->determineMode(health);
  
  EXPECT_EQ(mode, FusionMode::NOMINAL);
}

// Test 2: DeterminesSafeStopBelow0_55
TEST_F(ModeManagerTest, DeterminesSafeStopBelow0_55)
{
  common_msgs::msg::PerceptionHealth health;
  health.overall_health = 0.50f;
  health.lio_health = 0.60f;
  health.vio_health = 0.50f;
  health.kinematic_health = 0.70f;
  
  auto mode = mode_manager_->determineMode(health);
  
  EXPECT_EQ(mode, FusionMode::SAFE_STOP);
}

// Test 3: DeterminesDegradedLIOAt0_60
TEST_F(ModeManagerTest, DeterminesDegradedLIOAt0_60)
{
  common_msgs::msg::PerceptionHealth health;
  health.overall_health = 0.65f;  // Below nominal, above safe_stop
  health.lio_health = 0.60f;      // LIO is degraded
  health.vio_health = 0.85f;      // VIO is healthy
  health.kinematic_health = 0.80f;
  
  auto mode = mode_manager_->determineMode(health);
  
  EXPECT_EQ(mode, FusionMode::DEGRADED_LIO);
}

// Test 4: DeterminesDegradedVIOAt0_60
TEST_F(ModeManagerTest, DeterminesDegradedVIOAt0_60)
{
  common_msgs::msg::PerceptionHealth health;
  health.overall_health = 0.65f;
  health.lio_health = 0.85f;      // LIO healthy
  health.vio_health = 0.60f;      // VIO degraded
  health.kinematic_health = 0.80f;
  
  auto mode = mode_manager_->determineMode(health);
  
  EXPECT_EQ(mode, FusionMode::DEGRADED_VIO);
}

// Test 5: Requires3CyclesToSwitch
TEST_F(ModeManagerTest, Requires3CyclesToSwitch)
{
  // Start at nominal
  common_msgs::msg::PerceptionHealth health1;
  health1.overall_health = 0.80f;
  mode_manager_->determineMode(health1);
  
  // Try to switch to degraded once - should not switch yet
  common_msgs::msg::PerceptionHealth health2;
  health2.overall_health = 0.60f;
  health2.lio_health = 0.55f;
  
  auto mode = mode_manager_->determineMode(health2);
  EXPECT_FALSE(mode_manager_->shouldSwitchMode(mode));  // First attempt
  
  mode = mode_manager_->determineMode(health2);
  EXPECT_FALSE(mode_manager_->shouldSwitchMode(mode));  // Second attempt
  
  mode = mode_manager_->determineMode(health2);
  EXPECT_TRUE(mode_manager_->shouldSwitchMode(mode));   // Third attempt - switch
}

// Test 6: PreventsModeFlutter
TEST_F(ModeManagerTest, PreventsModeFlutter)
{
  // Simulate alternating health values
  common_msgs::msg::PerceptionHealth degraded;
  degraded.overall_health = 0.60f;
  degraded.lio_health = 0.55f;
  degraded.vio_health = 0.75f;
  
  common_msgs::msg::PerceptionHealth nominal;
  nominal.overall_health = 0.80f;
  nominal.lio_health = 0.85f;
  nominal.vio_health = 0.82f;
  
  // Should not switch immediately due to persistence
  int switches = 0;
  for (int i = 0; i < 10; ++i) {
    auto mode = mode_manager_->determineMode((i % 2 == 0) ? degraded : nominal);
    if (mode_manager_->shouldSwitchMode(mode)) {
      switches++;
      mode_manager_->executeModeSwitch(mode, "test");
    }
  }
  
  // Should have far fewer switches than evaluations
  EXPECT_LT(switches, 5);
}

// Test 7: RecordsTransitionHistory
TEST_F(ModeManagerTest, RecordsTransitionHistory)
{
  // Force a mode transition
  mode_manager_->forceMode(FusionMode::SAFE_STOP, "test");
  mode_manager_->forceMode(FusionMode::NOMINAL, "recovery");
  
  auto history = mode_manager_->getTransitionHistory();
  
  EXPECT_GE(history.size(), 2);
  
  if (history.size() >= 2) {
    EXPECT_EQ(history[0].to, FusionMode::SAFE_STOP);
    EXPECT_EQ(history[1].to, FusionMode::NOMINAL);
  }
}

// Test 8: NotifiesCallbacksOnChange
TEST_F(ModeManagerTest, NotifiesCallbacksOnChange)
{
  bool callback_called = false;
  FusionMode received_mode = FusionMode::NOMINAL;
  
  mode_manager_->registerModeChangeCallback(
    [&](const ModeTransition & transition) {
      callback_called = true;
      received_mode = transition.to;
    });
  
  // Force a mode change
  mode_manager_->forceMode(FusionMode::SAFE_STOP, "test callback");
  
  EXPECT_TRUE(callback_called);
  EXPECT_EQ(received_mode, FusionMode::SAFE_STOP);
}

// Test 9: HandlesManualOverride
TEST_F(ModeManagerTest, HandlesManualOverride)
{
  // Force to degraded_lio
  mode_manager_->forceMode(FusionMode::DEGRADED_LIO, "manual test");
  
  EXPECT_EQ(mode_manager_->getCurrentMode(), FusionMode::DEGRADED_LIO);
  
  // Force to safe_stop
  mode_manager_->forceMode(FusionMode::SAFE_STOP, "emergency");
  
  EXPECT_EQ(mode_manager_->getCurrentMode(), FusionMode::SAFE_STOP);
}

// Test 10: ModeToStringConversion
TEST_F(ModeManagerTest, ModeToStringConversion)
{
  EXPECT_EQ(mode_manager_->modeToString(FusionMode::NOMINAL), "nominal");
  EXPECT_EQ(mode_manager_->modeToString(FusionMode::DEGRADED_LIO), "degraded_lio");
  EXPECT_EQ(mode_manager_->modeToString(FusionMode::DEGRADED_VIO), "degraded_vio");
  EXPECT_EQ(mode_manager_->modeToString(FusionMode::DEGRADED_KINEMATIC), "degraded_kinematic");
  EXPECT_EQ(mode_manager_->modeToString(FusionMode::SAFE_STOP), "safe_stop");
}

// Test 11: StringToModeConversion
TEST_F(ModeManagerTest, StringToModeConversion)
{
  EXPECT_EQ(mode_manager_->stringToMode("nominal"), FusionMode::NOMINAL);
  EXPECT_EQ(mode_manager_->stringToMode("degraded_lio"), FusionMode::DEGRADED_LIO);
  EXPECT_EQ(mode_manager_->stringToMode("degraded_vio"), FusionMode::DEGRADED_VIO);
  EXPECT_EQ(mode_manager_->stringToMode("degraded_kinematic"), FusionMode::DEGRADED_KINEMATIC);
  EXPECT_EQ(mode_manager_->stringToMode("safe_stop"), FusionMode::SAFE_STOP);
  EXPECT_EQ(mode_manager_->stringToMode("unknown"), FusionMode::NOMINAL);  // Default
}

// Test 12: PriorityInDegradedModes
TEST_F(ModeManagerTest, PriorityInDegradedModes)
{
  // Both LIO and VIO degraded - should pick based on which is worse
  common_msgs::msg::PerceptionHealth health1;
  health1.overall_health = 0.65f;
  health1.lio_health = 0.60f;  // Worse
  health1.vio_health = 0.70f;  // Better
  
  auto mode1 = mode_manager_->determineMode(health1);
  EXPECT_EQ(mode1, FusionMode::DEGRADED_LIO);
  
  // Reverse - VIO worse
  common_msgs::msg::PerceptionHealth health2;
  health2.overall_health = 0.65f;
  health2.lio_health = 0.70f;  // Better
  health2.vio_health = 0.60f;  // Worse
  
  auto mode2 = mode_manager_->determineMode(health2);
  EXPECT_EQ(mode2, FusionMode::DEGRADED_VIO);
}

// Test 13: TracksTransitionProgress
TEST_F(ModeManagerTest, TracksTransitionProgress)
{
  // Force a mode change
  mode_manager_->forceMode(FusionMode::SAFE_STOP, "test progress");
  
  // Should be in transition initially
  EXPECT_TRUE(mode_manager_->isInTransition());
  EXPECT_GE(mode_manager_->getTransitionProgress(), 0.0f);
  EXPECT_LE(mode_manager_->getTransitionProgress(), 1.0f);
  
  // Update transition
  mode_manager_->updateTransition(0.5f);  // 0.5s
  float progress1 = mode_manager_->getTransitionProgress();
  
  mode_manager_->updateTransition(1.5f);  // Additional 1.5s (total 2.0s)
  float progress2 = mode_manager_->getTransitionProgress();
  
  // Progress should have increased
  EXPECT_GT(progress2, progress1);
  
  // After 2.0s, should be at 100%
  EXPECT_FLOAT_EQ(progress2, 1.0f);
  EXPECT_FALSE(mode_manager_->isInTransition());
}

// Test 14: ThreadSafeOperations
TEST_F(ModeManagerTest, ThreadSafeOperations)
{
  // Simulate concurrent mode evaluations
  std::vector<std::thread> threads;
  std::vector<FusionMode> results;
  std::mutex results_mutex;
  
  common_msgs::msg::PerceptionHealth health;
  health.overall_health = 0.80f;
  health.lio_health = 0.85f;
  health.vio_health = 0.82f;
  health.kinematic_health = 0.90f;
  
  for (int i = 0; i < 10; ++i) {
    threads.emplace_back([&]() {
      auto mode = mode_manager_->determineMode(health);
      std::lock_guard<std::mutex> lock(results_mutex);
      results.push_back(mode);
    });
  }
  
  for (auto & t : threads) {
    t.join();
  }
  
  // All should return the same mode
  EXPECT_EQ(results.size(), 10);
  for (const auto & mode : results) {
    EXPECT_EQ(mode, FusionMode::NOMINAL);
  }
}

}  // namespace testing
}  // namespace fusion
}  // namespace juppiter

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
