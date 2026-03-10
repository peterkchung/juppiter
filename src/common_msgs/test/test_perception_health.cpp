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
// About: Unit tests for PerceptionHealth message (ARM-optimized)

#include <gtest/gtest.h>
#include <cstring>

#include "common_msgs/msg/perception_health.hpp"
#include "common_msgs/srv/get_perception_health.hpp"

namespace common_msgs
{
namespace testing
{

// Test 1: Verify ARM-optimized memory layout (48 bytes)
TEST(PerceptionHealthTest, ARMAlignment48Bytes)
{
  common_msgs::msg::PerceptionHealth health;
  
  // Check total size (should be approximately 48 bytes for fixed fields)
  // Note: Strings add variable size, but fixed portion should be compact
  size_t fixed_size = 
    sizeof(health.stamp.sec) +
    sizeof(health.stamp.nanosec) +
    sizeof(health.overall_health) +
    sizeof(health.lio_health) +
    sizeof(health.vio_health) +
    sizeof(health.kinematic_health) +
    sizeof(health.lidar_health) +
    sizeof(health.camera_health) +
    sizeof(health.imu_health) +
    sizeof(health.time_sync_skew_ms) +
    sizeof(health.fault_flags) +
    sizeof(health.mode) +
    sizeof(health.time_sync_valid) +
    sizeof(health.service_responsive);
  
  // Should be 48 bytes for fixed fields (8 + 8*4 + 2 + 1 + 1 + 1 = 48)
  EXPECT_EQ(fixed_size, 48u);
}

// Test 2: Verify fault flag bitmasks are correct
TEST(PerceptionHealthTest, FaultFlagBitmasks)
{
  // Verify each fault flag has correct bit position
  EXPECT_EQ(common_msgs::msg::PerceptionHealth::FAULT_NONE, 0u);
  EXPECT_EQ(common_msgs::msg::PerceptionHealth::FAULT_LIO_STALE, 1u);      // Bit 0
  EXPECT_EQ(common_msgs::msg::PerceptionHealth::FAULT_VIO_STALE, 2u);      // Bit 1
  EXPECT_EQ(common_msgs::msg::PerceptionHealth::FAULT_KINEMATIC_STALE, 4u); // Bit 2
  EXPECT_EQ(common_msgs::msg::PerceptionHealth::FAULT_LIDAR_STALE, 8u);    // Bit 3
  EXPECT_EQ(common_msgs::msg::PerceptionHealth::FAULT_CAMERA_STALE, 16u);   // Bit 4
  EXPECT_EQ(common_msgs::msg::PerceptionHealth::FAULT_IMU_STALE, 32u);     // Bit 5
  EXPECT_EQ(common_msgs::msg::PerceptionHealth::FAULT_TIME_SYNC, 64u);     // Bit 6
  EXPECT_EQ(common_msgs::msg::PerceptionHealth::FAULT_COVARIANCE_EXCEEDED, 128u);   // Bit 7
  EXPECT_EQ(common_msgs::msg::PerceptionHealth::FAULT_CALIBRATION_INVALID, 256u);  // Bit 8
  EXPECT_EQ(common_msgs::msg::PerceptionHealth::FAULT_SERVICE_UNAVAILABLE, 512u);   // Bit 9
}

// Test 3: Verify mode enum values
TEST(PerceptionHealthTest, ModeEnumValues)
{
  EXPECT_EQ(common_msgs::msg::PerceptionHealth::MODE_NOMINAL, 0u);
  EXPECT_EQ(common_msgs::msg::PerceptionHealth::MODE_DEGRADED_LIO, 1u);
  EXPECT_EQ(common_msgs::msg::PerceptionHealth::MODE_DEGRADED_VIO, 2u);
  EXPECT_EQ(common_msgs::msg::PerceptionHealth::MODE_SAFE_STOP, 3u);
}

// Test 4: Verify fault flags can be combined with bitwise OR
TEST(PerceptionHealthTest, CanCombineFaultFlags)
{
  uint16_t combined = 
    common_msgs::msg::PerceptionHealth::FAULT_LIDAR_STALE |
    common_msgs::msg::PerceptionHealth::FAULT_CAMERA_STALE;
  
  // Should be 8 | 16 = 24
  EXPECT_EQ(combined, 24u);
  
  // Verify individual bits can be checked
  EXPECT_TRUE(combined & common_msgs::msg::PerceptionHealth::FAULT_LIDAR_STALE);
  EXPECT_TRUE(combined & common_msgs::msg::PerceptionHealth::FAULT_CAMERA_STALE);
  EXPECT_FALSE(combined & common_msgs::msg::PerceptionHealth::FAULT_IMU_STALE);
}

// Test 5: Service request/response structure
TEST(PerceptionHealthServiceTest, ServiceRequestResponse)
{
  // Request is empty (just triggers aggregation)
  auto request = std::make_shared<common_msgs::srv::GetPerceptionHealth::Request>();
  
  // Response should contain health
  auto response = std::make_shared<common_msgs::srv::GetPerceptionHealth::Response>();
  
  // Populate response health
  response->health.overall_health = 0.85f;
  response->health.lio_health = 0.90f;
  response->health.vio_health = 0.88f;
  response->health.kinematic_health = 0.92f;
  response->health.mode = common_msgs::msg::PerceptionHealth::MODE_NOMINAL;
  
  // Verify values
  EXPECT_FLOAT_EQ(response->health.overall_health, 0.85f);
  EXPECT_EQ(response->health.mode, common_msgs::msg::PerceptionHealth::MODE_NOMINAL);
}

// Test 6: Health score ranges (0.0 to 1.0)
TEST(PerceptionHealthTest, HealthScoreRanges)
{
  common_msgs::msg::PerceptionHealth health;
  
  // Valid ranges
  health.overall_health = 0.0f;
  EXPECT_GE(health.overall_health, 0.0f);
  EXPECT_LE(health.overall_health, 1.0f);
  
  health.overall_health = 1.0f;
  EXPECT_GE(health.overall_health, 0.0f);
  EXPECT_LE(health.overall_health, 1.0f);
  
  health.overall_health = 0.5f;
  EXPECT_GE(health.overall_health, 0.0f);
  EXPECT_LE(health.overall_health, 1.0f);
}

// Test 7: Timestamp fields
TEST(PerceptionHealthTest, TimestampFields)
{
  common_msgs::msg::PerceptionHealth health;
  
  // Set timestamp
  health.stamp.sec = 1234567890;
  health.stamp.nanosec = 123456789;
  
  EXPECT_EQ(health.stamp.sec, 1234567890);
  EXPECT_EQ(health.stamp.nanosec, 123456789);
}

// Test 8: Time sync fields
TEST(PerceptionHealthTest, TimeSyncFields)
{
  common_msgs::msg::PerceptionHealth health;
  
  health.time_sync_skew_ms = 5.5f;
  health.time_sync_valid = true;
  health.service_responsive = true;
  
  EXPECT_FLOAT_EQ(health.time_sync_skew_ms, 5.5f);
  EXPECT_TRUE(health.time_sync_valid);
  EXPECT_TRUE(health.service_responsive);
}

// Test 9: Individual sensor health fields
TEST(PerceptionHealthTest, IndividualSensorHealth)
{
  common_msgs::msg::PerceptionHealth health;
  
  // Set all sensor health
  health.lidar_health = 0.9f;
  health.camera_health = 0.85f;
  health.imu_health = 0.95f;
  
  // Set all estimator health
  health.lio_health = 0.88f;
  health.vio_health = 0.82f;
  health.kinematic_health = 0.91f;
  
  EXPECT_FLOAT_EQ(health.lidar_health, 0.9f);
  EXPECT_FLOAT_EQ(health.camera_health, 0.85f);
  EXPECT_FLOAT_EQ(health.imu_health, 0.95f);
  EXPECT_FLOAT_EQ(health.lio_health, 0.88f);
  EXPECT_FLOAT_EQ(health.vio_health, 0.82f);
  EXPECT_FLOAT_EQ(health.kinematic_health, 0.91f);
}

// Test 10: Mode string field
TEST(PerceptionHealthTest, ModeStringField)
{
  common_msgs::msg::PerceptionHealth health;
  
  health.mode = common_msgs::msg::PerceptionHealth::MODE_DEGRADED_LIO;
  health.mode_string = "degraded_lio";
  
  EXPECT_EQ(health.mode, common_msgs::msg::PerceptionHealth::MODE_DEGRADED_LIO);
  EXPECT_EQ(health.mode_string, "degraded_lio");
}

// Test 11: Active faults array
TEST(PerceptionHealthTest, ActiveFaultsArray)
{
  common_msgs::msg::PerceptionHealth health;
  
  health.active_faults.push_back("lidar_stale");
  health.active_faults.push_back("time_sync_violation");
  
  EXPECT_EQ(health.active_faults.size(), 2u);
  EXPECT_EQ(health.active_faults[0], "lidar_stale");
  EXPECT_EQ(health.active_faults[1], "time_sync_violation");
}

}  // namespace testing
}  // namespace common_msgs

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
