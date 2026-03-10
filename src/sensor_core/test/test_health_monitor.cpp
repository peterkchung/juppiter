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
// About: Unit tests for HealthMonitor with on-demand health aggregation

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "sensor_core/health_monitor.hpp"
#include "test/mock_sensor_driver.hpp"

namespace sensor_core
{
namespace testing
{

class HealthMonitorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");
    health_monitor_ = std::make_unique<HealthMonitor>(node_);
  }

  void TearDown() override
  {
    health_monitor_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<HealthMonitor> health_monitor_;
};

// Test 1: AggregateHealthReturnsValidMessage
TEST_F(HealthMonitorTest, AggregateHealthReturnsValidMessage)
{
  auto mock_driver = std::make_shared<MockSensorDriver>("lidar");
  sensor_interfaces::HealthStatus status;
  status.score = 0.95f;
  status.flags = sensor_interfaces::FaultFlags::NONE;
  mock_driver->setHealthStatus(status);

  health_monitor_->register_driver("lidar", mock_driver);
  
  auto health = health_monitor_->aggregate_health();
  
  EXPECT_GT(health.stamp.sec, 0);
  EXPECT_GE(health.lidar_health, 0.0f);
  EXPECT_LE(health.lidar_health, 1.0f);
  EXPECT_EQ(health.fault_flags, common_msgs::msg::PerceptionHealth::FAULT_NONE);
}

// Test 2: ComputesSensorHealthScores
TEST_F(HealthMonitorTest, ComputesSensorHealthScores)
{
  auto lidar_driver = std::make_shared<MockSensorDriver>("lidar");
  auto camera_driver = std::make_shared<MockSensorDriver>("camera");
  auto imu_driver = std::make_shared<MockSensorDriver>("imu");
  
  sensor_interfaces::HealthStatus lidar_status;
  lidar_status.score = 0.9f;
  lidar_driver->setHealthStatus(lidar_status);
  
  sensor_interfaces::HealthStatus camera_status;
  camera_status.score = 0.8f;
  camera_driver->setHealthStatus(camera_status);
  
  sensor_interfaces::HealthStatus imu_status;
  imu_status.score = 0.95f;
  imu_driver->setHealthStatus(imu_status);
  
  health_monitor_->register_driver("lidar", lidar_driver);
  health_monitor_->register_driver("camera", camera_driver);
  health_monitor_->register_driver("imu", imu_driver);
  
  auto health = health_monitor_->aggregate_health();
  
  EXPECT_FLOAT_EQ(health.lidar_health, 0.9f);
  EXPECT_FLOAT_EQ(health.camera_health, 0.8f);
  EXPECT_FLOAT_EQ(health.imu_health, 0.95f);
}

// Test 3: DetectsStaleSensors
TEST_F(HealthMonitorTest, DetectsStaleSensors)
{
  auto mock_driver = std::make_shared<MockSensorDriver>("lidar");
  
  sensor_interfaces::HealthStatus status;
  status.score = 0.5f;
  status.flags = sensor_interfaces::FaultFlags::STALE;
  mock_driver->setHealthStatus(status);
  
  health_monitor_->register_driver("lidar", mock_driver);
  
  auto health = health_monitor_->aggregate_health();
  
  EXPECT_TRUE(health.fault_flags & common_msgs::msg::PerceptionHealth::FAULT_LIDAR_STALE);
}

// Test 4: SetsTimeSyncStatus
TEST_F(HealthMonitorTest, SetsTimeSyncStatus)
{
  sensor_interfaces::TimeSyncStatus sync_status;
  sync_status.skew_max_ms = 15.0f;
  sync_status.is_valid = false;
  
  health_monitor_->set_sync_status(sync_status);
  
  // Need to register at least one driver
  auto mock_driver = std::make_shared<MockSensorDriver>("lidar");
  sensor_interfaces::HealthStatus status;
  status.score = 1.0f;
  mock_driver->setHealthStatus(status);
  health_monitor_->register_driver("lidar", mock_driver);
  
  auto health = health_monitor_->aggregate_health();
  
  EXPECT_FLOAT_EQ(health.time_sync_skew_ms, 15.0f);
  EXPECT_FALSE(health.time_sync_valid);
  EXPECT_TRUE(health.fault_flags & common_msgs::msg::PerceptionHealth::FAULT_TIME_SYNC);
}

// Test 5: ComputesFaultFlagsBitmask
TEST_F(HealthMonitorTest, ComputesFaultFlagsBitmask)
{
  auto lidar_driver = std::make_shared<MockSensorDriver>("lidar");
  auto camera_driver = std::make_shared<MockSensorDriver>("camera");
  
  sensor_interfaces::HealthStatus lidar_status;
  lidar_status.score = 0.5f;
  lidar_status.flags = sensor_interfaces::FaultFlags::STALE;
  lidar_driver->setHealthStatus(lidar_status);
  
  sensor_interfaces::HealthStatus camera_status;
  camera_status.score = 0.7f;
  camera_status.flags = sensor_interfaces::FaultFlags::STALE;
  camera_driver->setHealthStatus(camera_status);
  
  health_monitor_->register_driver("lidar", lidar_driver);
  health_monitor_->register_driver("camera", camera_driver);
  
  auto health = health_monitor_->aggregate_health();
  
  uint16_t expected_flags = 
    common_msgs::msg::PerceptionHealth::FAULT_LIDAR_STALE |
    common_msgs::msg::PerceptionHealth::FAULT_CAMERA_STALE;
  
  EXPECT_EQ(health.fault_flags, expected_flags);
}

// Test 6: HandlesEmptyDriverList
TEST_F(HealthMonitorTest, HandlesEmptyDriverList)
{
  auto health = health_monitor_->aggregate_health();
  
  // Should return valid message with zero health
  EXPECT_GE(health.lidar_health, 0.0f);
  EXPECT_GE(health.camera_health, 0.0f);
  EXPECT_GE(health.imu_health, 0.0f);
  EXPECT_EQ(health.fault_flags, common_msgs::msg::PerceptionHealth::FAULT_NONE);
}

// Test 7: CachesAggregatedHealth
TEST_F(HealthMonitorTest, CachesAggregatedHealth)
{
  auto mock_driver = std::make_shared<MockSensorDriver>("lidar");
  sensor_interfaces::HealthStatus status;
  status.score = 0.85f;
  mock_driver->setHealthStatus(status);
  
  health_monitor_->register_driver("lidar", mock_driver);
  
  auto health1 = health_monitor_->aggregate_health();
  auto cached = health_monitor_->get_cached_health();
  
  EXPECT_FLOAT_EQ(health1.lidar_health, cached.lidar_health);
  EXPECT_FLOAT_EQ(health1.camera_health, cached.camera_health);
}

// Test 8: ThreadSafeConcurrentAccess
TEST_F(HealthMonitorTest, ThreadSafeConcurrentAccess)
{
  auto mock_driver = std::make_shared<MockSensorDriver>("lidar");
  sensor_interfaces::HealthStatus status;
  status.score = 0.9f;
  mock_driver->setHealthStatus(status);
  
  health_monitor_->register_driver("lidar", mock_driver);
  
  // Simulate concurrent aggregation calls
  std::vector<std::thread> threads;
  std::vector<common_msgs::msg::PerceptionHealth> results;
  std::mutex results_mutex;
  
  for (int i = 0; i < 10; ++i) {
    threads.emplace_back([&]() {
      auto health = health_monitor_->aggregate_health();
      std::lock_guard<std::mutex> lock(results_mutex);
      results.push_back(health);
    });
  }
  
  for (auto & t : threads) {
    t.join();
  }
  
  // All results should be valid
  EXPECT_EQ(results.size(), 10);
  for (const auto & health : results) {
    EXPECT_GE(health.lidar_health, 0.0f);
    EXPECT_LE(health.lidar_health, 1.0f);
  }
}

// Test 9: UpdatesOnEachAggregation
TEST_F(HealthMonitorTest, UpdatesOnEachAggregation)
{
  auto mock_driver = std::make_shared<MockSensorDriver>("lidar");
  
  sensor_interfaces::HealthStatus status1;
  status1.score = 0.8f;
  mock_driver->setHealthStatus(status1);
  
  health_monitor_->register_driver("lidar", mock_driver);
  
  auto health1 = health_monitor_->aggregate_health();
  EXPECT_FLOAT_EQ(health1.lidar_health, 0.8f);
  
  // Update driver health
  sensor_interfaces::HealthStatus status2;
  status2.score = 0.6f;
  mock_driver->setHealthStatus(status2);
  
  auto health2 = health_monitor_->aggregate_health();
  EXPECT_FLOAT_EQ(health2.lidar_health, 0.6f);
}

// Test 10: PreservesTimestampOrdering
TEST_F(HealthMonitorTest, PreservesTimestampOrdering)
{
  auto mock_driver = std::make_shared<MockSensorDriver>("lidar");
  sensor_interfaces::HealthStatus status;
  status.score = 1.0f;
  mock_driver->setHealthStatus(status);
  
  health_monitor_->register_driver("lidar", mock_driver);
  
  rclcpp::Time time1 = node_->now();
  auto health = health_monitor_->aggregate_health();
  rclcpp::Time time2 = node_->now();
  
  EXPECT_GE(health.stamp.sec + health.stamp.nanosec * 1e-9,
            time1.sec + time1.nanosec * 1e-9);
  EXPECT_LE(health.stamp.sec + health.stamp.nanosec * 1e-9,
            time2.sec + time2.nanosec * 1e-9);
}

}  // namespace testing
}  // namespace sensor_core

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
