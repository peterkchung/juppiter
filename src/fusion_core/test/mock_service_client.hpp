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
// About: Manual mock for ROS 2 service client for unit testing

#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "common_msgs/msg/perception_health.hpp"
#include "common_msgs/srv/get_perception_health.hpp"

namespace juppiter
{
namespace fusion
{
namespace testing
{

/**
 * @brief Manual mock for GetPerceptionHealth service client
 * 
 * Allows setting response data and simulating service availability/timeouts
 * without actual ROS 2 service calls.
 */
class MockHealthServiceClient
{
public:
  MockHealthServiceClient()
  : service_available_(true)
  , should_timeout_(false)
  {
  }

  // Configuration methods
  void setServiceAvailable(bool available)
  {
    service_available_ = available;
  }

  void setShouldTimeout(bool timeout)
  {
    should_timeout_ = timeout;
  }

  void setResponse(const common_msgs::msg::PerceptionHealth & health)
  {
    response_ = health;
  }

  void setCallLatency(std::chrono::milliseconds latency)
  {
    call_latency_ = latency;
  }

  // Mock implementation of async_send_request
  std::shared_future<typename common_msgs::srv::GetPerceptionHealth::Response::SharedPtr>
  async_send_request(
    typename common_msgs::srv::GetPerceptionHealth::Request::SharedPtr /*request*/)
  {
    using Response = typename common_msgs::srv::GetPerceptionHealth::Response;
    using SharedPtr = typename Response::SharedPtr;
    
    auto promise = std::make_shared<std::promise<SharedPtr>>();
    
    if (should_timeout_) {
      // Simulate timeout - don't set value
      // In real implementation, this would time out
      promise->set_exception(std::make_exception_ptr(
        std::runtime_error("Service call timeout")));
    } else if (!service_available_) {
      promise->set_exception(std::make_exception_ptr(
        std::runtime_error("Service not available")));
    } else {
      auto response = std::make_shared<Response>();
      response->health = response_;
      promise->set_value(response);
    }
    
    return promise->get_future();
  }

  bool service_is_ready() const
  {
    return service_available_;
  }

  bool wait_for_service(const std::chrono::nanoseconds & timeout)
  {
    if (call_latency_ > std::chrono::duration_cast<std::chrono::milliseconds>(timeout)) {
      return false;  // Would timeout
    }
    return service_available_;
  }

  // Tracking
  int getCallCount() const { return call_count_; }
  void incrementCallCount() { call_count_++; }
  void resetCallCount() { call_count_ = 0; }

private:
  bool service_available_;
  bool should_timeout_;
  common_msgs::msg::PerceptionHealth response_;
  std::chrono::milliseconds call_latency_{0};
  int call_count_{0};
};

}  // namespace testing
}  // namespace fusion
}  // namespace juppiter
