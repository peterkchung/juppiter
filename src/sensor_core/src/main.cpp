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
// About: Entry point for sensor bridge orchestrator node.

#include <rclcpp/rclcpp.hpp>

#include "sensor_core/sensor_bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<sensor_core::SensorBridgeNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("sensor_bridge"), 
                 "Fatal error: %s", e.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
