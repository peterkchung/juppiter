// About: Entry point for sensor_bridge_node.

#include "sensor_bridge/sensor_bridge_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sensor_bridge::SensorBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
