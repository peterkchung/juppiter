// About: Scenario Runner Node
// Executes FDIIR test scenarios from YAML configuration

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <chrono>

namespace juppiter
{

struct FaultInjection
{
  double time_sec;
  std::string type;
  std::string target;
  double duration_sec;
  std::map<std::string, double> parameters;
};

struct ExpectedBehavior
{
  double at_time;
  std::string expected_mode;
  std::string expected_primary;
  double mode_switch_deadline_sec;
};

struct TestScenario
{
  std::string name;
  std::string description;
  double duration_seconds;
  std::vector<FaultInjection> faults;
  std::vector<ExpectedBehavior> expectations;
  std::map<std::string, double> success_criteria;
};

class ScenarioRunner : public rclcpp::Node
{
public:
  ScenarioRunner()
  : Node("scenario_runner")
  {
    // Parameters
    this->declare_parameter<std::string>("scenario_file", "");
    this->declare_parameter<bool>("auto_start", true);
    
    std::string scenario_file = this->get_parameter("scenario_file").as_string();
    bool auto_start = this->get_parameter("auto_start").as_bool();
    
    // Subscribers to monitor system state
    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/perception/mode", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        current_mode_ = msg->data;
      });
      
    health_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/perception/health", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        current_health_ = msg->data;
      });
      
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/perception/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_odom_ = *msg;
      });
    
    // Publishers for fault injection
    fault_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/test/fault_injection", 10);
    
    // Timer for scenario execution
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { this->executeScenario(); });
    
    if (!scenario_file.empty() && auto_start) {
      if (loadScenario(scenario_file)) {
        RCLCPP_INFO(this->get_logger(), 
          "Loaded scenario: %s", scenario_.name.c_str());
        startScenario();
      }
    }
  }

  bool loadScenario(const std::string & file_path)
  {
    try {
      YAML::Node config = YAML::LoadFile(file_path);
      
      scenario_.name = config["scenario_name"].as<std::string>();
      scenario_.description = config["description"].as<std::string>();
      scenario_.duration_seconds = config["duration_seconds"].as<double>();
      
      // Load fault injections
      if (config["fault_injection"]) {
        for (const auto & fault : config["fault_injection"]) {
          FaultInjection fi;
          fi.time_sec = fault["time_sec"].as<double>();
          fi.type = fault["type"].as<std::string>();
          fi.target = fault["target"].as<std::string>();
          fi.duration_sec = fault["duration_sec"] ? 
            fault["duration_sec"].as<double>() : 0.0;
          scenario_.faults.push_back(fi);
        }
      }
      
      // Load expected behaviors
      if (config["expected_behavior"]) {
        for (const auto & exp : config["expected_behavior"]) {
          ExpectedBehavior eb;
          eb.at_time = exp["at_time"].as<double>();
          eb.expected_mode = exp["expected_mode"].as<std::string>();
          eb.expected_primary = exp["expected_primary"].as<std::string>();
          eb.mode_switch_deadline_sec = exp["mode_switch_deadline_sec"] ?
            exp["mode_switch_deadline_sec"].as<double>() : 2.0;
          scenario_.expectations.push_back(eb);
        }
      }
      
      // Load success criteria
      if (config["success_criteria"]) {
        for (const auto & crit : config["success_criteria"]) {
          std::string key = crit.first.as<std::string>();
          scenario_.success_criteria[key] = crit.second.as<double>();
        }
      }
      
      return true;
    } catch (const YAML::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), 
        "Failed to load scenario: %s", e.what());
      return false;
    }
  }

  void startScenario()
  {
    scenario_start_time_ = this->now();
    scenario_running_ = true;
    results_.clear();
    
    RCLCPP_INFO(this->get_logger(), 
      "Starting scenario: %s (duration: %.1fs)",
      scenario_.name.c_str(),
      scenario_.duration_seconds);
  }

private:
  void executeScenario()
  {
    if (!scenario_running_) return;
    
    auto elapsed = (this->now() - scenario_start_time_).seconds();
    
    // Check if scenario is complete
    if (elapsed >= scenario_.duration_seconds) {
      finishScenario();
      return;
    }
    
    // Inject faults at scheduled times
    for (const auto & fault : scenario_.faults) {
      if (elapsed >= fault.time_sec && 
          elapsed < fault.time_sec + 0.1) {
        injectFault(fault);
      }
    }
    
    // Check expected behaviors
    for (const auto & exp : scenario_.expectations) {
      if (elapsed >= exp.at_time && 
          elapsed < exp.at_time + 0.1) {
        checkExpectation(exp);
      }
    }
    
    // Collect metrics
    collectMetrics(elapsed);
  }
  
  void injectFault(const FaultInjection & fault)
  {
    RCLCPP_WARN(this->get_logger(), 
      "Injecting fault: %s on %s (duration: %.1fs)",
      fault.type.c_str(),
      fault.target.c_str(),
      fault.duration_sec);
    
    std_msgs::msg::String msg;
    msg.data = fault.type + "|" + fault.target + "|" + 
               std::to_string(fault.duration_sec);
    fault_pub_->publish(msg);
    
    results_["faults_injected"].push_back(
      std::to_string(fault.time_sec) + ": " + fault.type);
  }
  
  void checkExpectation(const ExpectedBehavior & exp)
  {
    RCLCPP_INFO(this->get_logger(),
      "Checking expectation at t=%.1fs: mode=%s, primary=%s",
      exp.at_time, exp.expected_mode.c_str(), 
      exp.expected_primary.c_str());
    
    bool mode_ok = (current_mode_ == exp.expected_mode);
    
    if (mode_ok) {
      results_["expectations_met"].push_back(
        std::to_string(exp.at_time) + ": PASS");
      RCLCPP_INFO(this->get_logger(), "✓ Expectation met");
    } else {
      results_["expectations_met"].push_back(
        std::to_string(exp.at_time) + ": FAIL (got " + 
        current_mode_ + ")");
      RCLCPP_ERROR(this->get_logger(), 
        "✗ Expectation failed: expected %s, got %s",
        exp.expected_mode.c_str(), current_mode_.c_str());
    }
  }
  
  void collectMetrics(double elapsed)
  {
    // Store current state for analysis
    static int count = 0;
    if (count++ % 10 == 0) {  // Every second
      RCLCPP_DEBUG(this->get_logger(),
        "t=%.1fs mode=%s health=%s",
        elapsed, current_mode_.c_str(), 
        current_health_.c_str());
    }
  }
  
  void finishScenario()
  {
    scenario_running_ = false;
    
    RCLCPP_INFO(this->get_logger(), 
      "\n========================================");
    RCLCPP_INFO(this->get_logger(), 
      "Scenario Complete: %s", scenario_.name.c_str());
    RCLCPP_INFO(this->get_logger(), 
      "========================================");
    
    // Print results
    int pass_count = 0;
    int total_expectations = scenario_.expectations.size();
    
    for (const auto & result : results_["expectations_met"]) {
      if (result.find("PASS") != std::string::npos) {
        pass_count++;
      }
    }
    
    RCLCPP_INFO(this->get_logger(), 
      "Results: %d/%d expectations met", 
      pass_count, total_expectations);
    
    if (pass_count == total_expectations) {
      RCLCPP_INFO(this->get_logger(), "✓ SCENARIO PASSED");
    } else {
      RCLCPP_ERROR(this->get_logger(), "✗ SCENARIO FAILED");
    }
    
    // Shutdown node
    rclcpp::shutdown();
  }

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr health_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // State
  TestScenario scenario_;
  rclcpp::Time scenario_start_time_;
  bool scenario_running_ = false;
  
  std::string current_mode_;
  std::string current_health_;
  nav_msgs::msg::Odometry current_odom_;
  
  std::map<std::string, std::vector<std::string>> results_;
};

}  // namespace juppiter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<juppiter::ScenarioRunner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
