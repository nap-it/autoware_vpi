#ifndef BRAKING_SERVICE_HPP
#define BRAKING_SERVICE_HPP

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "tier4_external_api_msgs/srv/set_emergency.hpp"

// External
#include <spdlog/spdlog.h>
#include <rapidjson/document.h>

// STL
#include <string>
#include <vector>
#include <memory>
#include <cmath>

// Custom
#include "config_reader.hpp"
#include "fastdds-cpp-wrapper/dds.hpp"
#include "mqttwrapper.h"

namespace braking_service {

class BrakingService : public rclcpp::Node {
public:
  explicit BrakingService(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~BrakingService();

private:
  struct Config {
    int domain_id = 0;
    int debug_level = 1;
    std::string mqtt_host = "127.0.0.1";
  };

  // Members
  Config config_;

  std::unique_ptr<Dds> dds_;
  std::unique_ptr<MqttWrapper> mqtt_wrapper_;

  rclcpp::Client<tier4_external_api_msgs::srv::SetEmergency>::SharedPtr emergency_client_;
  bool inputBrake_ = false;

  // Setup
  void loadConfiguration();
  void setupDDS();
  void setupMQTT();
  void setupClient();

  // Callback (now a member function)
  void on_message(const std::string &topic, const std::string &message);
};

} // namespace braking_service

#endif // BRAKING_SERVICE_HPP