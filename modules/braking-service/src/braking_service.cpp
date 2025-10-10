#include "braking_service/braking_service.hpp"

namespace braking_service
{
  BrakingService::BrakingService(const rclcpp::NodeOptions &options)
      : Node("braking_service", options)
  {

    spdlog::info("[brake-service] Initializing Brake Service Node");

    try {
      loadConfiguration();
      setupDDS();
      setupMQTT();
      setupClient();

      spdlog::info("[braking-service] Braking Service Node initialized successfully");
    } catch (const std::exception &e) {
      spdlog::error("[braking-service] Failed to initialize Brake Service: {}", e.what());
      throw;
    }
  }

  BrakingService::~BrakingService() {
    spdlog::info("[braking-service] Shutting down Brake Service Node");

    // Clean up DDS and MQTT connections
    dds_.reset();
    mqtt_wrapper_.reset();
  }

  void BrakingService::setupDDS() {
    try {
      dds_ = std::make_unique<Dds>("VPIBrakingService", config_.domain_id, [this](const std::string &t, const std::string &m) { this->on_message(t, m); });
      dds_->subscribe("aw/in/brake");

      spdlog::info("[braking-service] DDS Subscriber setup complete - Domain ID: {}, Topic: aw/in/brake", config_.domain_id);
    } catch (const std::exception &e) {
      spdlog::error("[braking-service] Failed to setup DDS: {}", e.what());
      throw;
    }
  }

  void BrakingService::setupMQTT() {
    try {
      data_mqtt_server mqttInfo;
      mqttInfo.client_id = "vpi-braking_service";
      mqttInfo.address = "tcp://" + config_.mqtt_host + ":1883";
      string mqtt_topic = "aw/in/brake";

      vector<string> topics;
      topics.push_back(mqtt_topic);
      mqttInfo.subscription_topic = topics;

      mqtt_wrapper_ = std::make_unique<MqttWrapper>(mqttInfo, [this](const std::string &t, const std::string &m) { this->on_message(t, m); });

      // Wait for MQTT connection with timeout
      auto start_time = std::chrono::steady_clock::now();
      const auto timeout = std::chrono::seconds(10);

      while (!mqtt_wrapper_->is_connected()) {
        if (std::chrono::steady_clock::now() - start_time > timeout) {
          throw std::runtime_error("MQTT connection timeout");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      spdlog::info("[braking-service] Connected to MQTT server");
      spdlog::info("[braking-service] MQTT setup complete - Host: {}, Topic: aw/in/brake", config_.mqtt_host);

    } catch (const std::exception &e) {

      spdlog::error("[braking-service] Failed to setup MQTT: {}", e.what());
      throw;
    }
  }

  void BrakingService::setupClient() {
    emergency_client_ = this->create_client<tier4_external_api_msgs::srv::SetEmergency>("/api/autoware/set/emergency");

    // Wait for the service to become available
    while (!emergency_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        spdlog::error("[braking-service] Interrupted while waiting for the emergency service. Exiting.");
        return;
      }
      
      spdlog::warn("[braking-service] Emergency service not available, waiting again...");
    }
    
    spdlog::info("[braking-service] Emergency service available");
  }

  void BrakingService::on_message(const std::string &topic, const std::string &message) {
    spdlog::debug("[braking-service] Message: {}", message);

    rapidjson::Document d;
    d.Parse(message.c_str());

    if (topic == "aw/in/brake") {

      if (d.HasParseError()) {
        spdlog::error("[braking-service] Error parsing JSON message");
        return;
      }
      if (!d.HasMember("brake"))
      {
        spdlog::error("[braking-service] Message does not contain brake field");
        return;
      }
      if (!d["brake"].IsBool())
      {
        spdlog::error("[braking-service] Brake field is not a boolean");
        return;
      }
      inputBrake_ = d["brake"].GetBool();

      auto request = std::make_shared<tier4_external_api_msgs::srv::SetEmergency::Request>();

      // Set the emergency value based on the brake input
      request->emergency = inputBrake_;
      spdlog::debug("[braking-service] Changing emergency to {}", inputBrake_);

      // Call the service asynchronously
      auto result_future = emergency_client_->async_send_request(request, [request](rclcpp::Client<tier4_external_api_msgs::srv::SetEmergency>::SharedFuture future)
                                                      {
            try {
                auto response = future.get();
                spdlog::info("[braking-service] Service called with emergency: {}", request->emergency);

            } catch (const std::exception &e) {
                spdlog::error("[braking-service] Service call failed: {}", e.what());
            } });
    }
  }

  void BrakingService::loadConfiguration()
  {

    std::string config_path = "/braking-service/config.ini";
    INIReader reader(config_path);

    if (reader.ParseError() < 0)
    {
      spdlog::error("Can't load config file");
      return;
    }

    config_.domain_id = reader.GetInteger("braking-service", "dds_domain_id", 0);
    config_.debug_level = reader.GetInteger("braking-service", "debug", 0);
    config_.mqtt_host = reader.Get("braking-service", "mqtt_host", "127.0.0.1");

    spdlog::info("[braking-service] Fast-DDS Wrapper Domain ID: {}", config_.domain_id);
    spdlog::info("[braking-service] Debug Level: {}", config_.debug_level);
    spdlog::info("[braking-service] MQTT Host: {}", config_.mqtt_host);
  }

}