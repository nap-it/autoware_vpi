#ifndef V2X_CONVERTER_HPP
#define V2X_CONVERTER_HPP

#include <memory>
#include <chrono>
#include <tuple>
#include <map>
#include <string>
#include <cmath>
#include <thread>
#include <mutex>
#include <vector>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
//#include "autoware_perception_msgs/msg/tracked_objects.hpp"
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include "unique_identifier_msgs/msg/uuid.hpp"

// TF2 includes
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// External libraries
#include "rapidjson/document.h"
#include <spdlog/spdlog.h>

// Config Reader
#include "config_reader.hpp"

// 👇 NEW: Standard Paho MQTT Client (Bypassing the wrapper)
#include <mqtt/async_client.h>

namespace v2x_converter
{

  // Inherit from mqtt::callback to implement message_arrived
  class V2XConverter : public rclcpp::Node, public virtual mqtt::callback
  {
  public:
    explicit V2XConverter(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~V2XConverter();

  private:
    struct Config {
      int debug_level = 0;
      double reference_latitude = 0.0;
      double reference_longitude = 0.0;
      std::string mqtt_host = "127.0.0.1";
      std::string mqtt_topic_in = "/aw/in/objects";
      std::string ros_topic_out = "/perception/object_recognition/tracking/objects";
    };

    // Mapping External ID -> Autoware Label
    std::map<int, int> classification_map_ = {
        {0, 0}, {5, 1}, {7, 2}, {6, 3}, 
        {9, 4}, {4, 5}, {2, 6}, {1, 7}
    };

    static constexpr double EARTH_RADIUS_M = 111320.0;
    static constexpr double M_PI_180 = M_PI / 180.0;

    Config config_;

    // 👇 NEW: Direct Paho Client Pointer
    std::shared_ptr<mqtt::async_client> mqtt_client_;

    // Publisher
    //rclcpp::Publisher<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr objects_pub_;
    rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr objects_pub_;
    // Methods
    void loadConfiguration();
    void setupMQTT();
    void setupPublisher();

    // 👇 NEW: MQTT Callback Overrides (Required by mqtt::callback)
    void connection_lost(const std::string& cause) override;
    void message_arrived(mqtt::const_message_ptr msg) override;
    void delivery_complete(mqtt::delivery_token_ptr token) override;

    // Converters
    //autoware_perception_msgs::msg::TrackedObjects jsonToMsg(const std::string &json);
    autoware_perception_msgs::msg::DetectedObjects jsonToMsg(const std::string &json);
    unique_identifier_msgs::msg::UUID intToUuid(int id);
    geometry_msgs::msg::Quaternion headingToQuaternion(float heading_deg);
    std::pair<double, double> latLonToXY(double lat, double lon);
  };

}

#endif // V2X_CONVERTER_HPP