#ifndef OBJECTS_CONVERTER_HPP
#define OBJECTS_CONVERTER_HPP


#include <functional>
#include <memory>
#include <chrono>
#include <tuple>
#include <map>
#include <mutex>
#include <list>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <cmath>
#include <chrono>
#include <thread>
#include <boost/asio.hpp>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <boost/uuid/uuid.hpp>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_objects.hpp"

// TF2 includes
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// External libraries
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <rapidjson/prettywriter.h>
#include <spdlog/spdlog.h>

// Custom includes
#include "config_reader.hpp"
#include "fastdds-cpp-wrapper/dds.hpp"
#include "mqttwrapper.h"

namespace objects_converter
{

  class ObjectsConverter : public rclcpp::Node
  {
  public:
    explicit ObjectsConverter(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~ObjectsConverter();

  private:
    // Configuration structure
    struct Config {
      int domain_id = 0;
      int debug_level = 0;
      double reference_latitude = 0.0;
      double reference_longitude = 0.0;
      bool ignore_unknown_objects = true;
      string mqtt_host = "127.0.0.1";
    };

    // Struct to hold object data
    struct autowareObject
    {
      int classification;
      int confidence;
      float heading;
      float cov_heading;
      double latitude;
      double longitude;
      float speed;
      float cov_speed;
      unsigned long int timestamp;
      int objectID;
      float size_x;
      float size_y;
      float size_z;
      float x;
      float y;
      float z;
      float cov_x;
      float cov_y;
      float cov_z;
      float twist_angz;
      float cov_twist_angz;
    };

    map<string, int> object_id_mapping_;
    int object_id_counter_ = 1;
    
    map<int, string> classification_labels_ = {
        {0, "unknown"},
        {1, "car"},
        {2, "truck"},
        {3, "bus"},
        {4, "trailer"},
        {5, "motorcycle"},
        {6, "bicycle"},
        {7, "pedestrian"}};

    map<int, int> classification_converstion_ = {
        {0, 0}, // unknown
        {1, 5}, // car
        {2, 7}, // truck
        {3, 6}, // bus
        {4, 9}, // trailer
        {5, 4}, // motorcycle
        {6, 2}, // bycicle
        {7, 1}  // pedestrian
    };

    // Constants
    static constexpr double EARTH_RADIUS_M = 111320.0;
    static constexpr double M_PI_180 = M_PI / 180.0;
    static constexpr int MAX_SEQUENCE_NUMBER = 100000;

    // Member variables
    Config config_;
    
    int sequence_number_;

    // DDS and MQTT
    std::unique_ptr<Dds> dds_;
    std::unique_ptr<MqttWrapper> mqtt_wrapper_;

    // Subscribers
    rclcpp::Subscription<autoware_auto_perception_msgs::msg::TrackedObjects>::SharedPtr predicted_objects_sub_;

    // Private methods
    void loadConfiguration();
    void setupDDS();
    void setupMQTT();
    void setupSubscriber();

    // Callback method
    void objectsCallback(const autoware_auto_perception_msgs::msg::TrackedObjects::SharedPtr msg);

    // Utility methods
    std::string documentToString(const rapidjson::Document &doc) const;
    std::list<autowareObject> parse_msg(const autoware_auto_perception_msgs::msg::TrackedObjects &msg, double reference_latitude, double reference_longitude, bool ignore_unknown_objects);
    std::string structs_to_string(const std::list<autowareObject>& objects, int sequence_number);
    int getOrCreateObjectID(const string &object_id);
    string uuidToHexString(unique_identifier_msgs::msg::UUID &id);
    float radians_to_degrees(const float &radians);
    float quaternion_to_heading(const float &x, const float &y, const float &z, const float &w);
    string jsonToString(rapidjson::Document &d);

    // DDS callback (unused static)
    static void on_message(const std::string &topic, const std::string &message);
  };

}

#endif // OBJECTS_CONVERTER_HPP