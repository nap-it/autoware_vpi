#include <sstream>
#include "v2x_converter/v2x_converter.hpp"

namespace v2x_converter {

V2XConverter::V2XConverter(const rclcpp::NodeOptions &options)
  : Node("v2x_converter", options) {
  
  spdlog::info("[v2x-converter] Initializing V2X Converter Node");
  
  try {
    loadConfiguration();
    setupPublisher();
    setupMQTT();
    spdlog::info("[v2x-converter] Node initialized successfully");
  } catch (const std::exception &e) {
    spdlog::error("[v2x-converter] Initialization failed: {}", e.what());
    throw;
  }
}

V2XConverter::~V2XConverter() {
    if (mqtt_client_ && mqtt_client_->is_connected()) {
        try {
            spdlog::info("[v2x-converter] Disconnecting MQTT...");
            mqtt_client_->disconnect()->wait();
        } catch (...) {}
    }
}

void V2XConverter::loadConfiguration() {
  spdlog::info("[v2x-converter] Loading Configuration");

  std::string config_path = "/v2x-converter/config.ini"; 
  this->declare_parameter("config_file", config_path);
  config_path = this->get_parameter("config_file").as_string();

  INIReader reader(config_path);

  if (reader.ParseError() < 0) {
    spdlog::warn("[v2x-converter] Config file not found at {}, using defaults", config_path);
  }

  config_.debug_level = reader.GetInteger("v2x-converter", "debug", 0);
  config_.reference_latitude = reader.GetReal("v2x-converter", "reference_latitude", 0.0);
  config_.reference_longitude = reader.GetReal("v2x-converter", "reference_longitude", 0.0);
  config_.mqtt_host = reader.Get("v2x-converter", "mqtt_host", "127.0.0.1");
  config_.mqtt_topic_in = reader.Get("v2x-converter", "mqtt_topic_in", "/aw/in/objects");
  
  // Per defecte a deteccions, però es pot sobreescriure per config.ini
  config_.ros_topic_out = reader.Get("v2x-converter", "ros_topic_out", "/perception/object_recognition/detection/objects");

  spdlog::set_level(config_.debug_level > 0 ? spdlog::level::debug : spdlog::level::info);
  
  spdlog::info("[v2x-converter] Config Loaded: Host={}, Topic={}, RefLat={}, RefLon={}", 
      config_.mqtt_host, config_.mqtt_topic_in, config_.reference_latitude, config_.reference_longitude);
}

void V2XConverter::setupPublisher() {
  spdlog::debug("[v2x-converter] Setting up the publisher");
  // CANVI: Tipus de missatge DetectedObjects
  objects_pub_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
      config_.ros_topic_out, 10);
  spdlog::debug("[v2x-converter] Publisher set up");
}

void V2XConverter::setupMQTT() {
  spdlog::debug("[v2x-converter] Setting up the MQTT");

  std::string address = "tcp://" + config_.mqtt_host + ":1883";
  std::string client_id = "vpi-v2x-converter";

  spdlog::info("[v2x-converter] Connecting to MQTT Broker: {}", address);

  mqtt_client_ = std::make_shared<mqtt::async_client>(address, client_id);
  mqtt_client_->set_callback(*this);

  auto connOpts = mqtt::connect_options_builder()
    .clean_session(true)
    .automatic_reconnect(true)
    .finalize();

  try {
    mqtt_client_->connect(connOpts)->wait();
    spdlog::info("[v2x-converter] Connected to Broker");
    mqtt_client_->subscribe(config_.mqtt_topic_in, 1)->wait();
    spdlog::info("[v2x-converter] Subscribed to topic: {}", config_.mqtt_topic_in);
  } catch (const mqtt::exception& exc) {
    spdlog::error("[v2x-converter] MQTT Setup Error: {} (Reason Code: {})", exc.what(), exc.get_reason_code());
    throw;
  }
  spdlog::debug("[v2x-converter] MQTT set up");
}

void V2XConverter::message_arrived(mqtt::const_message_ptr msg) {
    spdlog::debug("[v2x-converter] Message arrived");
    std::string payload = msg->to_string();

    try {
        auto ros_msg = jsonToMsg(payload);

        const char* actual_topic = objects_pub_->get_topic_name();
        size_t sub_count = objects_pub_->get_subscription_count();

        // Logging estil "bulletproof" que tenies
        spdlog::warn("[v2x-converter] ATTEMPTING PUBLISH");
        spdlog::warn("[v2x-converter] Topic Name: {}", actual_topic);
        spdlog::warn("[v2x-converter] Subscribers Found: {}", sub_count); 

        spdlog::debug("[v2x-converter] FRAME START Count: {}", ros_msg.objects.size());

        for (const auto& obj : ros_msg.objects) {
             // DetectedObjects no té UUID, fem servir la classificació o posició per al log
             spdlog::debug("[v2x-converter] Obj | X: {:.2f} | Y: {:.2f}", 
                obj.kinematics.pose_with_covariance.pose.position.x, 
                obj.kinematics.pose_with_covariance.pose.position.y);
        }

        objects_pub_->publish(ros_msg);

        if (sub_count == 0) {
            spdlog::error("[v2x-converter] WARNING: Message published to '{}', but NO ONE is listening!", actual_topic);
        } else {
            spdlog::info("[v2x-converter] Successfully sent to {} subscribers on {}", sub_count, actual_topic);
        }

    } catch (const std::exception &e) {
        spdlog::error("[v2x-converter] Processing Error: {}", e.what());
    }
}

void V2XConverter::connection_lost(const std::string& cause) {
    spdlog::warn("[v2x-converter] MQTT Connection Lost: {}", cause);
}

void V2XConverter::delivery_complete(mqtt::delivery_token_ptr token) {
    (void)token;
}

// =================================================================================
// CONVERSION LOGIC (Adaptada a DetectedObjects)
// =================================================================================

autoware_perception_msgs::msg::DetectedObjects V2XConverter::jsonToMsg(const std::string &json) {
  spdlog::debug("[v2x-converter] Entering conversion logic");

  autoware_perception_msgs::msg::DetectedObjects msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";

  rapidjson::Document d;
  d.Parse(json.c_str());

  if (d.HasParseError()) {
    spdlog::error("[v2x-converter] JSON Parse Error: {}", static_cast<int>(d.GetParseError()));
    return msg;
  }

  if (!d.IsObject() || !d.HasMember("objects")) {
    spdlog::error("[v2x-converter] JSON invalid or missing 'objects'");
    return msg;
  }

  const auto& objects = d["objects"];
  if (!objects.IsArray()) return msg;

  for (const auto& obj : objects.GetArray()) {
    autoware_perception_msgs::msg::DetectedObject det_obj;

    det_obj.existence_probability = 1.0;

    // Position
    double lat = obj.HasMember("latitude") ? obj["latitude"].GetDouble() : config_.reference_latitude;
    double lon = obj.HasMember("longitude") ? obj["longitude"].GetDouble() : config_.reference_longitude;
    std::pair<double, double> xy = latLonToXY(lat, lon);
    
    det_obj.kinematics.pose_with_covariance.pose.position.x = xy.first;
    det_obj.kinematics.pose_with_covariance.pose.position.y = xy.second;
    det_obj.kinematics.pose_with_covariance.pose.position.z = obj.HasMember("z") ? obj["z"].GetFloat() : 0.0;

    // Heading
    float heading = obj.HasMember("heading") ? obj["heading"].GetFloat() : 0.0f;
    det_obj.kinematics.pose_with_covariance.pose.orientation = headingToQuaternion(heading);

    // Shape
    det_obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    det_obj.shape.dimensions.x = obj.HasMember("size_x") ? obj["size_x"].GetFloat() : 0.5;
    det_obj.shape.dimensions.y = obj.HasMember("size_y") ? obj["size_y"].GetFloat() : 0.5;
    det_obj.shape.dimensions.z = obj.HasMember("size_z") ? obj["size_z"].GetFloat() : 1.7;

    // Speed i Atributs obligatoris de Kinematics
    float speed = obj.HasMember("speed") ? obj["speed"].GetFloat() : 0.0f;
    det_obj.kinematics.twist_with_covariance.twist.linear.x = speed;
    
    det_obj.kinematics.has_twist = true; 
    det_obj.kinematics.has_position_covariance = true;
    det_obj.kinematics.has_twist_covariance = true;
    det_obj.kinematics.orientation_availability = 1;

    // Covariàncies (imprescindibles per al Tracker)
    for(int i=0; i<36; i++){
        det_obj.kinematics.pose_with_covariance.covariance[i] = 0.0;
        det_obj.kinematics.twist_with_covariance.covariance[i] = 0.0;
    }
    det_obj.kinematics.pose_with_covariance.covariance[0] = 0.5;
    det_obj.kinematics.pose_with_covariance.covariance[7] = 0.5;
    det_obj.kinematics.pose_with_covariance.covariance[14] = 0.1;
    det_obj.kinematics.pose_with_covariance.covariance[35] = 0.1;
    det_obj.kinematics.twist_with_covariance.covariance[0] = 0.5;
    det_obj.kinematics.twist_with_covariance.covariance[35] = 0.1;

    // Classification
    autoware_perception_msgs::msg::ObjectClassification cls;
    int cls_in = obj.HasMember("classification") ? obj["classification"].GetInt() : 0;
    
    if (classification_map_.count(cls_in)) {
        cls.label = classification_map_.at(cls_in);
    } else {
        cls.label = 0; 
    }
    
    cls.probability = obj.HasMember("confidence") ? static_cast<float>(obj["confidence"].GetInt()) / 100.0f : 0.9f;
    
    det_obj.classification.push_back(cls);
    msg.objects.push_back(det_obj);
  }

  return msg;
}

std::pair<double, double> V2XConverter::latLonToXY(double lat, double lon) {
    double y = (lat - config_.reference_latitude) * EARTH_RADIUS_M;
    double cos_ref = std::cos(config_.reference_latitude * M_PI_180);
    double x = (lon - config_.reference_longitude) * (EARTH_RADIUS_M * cos_ref);
    return {x, y};
}

geometry_msgs::msg::Quaternion V2XConverter::headingToQuaternion(float heading_deg) {
    float yaw_deg = 90.0f - heading_deg;
    float yaw_rad = yaw_deg * (static_cast<float>(M_PI) / 180.0f);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_rad);
    return tf2::toMsg(q);
}

} // namespace v2x_converter