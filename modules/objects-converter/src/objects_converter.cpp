#include "objects_converter/objects_converter.hpp"

namespace objects_converter {

ObjectsConverter::ObjectsConverter(const rclcpp::NodeOptions &options)
  : Node("objects_converter", options),
    sequence_number_(0) {
  spdlog::info("[objects-converter] Initializing Objects Converter Node");
  try {
    loadConfiguration();
    setupDDS();
    setupMQTT();
    setupSubscriber();
    spdlog::info("[objects-converter] Objects Converter Node initialized successfully");
  } catch (const std::exception &e) {
    spdlog::error("[objects-converter] Failed to initialize Objects Converter: {}", e.what());
    throw;
  }
}

ObjectsConverter::~ObjectsConverter() {
  spdlog::info("[objects-converter] Shutting down Objects Converter Node");
  dds_.reset();
  mqtt_wrapper_.reset();
}

void ObjectsConverter::setupDDS() {
  try {
    // Keep static callback; it's only for completeness (we publish; we don’t subscribe here)
    dds_ = std::make_unique<Dds>("ObjectsConverter", config_.domain_id, on_message);
    dds_->provision_publisher("aw/out/objects");

    spdlog::info("[objects-converter] DDS Publisher setup complete - Domain ID: {}, Topic: aw/out/objects",
                 config_.domain_id);
  } catch (const std::exception &e) {
    spdlog::error("[objects-converter] Failed to setup DDS: {}", e.what());
    throw;
  }
}

void ObjectsConverter::setupMQTT() {
  try {
    data_mqtt_server mqttInfo;
    mqttInfo.client_id = "vpi-objects-converter";
    mqttInfo.address   = "tcp://" + config_.mqtt_host + ":1883";

    mqtt_wrapper_ = std::make_unique<MqttWrapper>(mqttInfo);

    // Wait for MQTT connection with timeout
    auto start_time = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(10);

    while (!mqtt_wrapper_->is_connected()) {
      if (std::chrono::steady_clock::now() - start_time > timeout) {
        throw std::runtime_error("MQTT connection timeout");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    spdlog::info("[objects-converter] Connected to MQTT server");
    spdlog::info("[objects-converter] MQTT setup complete - Host: {}, Topic: aw/out/objects",
                 config_.mqtt_host);
  } catch (const std::exception &e) {
    spdlog::error("[objects-converter] Failed to setup MQTT: {}", e.what());
    throw;
  }
}

void ObjectsConverter::setupSubscriber() {
  predicted_objects_sub_ = this->create_subscription<autoware_auto_perception_msgs::msg::TrackedObjects>(
      "/perception/object_recognition/tracking/objects",
      10,
      std::bind(&ObjectsConverter::objectsCallback, this, std::placeholders::_1));

  spdlog::info("[objects-converter] Subscriber initialized");
}

void ObjectsConverter::objectsCallback(
    const autoware_auto_perception_msgs::msg::TrackedObjects::SharedPtr msg) {

  if (msg->objects.empty()) {
    spdlog::debug("[objects-converter] No objects in message, ignoring...");
    return;
  }

  // Parse the message into our lightweight structs
  std::list<autowareObject> objects =
      parse_msg(*msg, config_.reference_latitude, config_.reference_longitude,
                config_.ignore_unknown_objects);

  // Build JSON payload
  std::string serialized_list = structs_to_string(objects, sequence_number_);

  spdlog::debug("[objects-converter] Sending OBJECTS message: {}", serialized_list);

  dds_->publish("aw/out/objects", serialized_list);
  mqtt_wrapper_->publish("aw/out/objects", serialized_list);

  sequence_number_++;
  if (sequence_number_ > MAX_SEQUENCE_NUMBER) sequence_number_ = 0;
}

void ObjectsConverter::on_message(const std::string &topic, const std::string &message) {
  // Unused in current flow (we only publish), but keep for completeness
  spdlog::debug("[objects-converter] Received message on topic '{}' : {}", topic, message);
}

std::list<ObjectsConverter::autowareObject> ObjectsConverter::parse_msg(
    const autoware_auto_perception_msgs::msg::TrackedObjects &msg,
    double reference_latitude,
    double reference_longitude,
    bool ignore_unknown_objects) {

  std::list<autowareObject> aw_objects;
  const unsigned long int timestamp =
      static_cast<unsigned long int>(
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::system_clock::now().time_since_epoch()).count());

  spdlog::debug("[objects-converter] Timestamp: {} with {} objects",
                timestamp, msg.objects.size());
  spdlog::debug("[objects-converter] Reference Latitude: {}, Reference Longitude: {}",
                reference_latitude, reference_longitude);

  for (const auto &object : msg.objects) {
    // UUID as hex
    std::string obj_uuid = uuidToHexString(const_cast<unique_identifier_msgs::msg::UUID&>(object.object_id));

    // Unknown class handling
    if (!object.classification.empty() && object.classification[0].label == 0) {
      spdlog::warn("[objects-converter] Object is unknown");
      if (ignore_unknown_objects) {
        spdlog::warn("[objects-converter] Ignoring unknown object");
        continue;
      }
    }

    const double x = object.kinematics.pose_with_covariance.pose.position.x;
    const double y = object.kinematics.pose_with_covariance.pose.position.y;

    const double obj_lat = reference_latitude + (y / EARTH_RADIUS_M);
    const double obj_lon = reference_longitude +
        (x / (EARTH_RADIUS_M * std::cos(reference_latitude * M_PI_180)));

    const float obj_heading =
        quaternion_to_heading(
            object.kinematics.pose_with_covariance.pose.orientation.x,
            object.kinematics.pose_with_covariance.pose.orientation.y,
            object.kinematics.pose_with_covariance.pose.orientation.z,
            object.kinematics.pose_with_covariance.pose.orientation.w);

    autowareObject aw_object{};

    int new_classification = 0;
    try {
      if (!object.classification.empty()) {
        new_classification = classification_converstion_.at(object.classification[0].label);
      }
    } catch (const std::out_of_range &) {
      spdlog::error("[objects-converter] Classification not found: {}",
                    object.classification[0].label);
      new_classification = 0;
    }

    aw_object.classification = new_classification;
    aw_object.confidence     = !object.classification.empty()
                                 ? static_cast<int>(object.classification[0].probability * 100.0f)
                                 : 0;

    aw_object.heading     = obj_heading;
    aw_object.cov_heading = object.kinematics.pose_with_covariance.covariance[35];

    aw_object.size_x = object.shape.dimensions.x;
    aw_object.size_y = object.shape.dimensions.y;
    aw_object.size_z = object.shape.dimensions.z;

    aw_object.latitude  = obj_lat;
    aw_object.longitude = obj_lon;

    aw_object.x = static_cast<float>(x);
    aw_object.y = static_cast<float>(y);
    aw_object.z = object.kinematics.pose_with_covariance.pose.position.z;

    aw_object.cov_x = object.kinematics.pose_with_covariance.covariance[0];
    aw_object.cov_y = object.kinematics.pose_with_covariance.covariance[7];
    aw_object.cov_z = object.kinematics.pose_with_covariance.covariance[14];

    float speed = object.kinematics.twist_with_covariance.twist.linear.x;
    if (speed < 0.0f) speed = -speed;
    aw_object.speed     = speed;
    aw_object.cov_speed = object.kinematics.twist_with_covariance.covariance[0];

    aw_object.twist_angz      = object.kinematics.twist_with_covariance.twist.angular.z;
    aw_object.cov_twist_angz  = object.kinematics.twist_with_covariance.covariance[35];

    aw_object.timestamp = timestamp;
    aw_object.objectID  = getOrCreateObjectID(obj_uuid);

    spdlog::debug("[objects-converter] Object {} lat:{}, lon:{}, heading:{}, speed:{}",
                  aw_object.objectID, aw_object.latitude, aw_object.longitude,
                  aw_object.heading, aw_object.speed);
    spdlog::debug("------------------------------------");

    aw_objects.push_back(aw_object);
  }

  return aw_objects;
}

std::string ObjectsConverter::structs_to_string(const std::list<autowareObject>& objects,
                                                int sequence_number) {
  rapidjson::Document d(rapidjson::kObjectType);
  auto& alloc = d.GetAllocator();

  rapidjson::Value arr(rapidjson::kArrayType);
  arr.Reserve(static_cast<rapidjson::SizeType>(objects.size()), alloc);

  for (const auto& obj : objects) {
    rapidjson::Value o(rapidjson::kObjectType);

    o.AddMember("objID",            obj.objectID,        alloc);
    o.AddMember("sensorID",         2,                   alloc);
    o.AddMember("x",                obj.x,               alloc);
    o.AddMember("y",                obj.y,               alloc);
    o.AddMember("z",                obj.z,               alloc);
    o.AddMember("latitude",         obj.latitude,        alloc);
    o.AddMember("longitude",        obj.longitude,       alloc);
    o.AddMember("speed",            obj.speed,           alloc);
    o.AddMember("heading",          obj.heading,         alloc);
    o.AddMember("cov_heading",      obj.cov_heading,     alloc);
    o.AddMember("cov_speed",        obj.cov_speed,       alloc);
    o.AddMember("timestamp",        obj.timestamp,       alloc);
    o.AddMember("confidence",       obj.confidence,      alloc);
    o.AddMember("acceleration",     0,                   alloc);
    o.AddMember("size_x",           obj.size_x,          alloc);
    o.AddMember("size_y",           obj.size_y,          alloc);
    o.AddMember("size_z",           obj.size_z,          alloc);
    o.AddMember("cov_x",            obj.cov_x,           alloc);
    o.AddMember("cov_y",            obj.cov_y,           alloc);
    o.AddMember("cov_z",            obj.cov_z,           alloc);
    o.AddMember("twist_angz",       obj.twist_angz,      alloc);
    o.AddMember("cov_twist_angz",   obj.cov_twist_angz,  alloc);
    o.AddMember("classification",   obj.classification,  alloc);

    arr.PushBack(o, alloc);
  }

  d.AddMember("objects", arr, alloc);
  d.AddMember("sequenceNumber", sequence_number, alloc);

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  d.Accept(writer);
  return buffer.GetString();
}

int ObjectsConverter::getOrCreateObjectID(const std::string &object_id) {
  auto it = object_id_mapping_.find(object_id);
  if (it == object_id_mapping_.end()) {
    object_id_mapping_[object_id] = object_id_counter_;
    object_id_counter_++;
    if (object_id_counter_ >= 65536) {
      object_id_counter_ = 1;
    }
    return object_id_mapping_[object_id];
  }
  return it->second;
}

std::string ObjectsConverter::uuidToHexString(unique_identifier_msgs::msg::UUID &id) {
  std::stringstream ss;
  for (int i = 0; i < 16; ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << +id.uuid[i];
  }
  return ss.str();
}

float ObjectsConverter::radians_to_degrees(const float &radians) {
  return radians * (180.0f / static_cast<float>(M_PI));
}

float ObjectsConverter::quaternion_to_heading(const float &x, const float &y,
                                              const float &z, const float &w) {
  tf2::Quaternion q(x, y, z, w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  float converted_angle = (yaw < 0.0)
                            ? radians_to_degrees(static_cast<float>(yaw + (2 * M_PI)))
                            : radians_to_degrees(static_cast<float>(yaw));

  float heading_angle = 90.0f - converted_angle;
  if (heading_angle < 0.0f) heading_angle += 360.0f;
  return heading_angle;
}

void ObjectsConverter::loadConfiguration() {
  // Corrected default path
  std::string config_path = "/objects-converter/config.ini";
  INIReader reader(config_path);

  if (reader.ParseError() < 0) {
    spdlog::warn("[objects-converter] Can't load config file from {}, using defaults", config_path);
    return;
  }

  config_.domain_id            = reader.GetInteger("objects-converter", "dds_domain_id", 0);
  config_.debug_level          = reader.GetInteger("objects-converter", "debug", 0);
  config_.reference_latitude   = reader.GetReal("objects-converter", "reference_latitude", 0.0);
  config_.reference_longitude  = reader.GetReal("objects-converter", "reference_longitude", 0.0);
  config_.ignore_unknown_objects = reader.GetBoolean("objects-converter", "ignore_unknown_objects", true);
  config_.mqtt_host            = reader.Get("objects-converter", "mqtt_host", "127.0.0.1");

  spdlog::info("[objects-converter] Configuration loaded:");
  spdlog::info("[objects-converter] DDS Domain ID: {}", config_.domain_id);
  spdlog::info("[objects-converter] Debug Level: {}", config_.debug_level);
  spdlog::info("[objects-converter] Reference Latitude: {}", config_.reference_latitude);
  spdlog::info("[objects-converter] Reference Longitude: {}", config_.reference_longitude);
  spdlog::info("[objects-converter] Ignore Unknown Objects: {}", config_.ignore_unknown_objects);
  spdlog::info("[objects-converter] MQTT Host: {}", config_.mqtt_host);

  spdlog::set_level(config_.debug_level > 0 ? spdlog::level::debug : spdlog::level::info);
}

// ---- optional helpers declared in header (not heavily used here) ----
std::string ObjectsConverter::documentToString(const rapidjson::Document &doc) const {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  doc.Accept(writer);
  return buffer.GetString();
}

std::string ObjectsConverter::jsonToString(rapidjson::Document &d) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  d.Accept(writer);
  return buffer.GetString();
}

} // namespace objects_converter