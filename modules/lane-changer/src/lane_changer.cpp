#include "lane_changer/lane_changer.hpp"

namespace lane_changer {

LaneChanger::LaneChanger(const rclcpp::NodeOptions& options)
    : Node("lane_changer_node", options)
{
    spdlog::info("[lane-changer] Initializing");

    try {
        loadConfiguration();
        setupDDS();
        setupMQTT();
        setupSubscribers();
        spdlog::info("[lane-changer] Initialized successfully");
    } catch (const std::exception& e) {
        spdlog::error("[lane-changer] Initialization failed: {}", e.what());
        throw;
    }
}

LaneChanger::~LaneChanger()
{
    spdlog::info("[lane-changer] Shutting down");
    dds_.reset();
    mqtt_wrapper_.reset();
}

void LaneChanger::loadConfiguration()
{
    INIReader reader("/lane-changer/config.ini");

    if (reader.ParseError() < 0) {
        spdlog::warn("[lane-changer] Can't read config.ini, using defaults");
        return;
    }

    config_.domain_id         = reader.GetInteger("lane-changer", "dds_domain_id", 0);
    config_.debug             = reader.GetInteger("lane-changer", "debug", 0);
    config_.mqtt_host         = reader.Get("lane-changer", "mqtt_host", "127.0.0.1");
    config_.ref_lat           = reader.GetReal("lane-changer", "reference_latitude", 40.632263);
    config_.ref_lon           = reader.GetReal("lane-changer", "reference_longitude", -8.629927);
    config_.distance_to_start = reader.GetReal("lane-changer", "distance_to_start", 3.0);

    if (config_.debug > 0) {
        spdlog::set_level(spdlog::level::debug);
    } else {
        spdlog::set_level(spdlog::level::info);
    }

    spdlog::info("[lane-changer] DDS Domain ID:     {}", config_.domain_id);
    spdlog::info("[lane-changer] Debug:             {}", config_.debug);
    spdlog::info("[lane-changer] MQTT Host:         {}", config_.mqtt_host);
    spdlog::info("[lane-changer] Reference lat:     {}", config_.ref_lat);
    spdlog::info("[lane-changer] Reference lon:     {}", config_.ref_lon);
    spdlog::info("[lane-changer] Distance to start: {}", config_.distance_to_start);
}

void LaneChanger::setupDDS()
{
    dds_ = std::make_unique<Dds>(
        "LaneChanger",
        config_.domain_id,
        [this](const std::string& topic, const std::string& msg) { onMessage(topic, msg); });

    dds_->subscribe("aw/in/lane_change/overtake/request");
    dds_->subscribe("aw/in/lane_change/overtake/end_point");
    dds_->subscribe("aw/in/lane_change/direct");
    dds_->provision_publisher("aw/out/lane_change/overtake/start_point");
    dds_->provision_publisher("aw/out/timestamps");

    spdlog::info("[lane-changer] DDS setup complete - Domain ID: {}", config_.domain_id);
}

void LaneChanger::setupMQTT()
{
    data_mqtt_server mqttInfo;
    mqttInfo.client_id          = "vpi-lane-changer";
    mqttInfo.address            = "tcp://" + config_.mqtt_host + ":1883";
    mqttInfo.subscription_topic = {
        "aw/in/lane_change/overtake/request",
        "aw/in/lane_change/overtake/end_point",
        "aw/in/lane_change/direct"
    };

    mqtt_wrapper_ = std::make_unique<MqttWrapper>(
        mqttInfo,
        [this](const std::string& topic, const std::string& msg) { onMessage(topic, msg); });

    auto start = std::chrono::steady_clock::now();
    while (!mqtt_wrapper_->is_connected()) {
        if (std::chrono::steady_clock::now() - start > std::chrono::seconds(10))
            throw std::runtime_error("MQTT connection timeout");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    spdlog::info("[lane-changer] MQTT connected - Host: {}", config_.mqtt_host);
}

void LaneChanger::setupSubscribers()
{
    using std::placeholders::_1;

    pose_sub_ = create_subscription<Odometry>(
        "/localization/kinematic_state", 1,
        std::bind(&LaneChanger::poseCallback, this, _1));

    trajectory_sub_ = create_subscription<Trajectory>(
        "/planning/scenario_planning/trajectory", 1,
        std::bind(&LaneChanger::trajectoryCallback, this, _1));

    route_sub_ = create_subscription<LaneletRoute>(
        "/planning/mission_planning/route", 1,
        std::bind(&LaneChanger::routeCallback, this, _1));

    route_pub_ = create_publisher<LaneletRoute>(
        "/planning/mission_planning/route",
        rclcpp::QoS(10).transient_local().reliable());

    spdlog::info("[lane-changer] Subscribers initialized");
}

void LaneChanger::poseCallback(const Odometry::SharedPtr msg)
{
    const Pose& current_pose = msg->pose.pose;

    {
        std::lock_guard<std::mutex> lock(response_mutex_);
        if (lane_change_response_) {
            double dist = std::sqrt(
                std::pow(current_pose.position.x - start_pose_.position.x, 2) +
                std::pow(current_pose.position.y - start_pose_.position.y, 2));
            spdlog::debug("[lane-changer] Distance to start point: {:.2f}m", dist);

            if (dist <= config_.distance_to_start) {
                spdlog::debug("[lane-changer] Start point reached — activating lane change");
                latest_route_         = modifyRoute(latest_route_);
                lane_change_response_ = false;
                route_pub_->publish(latest_route_);

                std::lock_guard<std::mutex> lock2(lane_change_mutex_);
                lane_change_on_ = true;
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(lane_change_mutex_);
        if (lane_change_on_) {
            double end_x = (end_point_.lon - config_.ref_lon) *
                           (EARTH_RADIUS_M * std::cos(config_.ref_lat * M_PI_180));
            double end_y = (end_point_.lat - config_.ref_lat) * EARTH_RADIUS_M;

            double dist = std::sqrt(
                std::pow(current_pose.position.x - end_x, 2) +
                std::pow(current_pose.position.y - end_y, 2));
            spdlog::debug("[lane-changer] Distance to end point: {:.2f}m", dist);

            if (dist <= config_.distance_to_start) {
                spdlog::debug("[lane-changer] End point reached — restoring lane");
                lane_change_on_ = false;
                latest_route_   = modifyRoute(latest_route_);
                route_pub_->publish(latest_route_);
            }
        }
    }
}

void LaneChanger::trajectoryCallback(const Trajectory::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(request_mutex_);
    latest_trajectory_ = *msg;
}

void LaneChanger::routeCallback(const LaneletRoute::SharedPtr msg)
{
    latest_route_ = *msg;
    spdlog::debug("[lane-changer] Route updated");
}

LaneChanger::LaneletRoute LaneChanger::modifyRoute(LaneletRoute route)
{
    LaneletRoute modified = route;
    modified.segments.clear();

    for (const auto& segment : route.segments) {
        if (segment.primitives.size() < 2) {
            spdlog::warn("[lane-changer] Segment has fewer than 2 lanes — skipping");
            modified.segments.push_back(segment);
            continue;
        }

        LaneletSegment new_segment = segment;
        new_segment.preferred_primitive =
            (segment.preferred_primitive == segment.primitives[0])
                ? segment.primitives[1]
                : segment.primitives[0];

        modified.segments.push_back(new_segment);
    }

    return modified;
}

void LaneChanger::onMessage(const std::string& topic, const std::string& message)
{
    spdlog::debug("[lane-changer] Received on {}: {}", topic, message);

    if (topic == "aw/in/lane_change/overtake/request") {
        if (message != "start" && message != "START") {
            spdlog::warn("[lane-changer] Unexpected message on request topic: {}", message);
            return;
        }

        unsigned long ts1 = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        std::string ts1_str = "{\"timestamp_1\": " + std::to_string(ts1 / 1000000.0) + "}";
        mqtt_wrapper_->publish("aw/out/timestamps", ts1_str);
        dds_->publish("aw/out/timestamps", ts1_str);

        TrajectoryPoint point;
        {
            std::lock_guard<std::mutex> lock(request_mutex_);
            point            = latest_trajectory_.points[POINT_OFFSET];
            start_pose_      = point.pose;
            start_point_pending_ = true;
        }

        TrajectoryPointData tp;
        tp.x   = point.pose.position.x;
        tp.y   = point.pose.position.y;
        tp.lat = config_.ref_lat + (tp.y / EARTH_RADIUS_M);
        tp.lon = config_.ref_lon + (tp.x / (EARTH_RADIUS_M * std::cos(config_.ref_lat * M_PI_180)));
        tp.alt = point.pose.position.z;
        tp.heading = quaternionToHeading(
            point.pose.orientation.x, point.pose.orientation.y,
            point.pose.orientation.z, point.pose.orientation.w);
        tp.orientation_x = point.pose.orientation.x;
        tp.orientation_y = point.pose.orientation.y;
        tp.orientation_z = point.pose.orientation.z;
        tp.orientation_w = point.pose.orientation.w;
        tp.longitudinal_velocity = point.longitudinal_velocity_mps;
        tp.lateral_velocity      = point.lateral_velocity_mps;
        tp.acceleration          = point.acceleration_mps2;
        tp.heading_rate          = point.heading_rate_rps;
        tp.front_wheel_angle     = point.front_wheel_angle_rad;
        tp.rear_wheel_angle      = point.rear_wheel_angle_rad;

        rapidjson::Document doc;
        doc.SetObject();
        auto& alloc = doc.GetAllocator();

        rapidjson::Value tp_json(rapidjson::kObjectType);
        tp_json.AddMember("x",                     tp.x,                     alloc);
        tp_json.AddMember("y",                     tp.y,                     alloc);
        tp_json.AddMember("latitude",              tp.lat,                   alloc);
        tp_json.AddMember("longitude",             tp.lon,                   alloc);
        tp_json.AddMember("altitude",              tp.alt,                   alloc);
        tp_json.AddMember("heading",               tp.heading,               alloc);
        rapidjson::Value orient(rapidjson::kObjectType);
        orient.AddMember("x", tp.orientation_x, alloc);
        orient.AddMember("y", tp.orientation_y, alloc);
        orient.AddMember("z", tp.orientation_z, alloc);
        orient.AddMember("w", tp.orientation_w, alloc);
        tp_json.AddMember("orientation",           orient,                   alloc);
        tp_json.AddMember("longitudinal_velocity", tp.longitudinal_velocity, alloc);
        tp_json.AddMember("lateral_velocity",      tp.lateral_velocity,      alloc);
        tp_json.AddMember("acceleration",          tp.acceleration,          alloc);
        tp_json.AddMember("heading_rate",          tp.heading_rate,          alloc);
        tp_json.AddMember("front_wheel_angle",     tp.front_wheel_angle,     alloc);
        tp_json.AddMember("rear_wheel_angle",      tp.rear_wheel_angle,      alloc);
        doc.AddMember("start_point", tp_json, alloc);

        std::string payload = documentToString(doc);
        spdlog::info("[lane-changer] Publishing start point: {}", payload);
        mqtt_wrapper_->publish("aw/out/lane_change/overtake/start_point", payload);
        dds_->publish("aw/out/lane_change/overtake/start_point", payload);

        unsigned long ts2 = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        std::string ts2_str = "{\"timestamp_2\": " + std::to_string(ts2 / 1000000.0) + "}";
        mqtt_wrapper_->publish("aw/out/timestamps", ts2_str);
        dds_->publish("aw/out/timestamps", ts2_str);

    } else if (topic == "aw/in/lane_change/overtake/end_point") {
        {
            std::lock_guard<std::mutex> lock(request_mutex_);
            if (!start_point_pending_) {
                spdlog::warn("[lane-changer] Received end_point but no start request is pending — ignoring");
                return;
            }
        }

        rapidjson::Document d;
        d.Parse(message.c_str());

        if (d.HasParseError()) {
            spdlog::error("[lane-changer] JSON parse error on end_point topic");
            return;
        }
        if (!d.HasMember("approval") || !d["approval"].IsBool()) {
            spdlog::error("[lane-changer] Missing or invalid 'approval' field");
            return;
        }
        if (!d["approval"].GetBool()) {
            spdlog::warn("[lane-changer] Maneuver not approved");
            return;
        }
        if (!d.HasMember("end_point") || !d["end_point"].IsObject()) {
            spdlog::error("[lane-changer] Missing 'end_point' object");
            return;
        }
        const auto& ep = d["end_point"];
        if (!ep.HasMember("latitude")  || !ep["latitude"].IsDouble() ||
            !ep.HasMember("longitude") || !ep["longitude"].IsDouble()) {
            spdlog::error("[lane-changer] end_point missing valid latitude/longitude");
            return;
        }

        unsigned long ts3 = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        std::string ts3_str = "{\"timestamp_3\": " + std::to_string(ts3 / 1000000.0) + "}";
        mqtt_wrapper_->publish("aw/out/timestamps", ts3_str);
        dds_->publish("aw/out/timestamps", ts3_str);

        {
            std::lock_guard<std::mutex> lock(lane_change_mutex_);
            end_point_.lat = ep["latitude"].GetDouble();
            end_point_.lon = ep["longitude"].GetDouble();
        }
        {
            std::lock_guard<std::mutex> lock(request_mutex_);
            start_point_pending_ = false;
        }
        {
            std::lock_guard<std::mutex> lock(response_mutex_);
            lane_change_response_ = true;
        }

        spdlog::info("[lane-changer] Maneuver approved. End point: lat={}, lon={}",
                     end_point_.lat, end_point_.lon);

    } else if (topic == "aw/in/lane_change/direct") {
        rapidjson::Document d;
        d.Parse(message.c_str());

        if (d.HasParseError()) {
            spdlog::error("[lane-changer] JSON parse error on direct topic");
            return;
        }
        if (!d.HasMember("action") || !d["action"].IsString()) {
            spdlog::error("[lane-changer] direct message missing or invalid 'action' field");
            return;
        }

        std::string action = d["action"].GetString();
        if (action != "change") {
            spdlog::warn("[lane-changer] Unknown action on direct topic: {}", action);
            return;
        }

        if (isOvertakeActive()) {
            spdlog::warn("[lane-changer] Direct lane change blocked — overtake maneuver is in progress");
            return;
        }

        spdlog::info("[lane-changer] Direct lane change requested — toggling lane");
        latest_route_ = modifyRoute(latest_route_);
        route_pub_->publish(latest_route_);
        spdlog::info("[lane-changer] Direct lane change applied");
    }
}

bool LaneChanger::isOvertakeActive() const
{
    {
        std::lock_guard<std::mutex> lock(response_mutex_);
        if (lane_change_response_) return true;
    }
    {
        std::lock_guard<std::mutex> lock(lane_change_mutex_);
        if (lane_change_on_) return true;
    }
    return false;
}

float LaneChanger::radiansToDegrees(float radians) const
{
    return radians * (180.0f / M_PI);
}

float LaneChanger::quaternionToHeading(float x, float y, float z, float w) const
{
    tf2::Quaternion q(x, y, z, w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    float angle = (yaw < 0)
        ? radiansToDegrees(static_cast<float>(yaw) + 2.0f * M_PI)
        : radiansToDegrees(static_cast<float>(yaw));

    float heading = 90.0f - angle;
    if (heading < 0) heading += 360.0f;
    return heading;
}

std::string LaneChanger::documentToString(const rapidjson::Document& doc) const
{
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    doc.Accept(writer);
    return buffer.GetString();
}

} // namespace lane_changer
