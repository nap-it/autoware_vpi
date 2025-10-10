#include "pose_converter/pose_converter.hpp"

namespace pose_converter {

    PoseConverter::PoseConverter(const rclcpp::NodeOptions &options)
        : Node("pose_converter", options),
          last_pose_message_(std::chrono::system_clock::now()),
          pose_message_timeout_(false),
          sequence_number_(0)
    {

        spdlog::info("[pose-converter] Initializing Pose Converter Node");

        try {
            loadConfiguration();
            setupDDS();
            setupMQTT();
            setupSubscribers();
            setupPublishTimer();

            spdlog::info("[pose-converter] Pose Converter Node initialized successfully");
        } catch (const std::exception &e) {
            spdlog::error("[pose-converter] Failed to initialize Pose Converter: {}", e.what());
            throw;
        }
    }

    PoseConverter::~PoseConverter() {
        spdlog::info("[pose-converter] Shutting down Pose Converter Node");

        if (publish_timer_) {
            publish_timer_->cancel();
        }

        // Clean up DDS and MQTT connections
        dds_.reset();
        mqtt_wrapper_.reset();
    }

    void PoseConverter::loadConfiguration() {
        std::string config_path = "/pose-converter/config.ini";

        INIReader reader(config_path);

        if (reader.ParseError() < 0) {
            spdlog::warn("[pose-converter] Can't load config file from {}, using defaults", config_path);
            return;
        }

        config_.domain_id = reader.GetInteger("pose-converter", "dds_domain_id", 0);
        config_.debug_level = reader.GetInteger("pose-converter", "debug", 0);
        config_.reference_latitude = reader.GetReal("pose-converter", "reference_latitude", 0.0);
        config_.reference_longitude = reader.GetReal("pose-converter", "reference_longitude", 0.0);
        config_.mqtt_host = reader.Get("pose-converter", "mqtt_host", "127.0.0.1");

        spdlog::info("[pose-converter] Configuration loaded:");
        spdlog::info("[pose-converter] DDS Domain ID: {}", config_.domain_id);
        spdlog::info("[pose-converter] Debug Level: {}", config_.debug_level);
        spdlog::info("[pose-converter] Reference Latitude: {}", config_.reference_latitude);
        spdlog::info("[pose-converter] Reference Longitude: {}", config_.reference_longitude);
        spdlog::info("[pose-converter] MQTT Host: {}", config_.mqtt_host);

        // Set logging level based on debug configuration
        if (config_.debug_level > 0) {
            spdlog::set_level(spdlog::level::debug);
        } else {
            spdlog::set_level(spdlog::level::info);
        }
    }

    void PoseConverter::setupDDS() {
        try {
            dds_ = std::make_unique<Dds>("PoseConverter", config_.domain_id, onMessageDDS);
            dds_->provision_publisher("aw/out/pose");

            spdlog::info("[pose-converter] DDS Publisher setup complete - Domain ID: {}, Topic: aw/out/pose", config_.domain_id);
        } catch (const std::exception &e) {
            spdlog::error("[pose-converter] Failed to setup DDS: {}", e.what());
            throw;
        }
    }

    void PoseConverter::setupMQTT() {
        try {
            data_mqtt_server mqttInfo;
            mqttInfo.client_id = "vpi-pose-converter";
            mqttInfo.address = "tcp://" + config_.mqtt_host + ":1883";

            mqtt_wrapper_ = std::make_unique<MqttWrapper>(mqttInfo);

            // Wait for MQTT connection with timeout
            auto start_time = std::chrono::steady_clock::now();
            const auto timeout = std::chrono::seconds(10);

            while (!mqtt_wrapper_->is_connected()) {
                if (std::chrono::steady_clock::now() - start_time > timeout)
                {
                    throw std::runtime_error("MQTT connection timeout");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            spdlog::info("[pose-converter] Connected to MQTT server");
            spdlog::info("[pose-converter] MQTT setup complete - Host: {}, Topic: aw/out/pose", config_.mqtt_host);

        } catch (const std::exception &e) {
            spdlog::error("[pose-converter] Failed to setup MQTT: {}", e.what());
            throw;
        }
    }

    void PoseConverter::setupSubscribers() {
        kinematic_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/localization/kinematic_state",
            10,
            std::bind(&PoseConverter::kinematicCallback, this, std::placeholders::_1));

        acceleration_sub_ = this->create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
            "/localization/acceleration",
            10,
            std::bind(&PoseConverter::accelCallback, this, std::placeholders::_1));

        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf",
            10,
            std::bind(&PoseConverter::tfCallback, this, std::placeholders::_1));

        spdlog::info("[pose-converter] Subscribers initialized");
    }

    void PoseConverter::setupPublishTimer() {
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&PoseConverter::publishTimerCallback, this));

        spdlog::info("[pose-converter] Publish timer initialized (rate: {} ms)", PUBLISH_RATE_MS);
    }

    void PoseConverter::kinematicCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        {
            std::lock_guard<std::mutex> lock(pose_message_timeout_mutex_);
            pose_message_timeout_ = false;
            last_pose_message_ = std::chrono::system_clock::now();
        }

        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        // Convert coordinates based on configuration
        convertCoordinates(x, y, vehicle_state_.latitude, vehicle_state_.longitude);

        // Extract speed and ensure it's positive
        vehicle_state_.speed_val = std::abs(msg->twist.twist.linear.x);

        // Extract angular velocity and covariance
        vehicle_state_.twist_z = msg->twist.twist.angular.z;
        if (msg->twist.covariance.size() > 35) {
            vehicle_state_.cov_twist_z = msg->twist.covariance[35];
        }

        spdlog::debug("[pose-converter] Kinematic data updated - Lat: {}, Lon: {}, Speed: {}",
                      vehicle_state_.latitude, vehicle_state_.longitude, vehicle_state_.speed_val);
    }

    void PoseConverter::accelCallback(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg) {
        vehicle_state_.linear_acceleration = std::round(msg->accel.accel.linear.x * 100.0) / 100.0;
        spdlog::debug("[pose-converter] Acceleration updated: {}", vehicle_state_.linear_acceleration);
    }

    void PoseConverter::tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        if (!msg->transforms.empty()) {
            const auto &transform = msg->transforms[0].transform;
            vehicle_state_.heading = quaternionToHeading(
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w);

            spdlog::debug("[pose-converter] Heading updated: {}", vehicle_state_.heading);
        }
    }

    void PoseConverter::publishTimerCallback() {
        // Check for pose message timeout
        auto elapsed_time = std::chrono::system_clock::now() - last_pose_message_;

        {
            std::lock_guard<std::mutex> lock(pose_message_timeout_mutex_);
            if (elapsed_time > std::chrono::milliseconds(TIMEOUT_MS)) {
                if (!pose_message_timeout_) {
                    spdlog::warn("[pose-converter] Pose message timeout detected");
                    pose_message_timeout_ = true;
                }
            } else {
                pose_message_timeout_ = false;
            }
        }

        // Only publish if we have valid pose data
        if (!isPoseMessageValid()) {
            return;
        }

        try {
            // Create JSON message
            rapidjson::Document pose;
            pose.SetObject();
            auto allocator = pose.GetAllocator();

            pose.AddMember("sequenceNumber", sequence_number_, allocator);
            pose.AddMember("latitude", vehicle_state_.latitude, allocator);
            pose.AddMember("longitude", vehicle_state_.longitude, allocator);
            pose.AddMember("speed", vehicle_state_.speed_val, allocator);
            pose.AddMember("acceleration", vehicle_state_.linear_acceleration, allocator);
            pose.AddMember("heading", vehicle_state_.heading, allocator);
            pose.AddMember("heading_rate", vehicle_state_.heading_rate, allocator);
            pose.AddMember("twist_ang_z", static_cast<double>(vehicle_state_.twist_z), allocator);
            pose.AddMember("cov_twist_ang_z", static_cast<double>(vehicle_state_.cov_twist_z), allocator);

            std::string pose_str = documentToString(pose);

            // Update sequence number
            sequence_number_++;
            if (sequence_number_ > MAX_SEQUENCE_NUMBER) {
                sequence_number_ = 0;
            }

            // Publish to DDS and MQTT
            dds_->publish("aw/out/pose", pose_str);
            mqtt_wrapper_->publish("aw/out/pose", pose_str);

            spdlog::debug("[pose-converter] Published pose message: {}", pose_str);
        }
        catch (const std::exception &e) {
            spdlog::error("[pose-converter] Failed to publish pose message: {}", e.what());
        }
    }

    float PoseConverter::radiansToDegrees(float radians) const {
        return radians * (180.0f / M_PI);
    }

    float PoseConverter::quaternionToHeading(float x, float y, float z, float w) const {
        tf2::Quaternion q(x, y, z, w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;

        m.getRPY(roll, pitch, yaw);

        float converted_angle;
        if (yaw < 0) {
            converted_angle = radiansToDegrees(yaw + (2 * M_PI));
        } else {
            converted_angle = radiansToDegrees(yaw);
        }

        float heading_angle = 90.0f - converted_angle;
        if (heading_angle < 0) {
            heading_angle += 360.0f;
        }

        return heading_angle;
    }

    std::string PoseConverter::documentToString(const rapidjson::Document &doc) const {
        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        doc.Accept(writer);
        return buffer.GetString();
    }

    void PoseConverter::convertCoordinates(double x, double y, double &latitude, double &longitude) const {
        // Standard mode: use reference point conversion
        latitude = config_.reference_latitude + (y / EARTH_RADIUS_M);
        longitude = config_.reference_longitude +
                    (x / (EARTH_RADIUS_M * std::cos(config_.reference_latitude * M_PI_180)));
    }

    bool PoseConverter::isPoseMessageValid() const {
        std::lock_guard<std::mutex> lock(pose_message_timeout_mutex_);
        return !pose_message_timeout_;
    }

    void PoseConverter::onMessageDDS(const std::string &topic, const std::string &message) {
        // Static callback for DDS messages
    }

} // namespace pose_converter