#ifndef POSE_CONVERTER_HPP
#define POSE_CONVERTER_HPP

#include <memory>
#include <string>
#include <chrono>
#include <mutex>
#include <thread>
#include <cmath>
#include <numbers>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"

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

namespace pose_converter {

class PoseConverter : public rclcpp::Node {
public:
    explicit PoseConverter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~PoseConverter();

    private:
    // Configuration structure
    struct Config {
        int domain_id = 0;
        int debug_level = 0;
        double reference_latitude = 0.0;
        double reference_longitude = 0.0;
        string mqtt_host = "127.0.0.1"; 
    };

    // Vehicle state structure
    struct VehicleState {
        unsigned long int timestamp = 0;
        double latitude = 0.0;
        double longitude = 0.0;
        double altitude = 0.0;
        double heading = 0.0;
        double heading_rate = 0.0;
        double linear_acceleration = 0.0;
        double speed_val = 0.0;
        float twist_z = 0.0;
        float cov_twist_z = 0.0;
    };

    // Constants
    static constexpr double EARTH_RADIUS_M = 111320.0;
    static constexpr double M_PI_180 = M_PI / 180.0;
    static constexpr int TIMEOUT_MS = 2000;
    static constexpr int PUBLISH_RATE_MS = 100;
    static constexpr int MAX_SEQUENCE_NUMBER = 100000;

    // Member variables
    Config config_;
    VehicleState vehicle_state_;
    
    std::chrono::time_point<std::chrono::system_clock> last_pose_message_;
    bool pose_message_timeout_;
    mutable std::mutex pose_message_timeout_mutex_;
    int sequence_number_;
    
    // DDS and MQTT
    std::unique_ptr<Dds> dds_;
    std::unique_ptr<MqttWrapper> mqtt_wrapper_;
    
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kinematic_sub_;
    rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr acceleration_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;

    // Private methods
    void loadConfiguration();
    void setupDDS();
    void setupMQTT();
    void setupSubscribers();
    void setupPublishTimer();
    
    // Callback methods
    void kinematicCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void accelCallback(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg);
    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    void publishTimerCallback();
    
    // Utility methods
    float radiansToDegrees(float radians) const;
    float quaternionToHeading(float x, float y, float z, float w) const;
    std::string documentToString(const rapidjson::Document& doc) const;
    void convertCoordinates(double x, double y, double& latitude, double& longitude) const;
    bool isPoseMessageValid() const;
    
    // DDS callback (unused static)
    static void onMessageDDS(const std::string& topic, const std::string& message);
    };

} // namespace pose_converter

#endif // POSE_CONVERTER_HPP