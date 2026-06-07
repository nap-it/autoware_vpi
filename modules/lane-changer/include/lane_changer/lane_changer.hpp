#ifndef LANE_CHANGER_HPP
#define LANE_CHANGER_HPP

#include <memory>
#include <string>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <spdlog/spdlog.h>

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "autoware_planning_msgs/msg/lanelet_primitive.hpp"
#include "autoware_planning_msgs/msg/lanelet_segment.hpp"
#include "autoware_planning_msgs/msg/lanelet_route.hpp"

#include "config_reader.hpp"
#include "fastdds-cpp-wrapper/dds.hpp"
#include "mqttwrapper.h"

namespace lane_changer {

class LaneChanger : public rclcpp::Node {
public:
    explicit LaneChanger(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~LaneChanger();

private:
    using LaneletRoute    = autoware_planning_msgs::msg::LaneletRoute;
    using LaneletSegment  = autoware_planning_msgs::msg::LaneletSegment;
    using Odometry        = nav_msgs::msg::Odometry;
    using Trajectory      = autoware_auto_planning_msgs::msg::Trajectory;
    using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;
    using Pose            = geometry_msgs::msg::Pose;

    struct Config {
        int         domain_id         = 0;
        int         debug             = 0;
        std::string mqtt_host         = "127.0.0.1";
        double      ref_lat           = 40.632263;
        double      ref_lon           = -8.629927;
        double      distance_to_start = 3.0;
    };

    struct EndPoint {
        double lat = 0.0;
        double lon = 0.0;
    };

    struct TrajectoryPointData {
        double x, y, lat, lon, alt, heading;
        double orientation_x, orientation_y, orientation_z, orientation_w;
        double longitudinal_velocity, lateral_velocity, acceleration;
        double heading_rate, front_wheel_angle, rear_wheel_angle;
    };

    static constexpr double EARTH_RADIUS_M = 111320.0;
    static constexpr double M_PI_180       = M_PI / 180.0;
    static constexpr int    POINT_OFFSET   = 100;

    Config   config_;
    EndPoint end_point_;
    Pose     start_pose_;

    LaneletRoute latest_route_;
    Trajectory   latest_trajectory_;

    mutable std::mutex request_mutex_;
    mutable std::mutex response_mutex_;
    mutable std::mutex lane_change_mutex_;

    bool start_point_pending_  = false;
    bool lane_change_response_ = false;
    bool lane_change_on_       = false;

    std::unique_ptr<Dds>         dds_;
    std::unique_ptr<MqttWrapper> mqtt_wrapper_;

    rclcpp::Subscription<LaneletRoute>::SharedPtr route_sub_;
    rclcpp::Subscription<Odometry>::SharedPtr     pose_sub_;
    rclcpp::Subscription<Trajectory>::SharedPtr   trajectory_sub_;
    rclcpp::Publisher<LaneletRoute>::SharedPtr    route_pub_;

    void loadConfiguration();
    void setupDDS();
    void setupMQTT();
    void setupSubscribers();

    void poseCallback(const Odometry::SharedPtr msg);
    void trajectoryCallback(const Trajectory::SharedPtr msg);
    void routeCallback(const LaneletRoute::SharedPtr msg);

    LaneletRoute modifyRoute(LaneletRoute route);

    float       radiansToDegrees(float radians) const;
    float       quaternionToHeading(float x, float y, float z, float w) const;
    std::string documentToString(const rapidjson::Document& doc) const;

    void onMessage(const std::string& topic, const std::string& message);
    bool isOvertakeActive() const;
};

} // namespace lane_changer

#endif // LANE_CHANGER_HPP
