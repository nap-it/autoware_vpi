#include "lane_changer/lane_changer.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<lane_changer::LaneChanger>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
