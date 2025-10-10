#include "braking_service/braking_service.hpp"

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<braking_service::BrakingService>();
    
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
