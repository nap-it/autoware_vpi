#include "v2x_converter/v2x_converter.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<v2x_converter::V2XConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}