#include "objects_converter/objects_converter.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<objects_converter::ObjectsConverter>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
