#include "pose_converter/pose_converter.hpp"

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<pose_converter::PoseConverter>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}