#!/bin/bash

# Start all programs in the background
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash

# -- Pose Converter --
source /pose-converter/install/setup.bash
ros2 run pose_converter pose_converter_node & 

# -- Objects Converter --
source /objects-converter/install/setup.bash
ros2 run objects_converter objects_converter_node &

# -- Braking Service --
source /braking-service/include/tier4_external_api_msgs/install/setup.bash
source /braking-service/install/setup.bash
ros2 run braking_service braking_service_node &

# Wait for all background processes to complete
wait
