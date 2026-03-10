cmake_minimum_required(VERSION 3.8)
project(v2x_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

# Include directories
include_directories(include)
include_directories(${CMAKE_SOURCE_DIR}/include/paho-mqtt-cpp-wrapper/include)
include_directories(${CMAKE_SOURCE_DIR}/include/rapidjson)
include_directories(${CMAKE_SOURCE_DIR}/include/spdlog/include)

# Collect MQTT wrapper source files
file(GLOB MQTT_SRC
  ${CMAKE_SOURCE_DIR}/include/paho-mqtt-cpp-wrapper/src/*.cpp
)

add_executable(${PROJECT_NAME}_node 
  src/main.cpp 
  src/v2x_converter.cpp
  ${MQTT_SRC}
)

ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp 
  autoware_perception_msgs
  tf2
  tf2_geometry_msgs
  unique_identifier_msgs
)

# ... existing setup ...

target_link_libraries(${PROJECT_NAME}_node 
  pthread 
  paho-mqttpp3  # <--- Crucial
  paho-mqtt3as  # <--- Crucial
)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()