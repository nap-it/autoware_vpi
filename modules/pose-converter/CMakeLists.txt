cmake_minimum_required(VERSION 3.8)
project(pose_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)

# External dependencies
find_package(Fastcdr REQUIRED)
find_package(Fastrtps REQUIRED)

# Include directories
include_directories(include)
include_directories(${CMAKE_SOURCE_DIR}/include/paho-mqtt-cpp-wrapper/include)
include_directories(${CMAKE_SOURCE_DIR}/include/fastdds-cpp-wrapper)
include_directories(${CMAKE_SOURCE_DIR}/include/rapidjson)
include_directories(${CMAKE_SOURCE_DIR}/include/spdlog/include)

# Collect external source files
file(GLOB FASTDDS_SRC
  ${CMAKE_SOURCE_DIR}/include/fastdds-cpp-wrapper/*.cpp
  ${CMAKE_SOURCE_DIR}/include/fastdds-cpp-wrapper/*.c
  ${CMAKE_SOURCE_DIR}/include/fastdds-cpp-wrapper/*.h
  ${CMAKE_SOURCE_DIR}/include/fastdds-cpp-wrapper/*.hpp
  ${CMAKE_SOURCE_DIR}/include/fastdds-cpp-wrapper/*.cxx
)

# Collect MQTT wrapper source files
file(GLOB MQTT_SRC
  ${CMAKE_SOURCE_DIR}/include/paho-mqtt-cpp-wrapper/src/*.cpp
  ${CMAKE_SOURCE_DIR}/include/paho-mqtt-cpp-wrapper/src/*.c
)

# Create executable
add_executable(${PROJECT_NAME}_node 
  src/main.cpp 
  src/pose_converter.cpp
  ${FASTDDS_SRC}
  ${MQTT_SRC}
)

# Specify dependencies
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_msgs
  autoware_auto_vehicle_msgs
  Fastrtps
  Fastcdr
)

# Link external libraries
target_link_libraries(${PROJECT_NAME}_node 
  pthread 
  fastrtps 
  fastcdr 
  paho-mqttpp3 
  paho-mqtt3as
)

# Set C++ standard
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)

# Install targets
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()