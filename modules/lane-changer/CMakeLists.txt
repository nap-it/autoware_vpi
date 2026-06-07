cmake_minimum_required(VERSION 3.8)
project(lane_changer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
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
  ${CMAKE_SOURCE_DIR}/include/fastdds-cpp-wrapper/*.cxx
)

file(GLOB MQTT_SRC
  ${CMAKE_SOURCE_DIR}/include/paho-mqtt-cpp-wrapper/src/*.cpp
  ${CMAKE_SOURCE_DIR}/include/paho-mqtt-cpp-wrapper/src/*.c
)

# Create executable
add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/lane_changer.cpp
  ${FASTDDS_SRC}
  ${MQTT_SRC}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  nav_msgs
  tf2
  geometry_msgs
  autoware_planning_msgs
  autoware_auto_planning_msgs
  Fastrtps
  Fastcdr
)

target_link_libraries(${PROJECT_NAME}_node
  pthread
  fastrtps
  fastcdr
  paho-mqttpp3
  paho-mqtt3as
)

target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
