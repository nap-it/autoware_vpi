cmake_minimum_required(VERSION 3.8)
project(tier4_external_api_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ResponseStatus.msg"
  "srv/SetEmergency.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
