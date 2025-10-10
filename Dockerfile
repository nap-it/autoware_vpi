# ===== Builder Stage =====
FROM ros:humble-ros-base-jammy AS builder

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libgeographic-dev \
    geographiclib-tools \
    libboost-date-time-dev \
    libboost-program-options-dev \
    libboost-system-dev \
    libcrypto++-dev \
    libasio-dev \
    libtinyxml2-dev \
    libcurl4-openssl-dev \
    libssl-dev \
    ca-certificates \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# --------- Build MQTT Libraries ---------
WORKDIR /paho.mqtt.c
RUN git clone https://github.com/eclipse/paho.mqtt.c.git . && \
    git checkout v1.3.13 && \
    cmake -Bbuild -H. \
          -DPAHO_ENABLE_TESTING=OFF \
          -DPAHO_BUILD_STATIC=ON \
          -DPAHO_WITH_SSL=ON \
          -DPAHO_HIGH_PERFORMANCE=ON && \
    cmake --build build/ --target install && \
    ldconfig

WORKDIR /paho.mqtt.cpp
RUN git clone https://github.com/eclipse/paho.mqtt.cpp.git . && \
    git checkout v1.3.2 && \
    cmake -Bbuild -H. \
          -DPAHO_BUILD_STATIC=ON \
          -DPAHO_BUILD_DOCUMENTATION=OFF \
          -DPAHO_BUILD_SAMPLES=ON && \
    cmake --build build/ --target install && \
    ldconfig

# --------- External Dependencies ---------
WORKDIR /
RUN git -c http.sslVerify=false clone https://github.com/nap-it/fastdds-cpp-wrapper.git
WORKDIR /fastdds-cpp-wrapper
RUN git checkout ros

WORKDIR /
RUN git -c http.sslVerify=false clone https://github.com/gabime/spdlog.git
WORKDIR /spdlog
RUN git checkout v1.11.0

WORKDIR /
RUN git -c http.sslVerify=false clone https://github.com/nap-it/paho.mqtt.cpp.wrapper.git
RUN mv paho.mqtt.cpp.wrapper paho-mqtt-cpp-wrapper
WORKDIR /paho-mqtt-cpp-wrapper
RUN git checkout v1.0.0

WORKDIR /
RUN git -c http.sslVerify=false clone https://github.com/Tencent/rapidjson.git
WORKDIR /rapidjson
RUN git checkout 012be8528783cdbf4b7a9e64f78bd8f056b97e24

WORKDIR /
RUN mkdir -p tier4_external_api_msgs/msg && mkdir -p tier4_external_api_msgs/srv && \
    wget -O tier4_external_api_msgs/msg/ResponseStatus.msg https://raw.githubusercontent.com/tier4/tier4_autoware_msgs/tier4/universe/tier4_external_api_msgs/msg/ResponseStatus.msg && \
    wget -O tier4_external_api_msgs/srv/SetEmergency.srv https://raw.githubusercontent.com/tier4/tier4_autoware_msgs/tier4/universe/tier4_external_api_msgs/srv/SetEmergency.srv 
    
COPY aw_ros_msgs/package.xml tier4_external_api_msgs/package.xml
COPY aw_ros_msgs/CMakeLists.txt tier4_external_api_msgs/CMakeLists.txt

# -------- Build Autoware Messages ---------
WORKDIR /
RUN git clone https://github.com/tier4/autoware_auto_msgs.git
WORKDIR /autoware_auto_msgs
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS='-w'"

WORKDIR /
RUN git clone https://github.com/tier4/autoware_msgs.git
WORKDIR /autoware_msgs
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS='-w'"

# --------- Build Pose Converter ---------
WORKDIR /
RUN mkdir /pose-converter
COPY modules/pose-converter/ /pose-converter/
RUN cp -r /fastdds-cpp-wrapper /pose-converter/include/fastdds-cpp-wrapper && \
    cp -r /rapidjson/include/rapidjson /pose-converter/include/rapidjson && \
    cp -r /spdlog /pose-converter/include/spdlog && \
    cp -r /paho-mqtt-cpp-wrapper /pose-converter/include/paho-mqtt-cpp-wrapper && \
    cp -r /autoware_auto_msgs /pose-converter/include/autoware_auto_msgs

WORKDIR /pose-converter
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && source /pose-converter/include/autoware_auto_msgs/install/setup.bash && colcon build"

# --------- Build Objects Converter ---------
WORKDIR /
RUN mkdir /objects-converter
COPY modules/objects-converter/ /objects-converter/
RUN cp -r /fastdds-cpp-wrapper /objects-converter/include/fastdds-cpp-wrapper && \
    cp -r /rapidjson/include/rapidjson /objects-converter/include/rapidjson && \
    cp -r /spdlog /objects-converter/include/spdlog && \
    cp -r /paho-mqtt-cpp-wrapper /objects-converter/include/paho-mqtt-cpp-wrapper && \
    cp -r /autoware_auto_msgs /objects-converter/include/autoware_auto_msgs

WORKDIR /objects-converter
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && source /objects-converter/include/autoware_auto_msgs/install/setup.bash && colcon build"

# --------- Build Braking Service ---------
WORKDIR /
RUN mkdir /braking-service
COPY modules/braking-service/ /braking-service/
RUN cp -r /fastdds-cpp-wrapper /braking-service/include/fastdds-cpp-wrapper && \
    cp -r /rapidjson/include/rapidjson /braking-service/include/rapidjson && \
    cp -r /spdlog /braking-service/include/spdlog && \
    cp -r /paho-mqtt-cpp-wrapper /braking-service/include/paho-mqtt-cpp-wrapper 

WORKDIR /braking-service
RUN cp -r /tier4_external_api_msgs /braking-service/include/tier4_external_api_msgs
WORKDIR /braking-service/include/tier4_external_api_msgs
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --merge-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS='-w'"

WORKDIR /braking-service
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && source /braking-service/include/tier4_external_api_msgs/install/setup.bash && colcon build"


# ===== Runner Stage =====
FROM ros:humble-ros-base-jammy AS runner

RUN apt-get update && apt-get install -y \
    ca-certificates \
    ros-humble-rmw-cyclonedds-cpp \
    libgeographic-dev \
    geographiclib-tools \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /pose-converter /pose-converter
COPY --from=builder /objects-converter /objects-converter
COPY --from=builder /braking-service /braking-service

COPY --from=builder /usr/local/lib/libpaho-mqtt3*.so* /usr/local/lib/
COPY --from=builder /usr/local/lib/libpaho-mqttpp3.so* /usr/local/lib/

RUN echo "/usr/local/lib" > /etc/ld.so.conf.d/local.conf && ldconfig
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN echo "source /opt/ros/humbl e/setup.bash" >> /root/.bashrc