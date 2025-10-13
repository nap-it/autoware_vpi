# Overview
This project implements a **Vehicle Programming Interface (VPI)** that connects Autoware's ROS topics to external systems such as MQTT, DDS, and Zenoh, enabling seamless data exchange and V2X messaging without requiring deep knowledge of Autoware's internal stack.

VPIs are designed to manage various aspects of vehicle hardware, data, computation, services, and system management for autonomous vehicles. They were first introduced in Wu BF, Zhong R, Wang Y, et al. VPI: Vehicle Programming Interface for Vehicle Computing. Journal of Computer Science and Technology 39(1): 22–44, Jan. 2024. DOI: https://10.1007/s11390-024-4035-2.

This VPI enables:
- Gathering detailed status updates on the **vehicle’s internal state**, such as position, speed, and **detected objects**, and makes this information available for use by external systems.
- **External commands** to influence the vehicle’s behavior by **modifying the control stack**, ensuring that real-time data from external sources can be processed and acted upon.

This VPI was developed and tested using the [PIXKIT 2023.04](https://github.com/pixmoving-moveit/Autoware/tree/release/pixkit-2023.04) release of Autoware.

If you find this code useful in your research, please consider citing:

    @INPROCEEDINGS{11071413,
  		author={Amaral, João and Figueiredo, Andreia and Rito, Pedro and Sargento, Susana},
  		booktitle={2025 IEEE 3rd International Conference on Mobility, Operations, Services and Technologies (MOST)}, 
  		title={Cooperative V2X Communications and Sensing for Autonomous Mobility}, 
  		year={2025},
  		volume={},
  		number={},
  		pages={229-240},
  		keywords={Scalability;Urban areas;Microservice architectures;Transportation;Sensors;Safety;Vehicle dynamics;Low latency communication;Vehicle-to-everything;Autonomous vehicles;Autonomous Mobility;V2X;DDS;SDV;Microservices},
  		doi={10.1109/MOST65065.2025.00033}}


# Internal implementation
The VPI is designed to work in environments with Autoware, containing the following modules:<br />
###### Pose Converter
This module is designed to process information about the vehicle’s pose, including the **position**, **orientation** and **movement metrics**, transforming it into a standardized format that can be used by **external services**.<br />

###### Objects Converter
This module is responsible for converting the data produced by the vehicle’s **perception systems** into a standardized format that can be shared with **external systems** .<br />

###### Braking Service
The Braking Service module is a specialized addition to the VPI, designed to **interface** with the **emergency braking functionality** of the Autoware framework. It works by processing incoming braking messages from **MQTT** and **DDS** domains and calling the **internal Autoware emergency braking service** based on the provided input.<br />

# Deployment
1. Create a new directory:

```
mkdir ~/vpi
```

2. Create a new **docker-compose.yml** file containing the necessary VPI service and the the config.ini volumes.

```
services:
  vpi:
    image: vpi:latest
    container_name: vpi
    volumes:
      - ./config.ini:/pose_converter/config.ini
      - ./config.ini:/objects_converter/config.ini
      - ./config.ini:/braking_service/config.ini
	  - ./start_all.sh:/start_all.sh
    network_mode: host
    ipc: host
    command: ["/bin/bash", "-c" , "/start_all.sh"]
    tty: true
```
Create the ```start_all.sh``` file in the same directory as ```docker-compose.yml``` file with the following content (this is where you can edit the ***ROS_DOMAIN_ID*** and ***RMW_IMPLEMENTATION*** that connects to Autoware and disable specific internal modules):

```
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


```

 4. Each of the VPI's internal modules can be configured by the file ```config.ini```:
 - For the [pose-converter], the configuration is the following:
```
[pose-converter]
dds_domain_id=0
debug=0
reference_latitude=0.0			; autoware map reference latitude
reference_longitude=0.0			; autoware map reference longitude
mqtt_host=127.0.0.1             ; mqtt host to publish messages
```
 - For the [objects-converter], the configuration is the following:
```
[objects-converter]
dds_domain_id=0
debug=0
reference_latitude=0.0				; autoware map reference latitude
reference_longitude=0.0				; autoware map reference longitude
ignore_unknown_objects=true		    ; don't publish objects with unknown classification
mqtt_host=127.0.0.1                 ; mqtt host to publish messages
```

 - For the [braking-service], the configuration is the following:
```
[braking-service]
dds_domain_id=0
debug=1
mqtt_host=127.0.0.1                 ; mqtt host to publish messages
```

 5. Launch the VPI:
```
cd ~/vpi
docker compose up -d
```
# How to interact with the VPI

 - To gather the vehicle's pose information, subscribe to the DDS/MQTT topic ***aw/out/pose*** which contains the following information: 
```
{
	sequenceNumber: int,		// for messate integrity
	latitude: float,
	longitude; float,
	speed: float,
	acceleration: float,
	heading: float,
	heading_rate: float,
	twist_ang_z: float,			// angular velocity
	cov_twist_ang_z: float		// angular velocity covariance
}
```
 - To gather information about the vehicle's detected objects, subscribe to the DDS/MQTT topic ***aw/out/objects*** which contains the following information: 
```
{
	sequenceNumber: int,        // for messate integrity
	objects: [
		{
			heading: float,
			cov_heading: float,
			latitude: float,
			longitude; float,
			speed: float,
			cov_speed: float,
			sensorID: int,
			objID: int,
			size_x: float,
			size_y: float,
			size_z: float,
			x: float,			// distance in meters to the reference position in relation to true North
			y: float,			// distance in meters to the reference position in relation to true East
			z: float,			// altitude in meters
			cov_x: float,
			cov_y: float,
			cov_z: float,
			twist_ang_z: float,			
			cov_twist_ang_z: float,
			classification: int,
			confidence: int
		},
		(...)
	]
}
```

- To call the Autoware's internal emergency braking service through the VPI, publish the following message to the DDS or MQTT topic ***aw/in/brake***:
```
{
	brake: boolean,		 // true to engage the emergency braking, false to resume normal operations
}
```
For example, run the following command in the terminal to engage the emergency braking while Autoware and the VPI are running:
```
mosquitto_pub -t 'aw/in/brake' -m '{"brake":true}'
```

## License

autoware_vpi is licensed under LGPLv3, see [license file](LICENSE.md) for details.
