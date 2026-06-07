# Overview
A main problem that exists in autonomous driving is how to integrate V2X communications with the vehicle autonomous control, Autoware. This is difficult because of the different nature of each approach, one through brokers for the V2X messaging, such as MQTT, DDS, and Zenoh, and the other through ROS for the autonomous control.

This project implements a **Vehicle Programming Interface (VPI)** that connects Autoware's ROS topics to external systems such as MQTT, DDS, and Zenoh, enabling seamless data exchange and V2X messaging without requiring deep knowledge of Autoware's internal stack.

VPIs are designed to manage various aspects of vehicle hardware, data, computation, services, and system management for autonomous vehicles. They were first introduced in Wu BF, Zhong R, Wang Y, et al. VPI: Vehicle Programming Interface for Vehicle Computing. Journal of Computer Science and Technology 39(1): 22–44, Jan. 2024. DOI: https://10.1007/s11390-024-4035-2.

This work has been published in João Amaral, Andreia Figueiredo, Pedro Rito, Susana Sargento, Cooperative V2X Communications and Sensing for Autonomous Mobility, 2025 IEEE 3rd International Conference on Mobility, Operations, Services and Technologies (MOST), Delaware (USA), May 2025. DOI: https://10.1109/MOST65065.2025.00033.

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

[![PIXKIT + V2X Collective Perception](https://img.youtube.com/vi/puLR2LU472M/0.jpg)](https://www.youtube.com/watch?v=puLR2LU472M)

###### Braking Service
The Braking Service module is a specialized addition to the VPI, designed to **interface** with the **emergency braking functionality** of the Autoware framework. It works by processing incoming braking messages from **MQTT** and **DDS** domains and calling the **internal Autoware emergency braking service** based on the provided input.<br />

[![PIXKIT + V2X Emergency Braking](https://img.youtube.com/vi/zbTc_y0SD8g/0.jpg)](https://www.youtube.com/watch?v=zbTc_y0SD8g)

###### Lane Changer
The Lane Changer provides direct external influence over Autoware's motion planning, enabling the integration of cooperative maneuvering solutions with Autoware's autonomous driving capabilities. It supports two operational modes: **pre-planned overtaking maneuvers** and **single lane changes**. A pre-planned overtaking maneuver is triggered through dedicated **MQTT** or **DDS** messages containing the positional information of the desired overtaking location, allowing the vehicle to execute the maneuver at a predefined point. In contrast, a single lane change command initiates a lane change as soon as possible, without requiring additional positioning data.<br />

[![PIXKIT + MCS](https://img.youtube.com/vi/luE-kF-ow1g/0.jpg)](https://www.youtube.com/watch?v=luE-kF-ow1g)

# Deployment
This VPI is deployed through a Docker container that contains all the necessary dependencies and configurations to run the VPI modules. Follow the steps below to deploy the VPI:
1. Clone the repository:

```
git clone https://github.com/nap-it/autoware_vpi.git
```

2. Go to the cloned directory:

```
cd autoware_vpi
```

3. Run the following command to build the Docker image:

```
docker build -t autoware_vpi:latest .
```

4. Start the Docker container with the following command:

```
docker compose up -d
```

5. Access the Docker container's logs with:

```
docker compose logs -f
```

# How to configure the VPI

- The ```start_all.sh``` script is used to launch all the VPI internal modules in the background when the Docker container starts. By default, all modules are enabled. This is where you can edit the ***ROS_DOMAIN_ID*** and ***RMW_IMPLEMENTATION*** that connects to Autoware.

- The ```config.ini``` file is used to configure each of the VPI's internal modules. By default, all modules are configured to connect to DDS domain ID 0 and MQTT host at ```127.0.0.1```.

- For the [pose-converter]:
```
[pose-converter]
dds_domain_id=0
debug=0
reference_latitude=0.0			; autoware map origin latitude
reference_longitude=0.0			; autoware map origin longitude
mqtt_host=127.0.0.1             ; mqtt host to publish messages
```
  - For the [objects-converter]:
```
[objects-converter]
dds_domain_id=0
debug=0
reference_latitude=0.0				; autoware map origin latitude
reference_longitude=0.0				; autoware map origin longitude
ignore_unknown_objects=true		    ; don't publish objects with unknown classification
mqtt_host=127.0.0.1                 ; mqtt host to publish messages
```

 - For the [braking-service]:
```
[braking-service]
dds_domain_id=0
debug=0
mqtt_host=127.0.0.1                 ; mqtt host to subcribe to messages
```

- For the [lane-changer]:
```
[lane-changer]
dds_domain_id=0
debug=0
mqtt_host=127.0.0.1               ; mqtt host to subscribe to messages
reference_latitude=0.0            ; autoware map origin latitude
reference_longitude=0.0           ; autoware map origin longitude
distance_to_start=3.0             ; proximity threshold (meters) to trigger each overtake point
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

Note: The latitude and longitude are calculated based on the reference position defined in the configuration file on the parameters ```reference_latitude``` and ```reference_longitude```. This reference position should match the origin of the Autoware map being used.
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

- To send a single lane change request to Autoware, publish the following JSON message to the DDS or MQTT topic ***aw/in/lane_change/direct***:
```
{
  "action": "change"
}
```
For example, run the following command in the terminal to send a single lane change request while Autoware and the VPI are running:
```
mosquitto_pub -t 'aw/in/lane_change/direct' -m '{"action": "change"}'
```

For cooperative overtake mode and the full message reference, see the [lane-changer module README](modules/lane-changer/README.md).

## Acknowledgements
We thank [I2CAT](https://i2cat.net) for their valuable feedback and contributions to the development and documentation of this VPI.

## License

autoware_vpi is licensed under LGPLv3, see [license file](LICENSE.md) for details.
