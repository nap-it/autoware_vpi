import paho.mqtt.client as mqtt
import json
import math
import time

# ================= CONFIGURATION =================
REF_LAT = 0.0
REF_LON = 0.0
EARTH_RADIUS_M = 111320.0

MQTT_BROKER = "127.0.0.1"
MQTT_PORT = 1883
MQTT_TOPIC = "/aw/in/objects"
PUBLISH_RATE_HZ = 20  
# =================================================

def get_heading_from_quaternion(x, y, z, w):
    """
    Converts Quaternion to Navigation Heading (Degrees, 0=North, CW).
    """
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_rad = math.atan2(t3, t4)
    yaw_deg = math.degrees(yaw_rad)
    heading = 100.0 - yaw_deg
    if heading < 0:
        heading += 360.0
    return heading

def main():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "python_spawner")
    
    try:
        print(f"Connecting to {MQTT_BROKER}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start() 
    except Exception as e:
        print(f"Failed to connect to MQTT: {e}")
        return

    # --- BUS CONFIGURATION ---
    size_x = 12.0
    size_y = 2.5
    size_z = 3.2
    target_z = 1.6 
    
    # Original Quaternion
    quat_x = 0.0
    quat_y = 0.0
    quat_z = 0.7709528678719133
    quat_w = 0.6368922008629656

    speed = 0.0

    # --- LOCATION: Shinjuku ---
    #lat = 0.4493830426922386
    #lon = 0.7332619363097377
    # --- LOCATION: Carla Town04---
    #lat = -0.00034145617265456276
    #lon = -0.001727918913430572
    # --- LOCATION: Carla Town01 ---
    lat = -0.000018
    lon = 0.002173
    # 1. Calculate base heading from Quaternion
    base_heading = get_heading_from_quaternion(quat_x, quat_y, quat_z, quat_w)

    # 2. ROTATE 90 DEGREES RIGHT (Clockwise)
    # We add 90 to the heading and use modulo 360 to keep it within 0-360 range.
    heading = (base_heading + 90.0) % 360.0

    sequence_number = 0

    print(f"Starting BUS spawn (Rotated +90°) at {PUBLISH_RATE_HZ} Hz.")

    try:
        while True:
            payload = {
                "sequenceNumber": sequence_number,
                "objects": [
                    {
                        "objID": 999,
                        "classification": 3,   # 3 = BUS
                        "confidence": 100,
                        "latitude": lat,
                        "longitude": lon,
                        "z": target_z,
                        "heading": heading,    # Using the rotated heading
                        "speed": speed,
                        "size_x": size_x,
                        "size_y": size_y,
                        "size_z": size_z
                    }
                ]
            }

            json_str = json.dumps(payload)
            client.publish(MQTT_TOPIC, json_str)
            
            if sequence_number % 20 == 0:
                print(f"[Seq {sequence_number}] Published BUS at Lat: {lat:.6f}, Heading: {heading:.1f}")
            
            sequence_number += 1
            if sequence_number > 65535:
                sequence_number = 0
                
            time.sleep(1.0 / PUBLISH_RATE_HZ)

    except KeyboardInterrupt:
        print("\nStopping...")
        client.loop_stop()
        client.disconnect()
        print("Disconnected.")

if __name__ == "__main__":
    main()
