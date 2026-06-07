# Lane Changer

ROS2 node that performs lane change maneuvers by modifying the Autoware route in real time. Two independent modes are supported: a **cooperative overtake** mode that coordinates with an external system and a cooperating vehicle, and a **direct** mode that immediately toggles the lane on demand.

## How it works

### Route interception

When Autoware computes a route (after a goal pose is set), it publishes a `LaneletRoute` message on `/planning/mission_planning/route`. This message contains a list of **segments**, where each segment has a list of `primitives` (the candidate lanes) and a `preferred_primitive` (the lane Autoware will actually plan through).

The lane changer subscribes to this topic and stores the latest route. When a lane change is triggered (by either mode), it republishes a **modified version of that same route** back onto the same topic (`/planning/mission_planning/route`) with `transient_local` QoS, so Autoware picks it up as the new active route. The only modification is toggling `preferred_primitive` on every segment — if it was `primitives[0]`, it becomes `primitives[1]`, and vice versa. This causes Autoware's planning stack to replan through the alternate lane.

For this to work the map must have segments with **two lane primitives each**. The node does not add or remove lanes; it only switches which one is treated as preferred.

### Overtake mode

A multi-step cooperative flow with an external system and a cooperating vehicle. The maneuver is triggered at a specific geographic location and reverts at another.

1. An external system sends a `start` trigger. The node takes trajectory point `[100]` from the current planned trajectory as the **start point** and publishes it.
2. The external system processes the start point, negotiates the execution of an overtake maneuver with a cooperating vehicle, and sends back an **end point** (lat/lon), representing the expected position where the vehicle should return to its original lane, along with the maneuver coordination result (either an approval, or a rejection).
3. As the vehicle drives:
   - When it comes within `distance_to_start` meters of the start point, the modified route is published → Autoware replans through the alternate lane.
   - When it comes within `distance_to_start` meters of the end point, the route is toggled back → Autoware replans through the original lane.

### Direct mode

A single-message toggle with no negotiation or position tracking. Sending `{"action": "change"}` to `aw/in/lane_change/direct` immediately modifies the route and Autoware replans. Sending it again toggles back. This mode is **blocked while an overtake is in progress** to prevent conflicting route modifications.

---

## Important usage notes

- **The VPI (including this module) must be running before the goal pose is set.** The node subscribes to `/planning/mission_planning/route` to capture the route when Autoware first computes it. If the module starts after the route is already active, it will have no route stored and the lane change will not work.

- **Overtake: `start` must be sent before `end_point`.** The node enforces message ordering: if an `end_point` message arrives without a prior `start`, it is dropped with a warning log. The correct sequence is always: receive `start` → send `start_point` → receive `end_point` with approval.

- **Direct mode is blocked during an overtake.** If a direct change is requested while an overtake maneuver is pending or active, it is dropped with a warning log. Wait for the overtake to complete first.

- **Fine control over movement is not supported**. Even when requesting a direct lane change, the request is treated as a suggestion rather than a mandatory command. Autoware's planning modules remain responsible for assessing the vehicle's environment, determining whether the maneuver is achievable, and selecting the appropriate location for its execution. As a result, lane changes are not performed immediately and may be delayed or rejected altogether if current road conditions do not permit its execution.

---

## Configuration (`src/config.ini`)

```ini
[lane-changer]
dds_domain_id       = 0          # FastDDS domain ID
debug               = 0          # Set to 1 to enable debug logging (see below)
mqtt_host           = 127.0.0.1
reference_latitude  = 0.0        # Origin of the local Cartesian frame (degrees)
reference_longitude = 0.0        # Origin of the local Cartesian frame (degrees)
distance_to_start   = 3.0        # Proximity threshold to trigger each overtake point (meters)
```

### Debug logging

By default (`debug=0`) the node logs at `info` level. High-frequency callbacks — pose proximity checks, route updates, and received messages — are logged at `debug` level and are suppressed unless `debug=1` is set. 

Set `debug=1` when you need to trace the distance-to-start/end-point values in real time or diagnose message routing issues.

---

## Message Reference

### Inbound

#### `aw/in/lane_change/overtake/request`
Plain string. Initiates the overtake sequence.

```
start
```

| Value | Meaning |
|---|---|
| `start` / `START` | Request the start point for a cooperative overtake maneuver |

---

#### `aw/in/lane_change/overtake/end_point`
JSON. Sent after the cooperating vehicle approves the maneuver. Contains the geographic position where the vehicle should return to the original lane.

```json
{
  "approval": true,
  "end_point": {
    "latitude": 40.62841131318927,
    "longitude": -8.654421743394836
  }
}
```

| Field | Type | Description |
|---|---|---|
| `approval` | bool | Whether the cooperating vehicle approved the maneuver. If `false`, the message is ignored and no lane change occurs. |
| `end_point.latitude` | double | WGS84 latitude of the position where the vehicle should return to the original lane. |
| `end_point.longitude` | double | WGS84 longitude of the position where the vehicle should return to the original lane. |

---

#### `aw/in/lane_change/direct`
JSON. Immediately toggles the preferred lane. Each message is a toggle — send once to change, send again to revert. Blocked while an overtake is in progress.

```json
{ "action": "change" }
```

| Field | Type | Description |
|---|---|---|
| `action` | string | Must be `"change"`. Toggles the preferred lane on all route segments immediately. |

---

### Outbound

#### `aw/out/lane_change/overtake/start_point`
JSON. Published immediately after receiving a valid overtake `start` request. Contains the full state of the trajectory point that will trigger the lane change, in both local Cartesian and WGS84 coordinates.

```json
{
  "start_point": {
    "x": 5.967,
    "y": 0.533,
    "latitude": 40.62841131318927,
    "longitude": -8.654421743394836,
    "altitude": -1.638,
    "heading": 155.07,
    "orientation": {
      "x": -0.00074,
      "y": -0.00117,
      "z": -0.53785,
      "w": 0.84303
    },
    "longitudinal_velocity": 2.777,
    "lateral_velocity": 0.0,
    "acceleration": 0.0,
    "heading_rate": 0.0,
    "front_wheel_angle": 0.0,
    "rear_wheel_angle": 0.0
  }
}
```

| Field | Type | Description |
|---|---|---|
| `x` | double | Local Cartesian X position (meters), offset from the reference origin. |
| `y` | double | Local Cartesian Y position (meters), offset from the reference origin. |
| `latitude` | double | WGS84 latitude of the start point. |
| `longitude` | double | WGS84 longitude of the start point. |
| `altitude` | double | Altitude in meters. |
| `heading` | double | Compass heading in degrees (0 = North, 90 = East, clockwise). |
| `orientation.x/y/z/w` | double | Quaternion representing the vehicle's orientation at the start point. |
| `longitudinal_velocity` | double | Forward velocity at the start point (m/s). |
| `lateral_velocity` | double | Lateral velocity at the start point (m/s). |
| `acceleration` | double | Longitudinal acceleration at the start point (m/s²). |
| `heading_rate` | double | Rate of change of heading (rad/s). |
| `front_wheel_angle` | double | Front wheel steering angle (rad). |
| `rear_wheel_angle` | double | Rear wheel steering angle (rad). |

---

#### `aw/out/timestamps`
JSON. Published at each key step of the overtake maneuver for latency measurement. Three timestamps are emitted per full overtake cycle. Not used by the direct mode.

```json
{ "timestamp_1": 1743769658.587550 }
{ "timestamp_2": 1743769658.591234 }
{ "timestamp_3": 1743769658.612345 }
```

| Field | Description |
|---|---|
| `timestamp_1` | Unix timestamp (seconds) when the overtake `start` request was received. |
| `timestamp_2` | Unix timestamp (seconds) when the start point was published. |
| `timestamp_3` | Unix timestamp (seconds) when the end point approval was received. |

---

## Topic Summary

| Direction | Topic | Transport | Format | Mode |
|---|---|---|---|---|
| IN  | `aw/in/lane_change/overtake/request`    | MQTT + DDS | Plain string | Overtake |
| OUT | `aw/out/lane_change/overtake/start_point` | MQTT + DDS | JSON | Overtake |
| IN  | `aw/in/lane_change/overtake/end_point`  | MQTT + DDS | JSON | Overtake |
| IN  | `aw/in/lane_change/direct`              | MQTT + DDS | JSON | Direct |
| OUT | `aw/out/timestamps`                     | MQTT + DDS | JSON | Overtake |
