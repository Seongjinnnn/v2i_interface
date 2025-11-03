# v2i_interface

V2I to Autoware interface package for converting SPaT (Signal Phase and Timing) messages to Autoware virtual traffic light states.

## Overview

This package provides an interface between V2I communication systems (its_ros) and Autoware's Virtual Traffic Light module. It converts SPaT messages received from roadside infrastructure (RSU) via an OBU into Autoware-compatible virtual traffic light states.

## Features

- **Real-time SPaT to VTL conversion**: Converts J2735 SPaT messages to Autoware virtual traffic light states
- **Signal timeout handling**: Automatically removes stale signals after configurable timeout
- **Multi-intersection support**: Handles multiple signal groups simultaneously
- **Fail-safe design**: Defaults to stop on unknown or invalid signal states

## Node: v2i_interface_node

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/signal_info` (default) | `its_ros_msgs/msg/SPaTMsgs` | SPaT messages from RSU via OBU |

### Published Topics

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `/awapi/tmp/virtual_traffic_light_states` (default) | `tier4_v2x_msgs/msg/VirtualTrafficLightStateArray` | 10 | Virtual traffic light states for Autoware |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input_topic` | string | `/signal_info` | Input SPaT topic name |
| `output_topic` | string | `/awapi/tmp/virtual_traffic_light_states` | Output VTL topic name |
| `virtual_traffic_light_type` | string | `V2I_RSU` | Type identifier for VTL |
| `signal_timeout_sec` | double | `5.0` | Max time without signal before considering invalid |
| `default_to_stop` | bool | `true` | Default to stop when signal unknown |

## Message Mapping

### SPaT Event State (J2735) to Approval

| Event State | Name | Approval |
|-------------|------|----------|
| 5 | permissive-Movement-Allowed (green) | **true** |
| 6 | protected-Movement-Allowed (green arrow) | **true** |
| 0, 1, 2, 3, 4, 7, 8, 9 | All other states | **false** |

## Usage

### Building

```bash
cd ~/your_workspace
colcon build --packages-select v2i_interface
```

### Running

```bash
# Source workspace
source ~/your_workspace/install/setup.bash

# Launch with default parameters
ros2 launch v2i_interface v2i_interface.launch.xml

# Or run node directly
ros2 run v2i_interface v2i_interface_node
```

### Testing

```bash
# Terminal 1: Launch the interface node
ros2 launch v2i_interface v2i_interface.launch.xml

# Terminal 2: Publish test SPaT message
ros2 topic pub /signal_info its_ros_msgs/msg/SPaTMsgs "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, signal_group_id: 7, event_state: 5, min_end_time: 2550}" --once

# Terminal 3: Monitor output
ros2 topic echo /awapi/tmp/virtual_traffic_light_states
```

## System Integration

```
┌─────────────┐
│     RSU     │
└──────┬──────┘
       │ V2I
       ▼
┌─────────────┐
│   its_ros   │ /signal_info
│  (OBU pkg)  ├──────────────────┐
└─────────────┘                  │
                                 ▼
                        ┌────────────────┐
                        │ v2i_interface  │
                        └────────┬───────┘
                                 │ /awapi/tmp/virtual_traffic_light_states
                                 ▼
                        ┌────────────────┐
                        │    Autoware    │
                        │ VTL Planner    │
                        └────────────────┘
```

## Dependencies

- ROS 2 Humble or later
- its_ros_msgs
- tier4_v2x_msgs (from Autoware)
- rclcpp
- rclcpp_components

## License

Apache-2.0

## Author

Jeong Seongjin

## See Also

- [Autoware Virtual Traffic Light Module](https://autowarefoundation.github.io/autoware.universe/latest/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/)
- SAE J2735: DSRC Message Set Dictionary
