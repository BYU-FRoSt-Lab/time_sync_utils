# QoS Configuration Guide for Topic Monitor

## Overview

The Topic Monitor now supports customizable QoS (Quality of Service) profiles for each monitored topic. This allows you to match the subscriber's QoS settings with the publisher's QoS to ensure proper message reception.

## Why QoS Matters

In ROS 2, QoS profiles must be compatible between publishers and subscribers for messages to be delivered. If a publisher uses `best_effort` reliability, the subscriber must also use `best_effort` (or be compatible). This feature allows the topic monitor to adapt to different publisher configurations.

## Configuration Options

### history_depth
- **Type**: Integer
- **Default**: 10
- **Description**: The depth of the message history queue. Larger values allow the subscriber to buffer more messages but use more memory.

### reliability
- **Type**: String
- **Default**: "reliable"
- **Options**:
  - `"reliable"`: Ensures message delivery (may retry). Use for critical data.
  - `"best_effort"`: Does not guarantee message delivery. Use for high-frequency sensor data where occasional loss is acceptable.

### durability
- **Type**: String
- **Default**: "volatile"
- **Options**:
  - `"volatile"`: Only subscribers that are alive at publication time receive messages.
  - `"transient_local"`: Late-joining subscribers may receive previously published messages. Use for static configuration data.

## Common Use Cases

### 1. High-Frequency Sensor Data (e.g., LiDAR)
```yaml
- name: /lidar/points
  type: sensor_msgs/msg/PointCloud2
  qos:
    history_depth: 5
    reliability: best_effort
    durability: volatile
```
- Uses `best_effort` for low latency
- Small history to minimize memory usage
- Volatile since real-time data becomes stale quickly

### 2. Critical Navigation Data
```yaml
- name: /gps/fix
  type: sensor_msgs/msg/NavSatFix
  qos:
    history_depth: 10
    reliability: reliable
    durability: volatile
```
- Uses `reliable` to ensure every message is received
- Moderate history depth for buffering

### 3. Static Configuration Data
```yaml
- name: /robot_description
  type: std_msgs/msg/String
  qos:
    history_depth: 1
    reliability: reliable
    durability: transient_local
```
- Transient local allows late joiners to receive the configuration
- Only needs 1 message in history since data doesn't change

### 4. Camera Images
```yaml
- name: /camera/image_raw
  type: sensor_msgs/msg/Image
  qos:
    history_depth: 2
    reliability: best_effort
    durability: volatile
```
- Best effort for real-time performance
- Small history since images are large

## Debugging QoS Issues

If you're not receiving messages from a topic:

1. **Check publisher's QoS**: Use `ros2 topic info -v /topic_name` to see the publisher's QoS
2. **Match reliability**: If publisher uses `best_effort`, subscriber must also use `best_effort` (or lower)
3. **Check logs**: The topic monitor will log warnings for invalid QoS configurations
4. **Start with defaults**: If unsure, omit the `qos` section to use safe defaults

## Examples

See `topics_qos_example.yaml` for complete working examples demonstrating various QoS configurations.

## Backward Compatibility

If you omit the `qos` section from a topic configuration, the following defaults are used:
- history_depth: 10
- reliability: reliable
- durability: volatile

This matches the previous behavior, so existing configurations continue to work without modification.
