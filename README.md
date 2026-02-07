# time_sync_utils

This repository contains utilities for handling and monitoring time-synchronized data in ROS 2. It consists of two main packages: `nmea_gpsd` for bridging GPS data and `topic_monitor` for validating message synchronization.


## 1. topic_monitor

The `topic_monitor` package is a diagnostic tool used to verify that various ROS 2 topics are active and that their timestamps are synchronized within a specified threshold. It reads a list of target topics from a configuration file, monitors them for 5 seconds, and reports a summary of message counts and synchronization offsets.

### Nodes

* **`topic_monitor_node`**: The primary C++ node that subscribes to topics and performs timing analysis.

### Parameters

* **`topics_file`** (string, default: `config/topics.yaml`): The path to the YAML file listing topics to monitor.
* **`relative_path`** (bool, default: `true`): If true, the node looks for the `topics_file` relative to the package's share directory.
* **`sync_threshold_warn_seconds`** (double, default: `0.1`): The time difference (in seconds) between a topic's timestamp and the reference topic that triggers a warning.
* **`sync_threshold_error_seconds`** (double, default: `1.0`): The time difference that triggers an error.

### QoS Configuration

Each topic in the `topics.yaml` file can optionally specify custom QoS (Quality of Service) settings. If not specified, default values are used.

**Available QoS Options:**

* **`history_depth`** (integer, default: `10`): The depth of the message history queue.
* **`reliability`** (string, default: `"reliable"`): The reliability policy. Options:
  * `"reliable"`: Ensures message delivery (may retry)
  * `"best_effort"`: Does not guarantee message delivery
* **`durability`** (string, default: `"volatile"`): The durability policy. Options:
  * `"volatile"`: Only subscribers that are alive at publication time receive messages
  * `"transient_local"`: Late-joining subscribers may receive previously published messages

**Example:**

```yaml
topics:
  - name: /example_topic
    type: sensor_msgs/msg/NavSatFix
    qos:
      history_depth: 5
      reliability: best_effort
      durability: transient_local
```

---

## 2. nmea_gpsd

The `nmea_gpsd` package is a ROS 2 utility designed to bridge ROS-based NMEA data to the `gpsd` service. It provides nodes to convert standard ROS messages (like `NavSatFix`) into NMEA sentences and forward them to `gpsd` via various protocols.

**Note:** The serial and TCP implementations in this package are currently considered partially developed and may require further refinement for specific production environments.

### Nodes

#### `navsat_gpsd_serial`

This node converts ROS `sensor_msgs/msg/NavSatFix` and `geometry_msgs/msg/TwistWithCovarianceStamped` messages into standard NMEA `$GPGGA` and `$GPRMC` sentences. These sentences are published to a ROS topic and optionally written to a serial port.

* **Subscribed Topics:**
* `gnss_1/llh_position` (`sensor_msgs/msg/NavSatFix`): Source of position and fix status.
* `gnss_1/velocity` (`geometry_msgs/msg/TwistWithCovarianceStamped`): Source of speed and course.


* **Published Topics:**
* `nmea_constructed` (`nmea_msgs/msg/Sentence`): The generated NMEA sentences.


* **Parameters:**
* `serial_port` (string, default: `/dev/tnt1`): The virtual or physical serial port to write NMEA sentences to.
* `baud_rate` (int, default: `9600`): Baud rate for the serial port.
* `nmea_talker_id` (string, default: `GP`): The talker ID prefix (e.g., `GP` for GPS, `GN` for GLONASS).
* `default_num_satellites` (int, default: `4`): Default satellite count used in GGA sentences.
* `default_hdop` (double, default: `1.5`): Default Horizontal Dilution of Precision.
* `default_geoid_separation` (double, default: `0.0`): Default geoidal separation in meters.



#### `nmea_gpsd_socket`

This node acts as a TCP server that listens for a connection from `gpsd`. Once connected, it forwards NMEA sentences from a ROS topic directly to the TCP client.

* **Subscribed Topics:**
* Defined by `nmea_topic` parameter (default: `/nmea`) (`nmea_msgs/msg/Sentence`).


* **Parameters:**
* `gpsd_host` (string, default: `0.0.0.0`): The address the TCP server binds to.
* `gpsd_port` (int, default: `3001`): The port `gpsd` will connect to.
* `nmea_topic` (string, default: `/nmea`): The ROS topic containing NMEA sentences to forward.



#### `nmea_gpsd_udp`

This node forwards NMEA data to `gpsd` via UDP. It can also synthesize `$GPRMC` sentences by combining raw NMEA GGA data with SBG-specific UTC time messages.

* **Subscribed Topics:**
* `nmea_topic` (default: `/nmea`): ROS topic for incoming NMEA sentences.
* `utc_topic` (default: `/sbg/utc_time`): ROS topic for `sbg_driver/msg/SbgUtcTime` used to generate RMC sentences.


* **Parameters:**
* `gpsd_host` (string, default: `127.0.0.1`): The destination IP where `gpsd` is listening for UDP.
* `gpsd_port` (int, default: `3001`): The destination UDP port.






## License

Both packages are licensed under the Apache License 2.0.