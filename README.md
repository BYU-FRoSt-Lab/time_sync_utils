# nmea_gpsd

The `nmea_gpsd` package is a ROS 2 utility designed to bridge ROS-based NMEA data to the `gpsd` service. It provides nodes to convert standard ROS messages (like `NavSatFix`) into NMEA sentences and forward them to `gpsd` via various protocols.

**Note:** The serial and TCP implementations are currently considered partially developed and may require further refinement for specific production environments.

## Nodes

### 1. `navsat_gpsd_serial`

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



### 2. `nmea_gpsd_socket`

This node acts as a TCP server that listens for a connection from `gpsd`. Once connected, it forwards NMEA sentences from a ROS topic directly to the TCP client.

* **Subscribed Topics:**
* Defined by `nmea_topic` parameter (default: `/nmea`) (`nmea_msgs/msg/Sentence`).


* **Parameters:**
* `gpsd_host` (string, default: `0.0.0.0`): The address the TCP server binds to.
* `gpsd_port` (int, default: `3001`): The port `gpsd` will connect to.
* `nmea_topic` (string, default: `/nmea`): The ROS topic containing NMEA sentences to forward.



### 3. `nmea_gpsd_udp`

This node forwards NMEA data to `gpsd` via UDP. It can also synthesize `$GPRMC` sentences by combining raw NMEA GGA data with SBG-specific UTC time messages.

* **Subscribed Topics:**
* `nmea_topic` (default: `/nmea`): ROS topic for incoming NMEA sentences.
* `utc_topic` (default: `/sbg/utc_time`): ROS topic for `sbg_driver/msg/SbgUtcTime` used to generate RMC sentences.


* **Parameters:**
* `gpsd_host` (string, default: `127.0.0.1`): The destination IP where `gpsd` is listening for UDP.
* `gpsd_port` (int, default: `3001`): The destination UDP port.



## Installation

This package depends on `rclpy` and `nmea_msgs`. It is built using `ament_python`.

```bash
# Example build command
colcon build --packages-select nmea_gpsd

```

## License

This package is licensed under the Apache License 2.0.