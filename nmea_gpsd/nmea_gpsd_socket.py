#!/usr/bin/env python3

import socket
import time

import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Sentence


class NmeaToGpsd(Node):
    def __init__(self):
        super().__init__('nmea_to_gpsd')

        # Parameters
        self.declare_parameter('gpsd_host', '127.0.0.1')
        self.declare_parameter('gpsd_port', 3001)
        self.declare_parameter('nmea_topic', '/nmea')

        self.gpsd_host = self.get_parameter('gpsd_host').value
        self.gpsd_port = self.get_parameter('gpsd_port').value
        self.nmea_topic = self.get_parameter('nmea_topic').value

        self.sock = None
        self._connect()

        self.sub = self.create_subscription(
            Sentence,
            self.nmea_topic,
            self.nmea_callback,
            10
        )

        self.get_logger().info(
            f'Forwarding NMEA from [{self.nmea_topic}] '
            f'to gpsd at {self.gpsd_host}:{self.gpsd_port}'
        )

    def _connect(self):
        while rclpy.ok():
            try:
                self.sock = socket.create_connection(
                    (self.gpsd_host, self.gpsd_port), timeout=5.0
                )
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.get_logger().info('Connected to gpsd socket')
                return
            except OSError as e:
                self.get_logger().warn(
                    f'Failed to connect to gpsd ({e}), retrying...'
                )
                time.sleep(2.0)

    def nmea_callback(self, msg: Sentence):
        if self.sock is None:
            self._connect()
            return

        sentence = msg.sentence.strip()
        if not sentence.startswith('$'):
            return

        data = (sentence + '\r\n').encode('ascii', errors='ignore')

        try:
            self.sock.sendall(data)
        except OSError:
            self.get_logger().warn('Lost gpsd connection, reconnecting')
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None


def main():
    rclpy.init()
    node = NmeaToGpsd()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
