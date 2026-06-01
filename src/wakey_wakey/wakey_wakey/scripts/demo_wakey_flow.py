#!/usr/bin/env python3

import time

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


class WakeyDemoDriver(Node):
    IMAGE_TOPIC = '/oak/stereo/image_raw'

    def __init__(self):
        super().__init__('wakey_demo_driver')

        self.start_alarm_pub = self.create_publisher(Bool, '/wakey/start_alarm', 10)
        self.image_pub = self.create_publisher(Image, self.IMAGE_TOPIC, 10)

        self.create_subscription(String, '/wakey/robot_state', self.on_state, 10)

        self.state = None
        self.started = False
        self.start_time = time.time()

        self.timer = self.create_timer(0.1, self.tick)

    def on_state(self, msg):
        if msg.data != self.state:
            self.state = msg.data
            self.get_logger().info(f'FSM state = {self.state}')

    def make_depth_image(self, direction=None):
        width = 160
        height = 120
        depth_mm = np.full((height, width), 3000, dtype=np.uint16)

        if direction is not None:
            x_ranges = {
                'left': (5, 55),
                'center': (55, 105),
                'right': (105, 155),
            }
            x0, x1 = x_ranges[direction]
            depth_mm[25:95, x0:x1] = 650

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = height
        msg.width = width
        msg.encoding = '16UC1'
        msg.is_bigendian = 0
        msg.step = width * 2
        msg.data = depth_mm.tobytes()
        return msg

    def demo_direction(self, elapsed):
        if elapsed < 4.0:
            return 'left'
        if elapsed < 6.0:
            return 'center'
        return 'right'

    def tick(self):
        elapsed = time.time() - self.start_time

        # First publish far depth frames while FSM is IDLE.
        if elapsed < 2.0:
            self.image_pub.publish(self.make_depth_image())
            return

        # Trigger alarm once.
        if not self.started:
            msg = Bool()
            msg.data = True
            self.start_alarm_pub.publish(msg)
            self.started = True
            self.get_logger().info('Published /wakey/start_alarm = True')

        # Keep publishing a close depth blob so detection can report direction.
        if elapsed < 8.0:
            self.image_pub.publish(self.make_depth_image(self.demo_direction(elapsed)))
            return

        self.get_logger().info('Demo driver done. Now use wakefulness touch sensors to enter GAME.')
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = WakeyDemoDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
