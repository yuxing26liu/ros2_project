#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


class WakeyDemoDriver(Node):
    def __init__(self):
        super().__init__('wakey_demo_driver')

        self.start_alarm_pub = self.create_publisher(Bool, '/wakey/start_alarm', 10)
        self.image_pub = self.create_publisher(Image, '/oak/right/image_rect', 10)

        self.create_subscription(String, '/wakey/robot_state', self.on_state, 10)

        self.state = None
        self.started = False
        self.start_time = time.time()

        self.timer = self.create_timer(0.1, self.tick)

    def on_state(self, msg):
        if msg.data != self.state:
            self.state = msg.data
            self.get_logger().info(f'FSM state = {self.state}')

    def make_image(self, blob=False):
        width = 160
        height = 120
        data = bytearray(width * height)

        if blob:
            # Bright blob on left side. Detection should publish direction='left'.
            for y in range(25, 95):
                for x in range(5, 65):
                    data[y * width + x] = 255

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = height
        msg.width = width
        msg.encoding = 'mono8'
        msg.is_bigendian = 0
        msg.step = width
        msg.data = bytes(data)
        return msg

    def tick(self):
        elapsed = time.time() - self.start_time

        # First publish clean background frames while FSM is IDLE.
        if elapsed < 2.0:
            self.image_pub.publish(self.make_image(blob=False))
            return

        # Trigger alarm once.
        if not self.started:
            msg = Bool()
            msg.data = True
            self.start_alarm_pub.publish(msg)
            self.started = True
            self.get_logger().info('Published /wakey/start_alarm = True')

        # Keep publishing foreground blob so detection can trigger FLEEING.
        if elapsed < 8.0:
            self.image_pub.publish(self.make_image(blob=True))
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