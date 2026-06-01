#!/usr/bin/env python3

import argparse
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String


class WakeyDemoDriver(Node):
    """
    Drives the Wakey state-machine demo using the real detection node.

    This node intentionally does not publish camera images. Start the OAK-D ROS
    camera publisher separately, then run detection.py against that real image
    topic.
    """

    def __init__(self, alarm_delay_sec):
        super().__init__('wakey_demo_driver')

        self.start_alarm_pub = self.create_publisher(Bool, '/wakey/start_alarm', 10)

        self.create_subscription(String, '/wakey/robot_state', self.on_state, 10)

        self.state = None
        self.started = False
        self.done_logged = False
        self.alarm_delay_sec = alarm_delay_sec
        self.start_time = time.time()

        self.timer = self.create_timer(0.1, self.tick)

    def on_state(self, msg):
        if msg.data != self.state:
            self.state = msg.data
            self.get_logger().info(f'FSM state = {self.state}')

    def tick(self):
        elapsed = time.time() - self.start_time

        if not self.started and elapsed >= self.alarm_delay_sec:
            msg = Bool()
            msg.data = True
            self.start_alarm_pub.publish(msg)
            self.started = True
            self.get_logger().info('Published /wakey/start_alarm = True')

        if self.state == 'DONE' and not self.done_logged:
            self.done_logged = True
            self.get_logger().info('Demo flow reached DONE.')
            return

        if self.started and self.state == 'FLEEING':
            self.get_logger().info('Detection triggered FLEEING. Catch/lift Pupper to enter GAME.')
            self.timer.cancel()


def main(args=None):
    parser = argparse.ArgumentParser(description='Start Wakey flow while using real camera detection.')
    parser.add_argument(
        '--alarm-delay-sec',
        type=float,
        default=2.0,
        help='Seconds to wait before publishing /wakey/start_alarm.',
    )
    parsed_args, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = WakeyDemoDriver(parsed_args.alarm_delay_sec)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
