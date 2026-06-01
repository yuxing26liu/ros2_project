import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String

import RPi.GPIO as GPIO

class WakefulnessNode(Node):
    def __init__(self):
        super().__init__('wakefulness_node')

        # Publishes to FSM
        self.wakefulness_confirmed_pub = self.create_publisher(Bool, '/wakey/wakefulness_confirmed', 10)

        # Listen to the FSM state so this node only checks touch during FLEEING state
        self.create_subscription(String, '/wakey/robot_state', self.on_state_change, 10)

        # User must create this many distinct touch events within touch_window seconds.
        # A touch event is counted when any sensor changes from untouched -> touched.
        # Alternatively the user can hold down for at least required_hold_time
        self.required_touch_count = 2
        self.touch_window = 3.0
        self.required_hold_time = 1.0
        self.touch_start_time = None

        # Ignore additional detected touches that happen too close together.
        # Help avoid counting sensor noise as multiple touches
        self.debounce_time = 0.08

        # How often to check the sensors, in seconds.
        self.poll_period = 0.02

        # Internal state
        self.robot_state = None
        self.active = False
        self.confirmed = False
        self.touch_times = []
        self.was_touching = False
        self.last_touch_event_time = 0.0

        self.front_pin = 6
        self.left_pin = 3
        self.right_pin = 16
    
        # 0 = touched, 1 = not touched.
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.left_pin, GPIO.IN)
        GPIO.setup(self.right_pin, GPIO.IN)
        GPIO.setup(self.front_pin, GPIO.IN)

        # Timer periodically polls the GPIO pins.
        self.timer = self.create_timer(self.poll_period, self.check_touch_sensors)

        self.get_logger().info('Wakefulness node started.')

    def reset_touch_tracking(self):
        """
        Clears any touch history so a new FLEEING interaction starts fresh.
        """
        self.touch_times = []
        self.was_touching = False
        self.last_touch_event_time = 0.0
        self.touch_start_time = None

    def on_state_change(self, msg: String):
        """
        Called whenever the state machine publishes a new robot state.
        This node only activates during FLEEING, because the user should only
        be able to confirm wakefulness after Pupper has started fleeing.
        """
        previous_state = self.robot_state
        self.robot_state = msg.data

        if self.robot_state == 'FLEEING':
            if not self.active:
                self.get_logger().info(
                    f"Wakefulness detection active. Touch pupper's head "
                    f'{self.required_touch_count} times within {self.touch_window:.1f} seconds or hold down for over {self.required_hold_time:.1f} seconds.'
                )
                self.reset_touch_tracking()
            self.active = True
            self.confirmed = False

        else:
            if previous_state == 'FLEEING': # FLEEING has ended (dictated by FSM)
                self.get_logger().info('Wakefulness detection inactive.')
            self.active = False
            self.confirmed = False
            self.reset_touch_tracking()

    def any_sensor_touched(self) -> bool:
        """
        Returns True when at least one touch sensor is touched.
        """
        left = GPIO.input(self.left_pin)
        right = GPIO.input(self.right_pin)
        front = GPIO.input(self.front_pin)

        return (left == 0) or (right == 0) or (front == 0)
    
    def check_touch_sensors(self):
        """
        Periodically checks for touch events (tapping or holding). Once confirmed, publishes True once.
        """
        if not self.active:
            return

        if self.confirmed:
            return

        now = time.time()
        touching = self.any_sensor_touched()

        # Keep only taps inside the rolling time window.
        self.touch_times = [t for t in self.touch_times if now - t <= self.touch_window]

        # Track continuous holding.
        if touching:
            if self.touch_start_time is None:
                self.touch_start_time = now

            held_duration = now - self.touch_start_time

            if held_duration >= self.required_hold_time:
                self.get_logger().info(f'Touch held for {held_duration:.2f} seconds. Wakefulness confirmed.')
                self.publish_wakefulness_confirmed()
                self.confirmed = True
                self.active = False
                self.reset_touch_tracking()
                return
        else:
            # Once the user releases, the current hold attempt ends.
            self.touch_start_time = None

        # Count a new tap only on the transition from untouched -> touched.
        # This means one long hold (spanning multiple polled time intervals) counts as one tap, not many taps.
        new_touch_event = touching and not self.was_touching

        if new_touch_event:
            if now - self.last_touch_event_time >= self.debounce_time:
                self.touch_times.append(now)
                self.last_touch_event_time = now
                self.get_logger().info(
                    f'Touch detected. Count = {len(self.touch_times)}/{self.required_touch_count} '
                    f'within {self.touch_window:.1f} seconds.'
                )

                if len(self.touch_times) >= self.required_touch_count:
                    self.get_logger().info(f'{len(self.touch_times)} touches detected. Wakefulness confirmed.')
                    self.publish_wakefulness_confirmed()
                    self.confirmed = True
                    self.active = False
                    self.reset_touch_tracking()
                    return

        self.was_touching = touching

    def publish_wakefulness_confirmed(self):
        """
        Publishes a confirmation signal to the state machine.
        FSM should subscribe to /wakey/wakefulness_confirmed.
        """
        msg = Bool()
        msg.data = True
        self.wakefulness_confirmed_pub.publish(msg)

        self.get_logger().info('Wakefulness confirmed. Published /wakey/wakefulness_confirmed = True.')


def main(args=None):
    rclpy.init(args=args)

    node = WakefulnessNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()