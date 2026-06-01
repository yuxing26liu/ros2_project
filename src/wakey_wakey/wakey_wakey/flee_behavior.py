import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Int32
from geometry_msgs.msg import Twist

class FleeBehavior(Node):

    def __init__(self):
        super().__init__('flee_behavior')

        # --- State tracking ---
        self.active = False
        self.escalation_level = 0
        self.latest_user_direction = None

        # --- Wiggle state ---
        self.wiggle_timer = None
        self.wiggle_state = 0
        self.wiggle_count = 0

        # --- Flee timer (duration of flee) ---
        self.flee_timer = None

        # --- Subscribers ---
        self.create_subscription(String, '/wakey/robot_state', self.on_state_change, 10)
        self.create_subscription(String, '/wakey/user_direction', self.on_user_direction, 10)
        self.create_subscription(Int32, '/wakey/escalation_level', self.on_escalation, 10)
        # TODO: subscribe to Pupper bump sensor topic

        # --- Publishers ---
        # TODO: confirm Pupper's actual cmd_vel topic name from ros2 topic list
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('FleeBehavior node started.')

    # -------------------------------------------------------
    # State change handler
    # -------------------------------------------------------

    def on_state_change(self, msg: String):
        if msg.data == 'ALARMING':
            self.cancel_flee_timer()
            self.start_wiggle_animation()
        elif msg.data == 'FLEEING':
            self.cancel_wiggle_timer()  # stop wiggle if still running
            self.active = True
            if self.latest_user_direction is not None:
                self.select_flee_trajectory(self.latest_user_direction)
        else:
            # GAME, DONE, IDLE — stop everything
            self.active = False
            self.cancel_wiggle_timer()
            self.cancel_flee_timer()
            self.stop_movement()

    # -------------------------------------------------------
    # Subscriber callbacks
    # -------------------------------------------------------

    def on_user_direction(self, msg: String):
        """React to user direction only while FLEEING."""
        self.latest_user_direction = msg.data

        if not self.active:
            return
        self.select_flee_trajectory(msg.data)

    def on_escalation(self, msg: Int32):
        self.escalation_level = msg.data

    def on_bump_sensor(self, msg):
        """Triggered by boundary contact — back away from wall."""
        if self.active:
            self.select_flee_trajectory('center')

    # -------------------------------------------------------
    # Wiggle animation
    # -------------------------------------------------------

    def start_wiggle_animation(self):
        """Start wiggle timer — called on ALARMING state entry."""
        if self.wiggle_timer is not None:
            return

        self.wiggle_state = 0
        self.wiggle_count = 0
        self.wiggle_timer = self.create_timer(0.4, self.wiggle_step)

    def wiggle_step(self):
        """Called every 0.4s by timer. Alternates left/right turns."""
        MAX_WIGGLES = 6  # 3 full left-right cycles

        if self.wiggle_count >= MAX_WIGGLES:
            self.stop_movement()
            self.cancel_wiggle_timer()
            return

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.6 if self.wiggle_state == 0 else -0.6
        self.wiggle_state = 1 - self.wiggle_state  # toggle 0 → 1 → 0
        self.cmd_pub.publish(twist)
        self.wiggle_count += 1

    def cancel_wiggle_timer(self):
        if self.wiggle_timer is not None:
            self.wiggle_timer.cancel()
            self.wiggle_timer = None

    # -------------------------------------------------------
    # Flee movement
    # -------------------------------------------------------

    def select_flee_trajectory(self, user_direction: str):
        """Pick evasive trajectory opposite to user direction."""
        self.cancel_flee_timer()  # cancel any in-progress flee first
        if user_direction == 'left':
            self.flee_right()
        elif user_direction == 'right':
            self.flee_left()
        else:
            self.flee_backward()

    def flee_right(self):
        twist = Twist()
        twist.linear.x = -0.5
        twist.angular.z = 1.0
        self.cmd_pub.publish(twist)
        self.flee_timer = self.create_timer(3.0, self.stop_after_flee)

    def flee_left(self):
        twist = Twist()
        twist.linear.x = -0.5
        twist.angular.z = -1.0
        self.cmd_pub.publish(twist)
        self.flee_timer = self.create_timer(3.0, self.stop_after_flee)

    def flee_backward(self):
        twist = Twist()
        twist.linear.x = -0.5
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.flee_timer = self.create_timer(3.0, self.stop_after_flee)

    def stop_after_flee(self):
        """Called by flee_timer after movement duration expires."""
        self.stop_movement()
        self.cancel_flee_timer()

    def cancel_flee_timer(self):
        if self.flee_timer is not None:
            self.flee_timer.cancel()
            self.flee_timer = None

    def stop_movement(self):
        self.cmd_pub.publish(Twist())  # all zeros = stop


def main(args=None):
    rclpy.init(args=args)
    node = FleeBehavior()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
