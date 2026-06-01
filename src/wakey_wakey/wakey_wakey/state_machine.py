import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String, Int32

from datetime import datetime

import os

from PIL import Image

from ament_index_python.packages import get_package_share_directory

try:
    from MangDang.mini_pupper.display import Display
except ImportError:
    Display = None

# Valid state transitions — only these are allowed
VALID_TRANSITIONS = {
    'IDLE':     ['ALARMING'],
    'ALARMING': ['FLEEING'],
    'FLEEING':  ['GAME'],
    'GAME':     ['DONE'],
    'DONE':     ['IDLE'],   # allows reset for multi-session testing
}

class StateMachineNode(Node):

    def __init__(self):
        super().__init__('state_machine_node')

        # --- Parameters ---
        self.declare_parameter('alarm_time', '07:00')  # set via launch file
        self.alarm_time = self.get_parameter('alarm_time').value

        # --- State ---
        self.state = 'IDLE'

        # --- Publishers ---
        self.robot_state_pub = self.create_publisher(String, '/wakey/robot_state', 10)
        self.escalation_pub = self.create_publisher(Int32, '/wakey/escalation_level', 10)

        # --- Subscribers ---
        self.create_subscription(Bool, '/wakey/flee_trigger',               self.on_flee_trigger,               10)
        self.create_subscription(Bool, '/wakey/wakefulness_confirmed',      self.on_wakefulness_confirmed,      10)
        self.create_subscription(Bool, '/wakey/game_complete',              self.on_game_complete,              10)
        self.create_subscription(Bool, '/wakey/start_alarm',                self.on_start_alarm,                10)  # manual test trigger

        # --- Timers ---
        self.create_timer(1.0,  self.publish_state)       # re-broadcast state every second
        self.create_timer(30.0, self.escalate_urgency)    # escalation while ALARMING/FLEEING
        self.create_timer(1.0,  self.check_alarm_time)    # poll clock for alarm trigger

        self.escalation_level = 0

        self.display = Display() if Display is not None else None
        self.package_share = get_package_share_directory('wakey_wakey')
        self.image_dir = os.path.join(self.package_share, 'assets', 'images')

        self.get_logger().info(f'State machine started. Alarm set for {self.alarm_time}.')
        self.publish_state()

    def setImage(self, image_name: str):
        if self.display is None:
            self.get_logger().warn('Display library unavailable. Skipping image display.')
            return

        image_path = os.path.join(self.image_dir, image_name)

        if not os.path.exists(image_path):
            self.get_logger().warn(f'Image not found: {image_path}')
            return

        try:
            prepared_path = self.prepare_display_image(image_path)
            self.display.show_image(prepared_path)
        except Exception as exc:
            self.get_logger().error(f'Failed to display image {image_path}: {exc}')

    def prepare_display_image(self, image_path: str):
        display_width = 320
        display_height = 240

        img = Image.open(image_path).convert('RGBA')
        img.thumbnail((display_width, display_height), Image.Resampling.LANCZOS)

        canvas = Image.new('RGBA', (display_width, display_height), (255, 255, 255, 255))
        x = (display_width - img.width) // 2
        y = (display_height - img.height) // 2
        canvas.paste(img, (x, y), img)

        output_path = '/tmp/wakey_wakey_state_face.png'
        canvas.save(output_path, 'PNG')
        return output_path

    # -------------------------------------------------------
    # Timer callbacks
    # -------------------------------------------------------

    def check_alarm_time(self):
        """Trigger IDLE → ALARMING at the scheduled alarm time."""
        if self.state != 'IDLE':
            return
        now = datetime.now().strftime('%H:%M')
        if now == self.alarm_time:
            self.get_logger().info('Alarm time reached.')
            self.transition_to('ALARMING')

    def escalate_urgency(self):
        """Increase urgency every 30s while robot is active."""
        if self.state not in ['ALARMING', 'FLEEING']:
            self.escalation_level = 0
            return
        self.escalation_level = min(self.escalation_level + 1, 3)
        self.get_logger().info(f'Escalation level: {self.escalation_level}')
        self.publish_escalation_level()

    # -------------------------------------------------------
    # Subscriber callbacks
    # -------------------------------------------------------

    def on_start_alarm(self, msg: Bool):
        """Manual test trigger — bypasses clock check."""
        if msg.data and self.state == 'IDLE':
            self.transition_to('ALARMING')

    def on_flee_trigger(self, msg: Bool):
        """Estella's detection node — user is approaching."""
        if msg.data and self.state == 'ALARMING':
            self.transition_to('FLEEING')
        elif msg.data:
            self.get_logger().warn(f'Flee trigger ignored in state {self.state}')

    def on_wakefulness_confirmed(self, msg: Bool):
        """Christina's wakefulness node — robot has been picked up."""
        if msg.data and self.state == 'FLEEING':
            self.transition_to('GAME')
        elif msg.data:
            self.get_logger().warn(f'Wakefulness confirmed ignored in state {self.state}')

    def on_game_complete(self, msg: Bool):
        """Christina's game node — user answered correctly."""
        if msg.data and self.state == 'GAME':
            self.transition_to('DONE')
        elif msg.data:
            self.get_logger().warn(f'Game complete ignored in state {self.state}')

    # -------------------------------------------------------
    # Core transition logic
    # -------------------------------------------------------

    def transition_to(self, new_state: str):
        """Single entry point for all state changes."""
        if new_state == self.state:
            return

        # Guard: reject invalid transitions
        if new_state not in VALID_TRANSITIONS.get(self.state, []):
            self.get_logger().error(
                f'Invalid transition: {self.state} → {new_state}. Ignoring.'
            )
            return

        old_state = self.state
        self.state = new_state
        self.escalation_level = 0

        self.get_logger().info(f'State: {old_state} → {new_state}')
        self.publish_state()
        self.publish_escalation_level()

        # On-entry hooks
        self._on_entry(new_state)

    def _on_entry(self, state: str):
        """Called once every time a new state is entered."""
        if state == 'ALARMING':
            self.get_logger().info('ALARMING: play rooster crow + display sunrise face')
            # TODO: trigger sound playback
            # TODO: trigger display face
            self.setImage("trot.png")
        elif state == 'FLEEING':
            self.get_logger().info('FLEEING: begin evasive movement')
            # flee_behavior.py handles this reactively via /wakey/robot_state
        elif state == 'GAME':
            self.get_logger().info('GAME: wakefulness game starting')
            # game.py handles this reactively via /wakey/robot_state
        elif state == 'DONE':
            self.get_logger().info('DONE: play triumphant chime + display happy face')
            # TODO: trigger sound + display
            self.setImage("shutdown.png")
        elif state == 'IDLE':
            self.get_logger().info('IDLE: system reset, waiting for alarm time')

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.robot_state_pub.publish(msg)

    def publish_escalation_level(self):
        msg = Int32()
        msg.data = self.escalation_level
        self.escalation_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
