import os
import random
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String

import RPi.GPIO as GPIO

from ament_index_python.packages import get_package_share_directory

from PIL import Image

try:
    from MangDang.mini_pupper.display import Display
except ImportError:
    Display = None


class GameNode(Node):
    """
    Wakey Wakey sequence-repeat game.

    Flow:
      1. State machine publishes /wakey/robot_state=GAME.
      2. Game node plays 3 increasingly difficult direction sequences.
      3. User repeats each sequence using Pupper touch sensors.
      4. After 3 successful rounds, publishes /wakey/game_complete=True.

    Touch mapping:
      LEFT        = left sensor
      RIGHT       = right sensor
      FRONT       = front sensor
      LEFT_RIGHT  = left + right together
      LEFT_FRONT  = left + front together
      RIGHT_FRONT = right + front together
    """

    def __init__(self):
        super().__init__('game_node')

        # Publishers/subscribers
        self.game_complete_pub = self.create_publisher(Bool, '/wakey/game_complete', 10)

        self.create_subscription(String, '/wakey/robot_state', self.on_state_change, 10)

        # GPIO setup
        self.front_pin = 6
        self.left_pin = 3
        self.right_pin = 16

        GPIO.setmode(GPIO.BCM)

        # 0 = touched, 1 = not touched.
        GPIO.setup(self.front_pin, GPIO.IN)
        GPIO.setup(self.left_pin, GPIO.IN)
        GPIO.setup(self.right_pin, GPIO.IN)

        # Display + assets setup
        self.display = Display() if Display is not None else None

        self.package_share = get_package_share_directory('wakey_wakey')
        self.image_dir = os.path.join(self.package_share, 'assets', 'images')

        # Images for game interaction
        # TODO: test if upper-left/right look more intuitive
        self.action_images = {
            'RIGHT': 'arrow_upper_left.png',
            'LEFT': 'arrow_upper_right.png',
            'FRONT': 'arrow_up.png',
        }

        # Default Pupper facial expression images
        self.face_images = {
            'neutral': 'trot.png',
            'sad': 'rest.png',
            'happy': 'hop.png',
            'curious': 'finishhop.png',
            'sleep': 'shutdown.png',
        }

        # ------------------------------------------------------------------
        # Game configuration
        # ------------------------------------------------------------------
        self.total_rounds = 3
        self.max_attempts_per_round = 3

        # Sequence length per round, increasing difficulty
        self.round_lengths = {
            1: 2,
            2: 3,
            3: 5,
        }

        # Timeout behavior
        self.no_input_timeout = 12.0

        # Long-hold behavior
        self.long_hold_threshold = 1.0

        # Timer for sensor polling
        self.poll_period = 0.05
        self.timer = self.create_timer(self.poll_period, self.update)

        # ------------------------------------------------------------------
        # Runtime state
        # ------------------------------------------------------------------
        self.active = False
        self.mode = 'INACTIVE'  # INACTIVE, PLAYING, LISTENING, FEEDBACK, COMPLETE

        self.round_num = 0
        self.attempts_this_round = 0
        self.sequence = []
        self.user_input = []

        # Playback timing
        self.play_index = 0
        self.next_play_time = 0.0
        self.display_step_duration = 1.5

        # Input edge detection
        self.last_touch_token = None
        self.current_touch_token = None
        self.current_touch_start_time = None
        self.long_hold_warned = False

        # Timeout tracking
        self.last_input_time = None

        # Feedback timing
        # more robust than time.sleep()
        self.feedback_until = None
        self.pending_feedback_action = None

        self.get_logger().info('Game node started.')

    # ----------------------------------------------------------------------
    # ROS callbacks
    # ----------------------------------------------------------------------
    def on_state_change(self, msg: String):
        if msg.data == 'GAME' and not self.active:
            self.start_game()

        elif msg.data != 'GAME' and self.active:
            self.get_logger().info(f'Leaving GAME because state is now {msg.data}.')
            self.reset_game()

    # ----------------------------------------------------------------------
    # Game lifecycle
    # ----------------------------------------------------------------------
    def start_game(self):
        self.active = True
        self.mode = 'FEEDBACK'
        self.round_num = 0
        self.attempts_this_round = 0
        self.sequence = []
        self.user_input = []

        self.get_logger().info("You caught me! Let's make sure you're really awake.")

        self.show_face('happy')

        # Sound placeholder:
        # self.play_sound('you_caught_me.wav')
        # self.play_sound('celebrate.wav')
        # self.play_sound('lets_make_sure_youre_awake.wav')

        self.feedback_until = time.time() + 4.0
        self.pending_feedback_action = self.start_next_round

    def reset_game(self):
        self.active = False
        self.mode = 'INACTIVE'

        self.round_num = 0
        self.attempts_this_round = 0
        self.sequence = []
        self.user_input = []

        self.play_index = 0
        self.next_play_time = 0.0

        self.last_touch_token = None
        self.current_touch_token = None
        self.current_touch_start_time = None
        self.long_hold_warned = False
        self.last_input_time = None

        self.feedback_until = None
        self.pending_feedback_action = None

    def start_next_round(self):
        self.round_num += 1

        if self.round_num > self.total_rounds:
            self.complete_game()
            return

        self.attempts_this_round = 0
        self.sequence = self.generate_sequence(self.round_num)
        self.user_input = []

        self.get_logger().info(
            f'Round {self.round_num}/{self.total_rounds}: sequence is {self.sequence}'
        )

        self.show_face('neutral')

        # Sound placeholder:
        # self.play_sound(f'round_start.wav')

        self.replay_sequence()

    def complete_game(self):
        self.mode = 'COMPLETE'
        self.show_face('happy')

        self.get_logger().info('Game complete! Publishing /wakey/game_complete = True.')

        # Sound placeholder:
        # self.play_sound('game_complete_success_chime.wav')
        # self.play_sound('you_are_awake.wav')

        msg = Bool()
        msg.data = True
        self.game_complete_pub.publish(msg)

        self.reset_game()

    # ----------------------------------------------------------------------
    # Sequence generation
    # ----------------------------------------------------------------------
    def generate_sequence(self, round_num: int):
        """
        Rounds increase in difficulty:
          Round 1: only LEFT/RIGHT/FRONT, length 2
          Round 2: includes simultaneous two-sensor actions, length 3
          Round 3: includes all available actions, length 5
        """
        length = self.round_lengths.get(round_num, 3)
        allowed = ['LEFT', 'FRONT', 'RIGHT']

        sequence = []
        previous = None

        for _ in range(length):
            choices = [action for action in allowed if action != previous]
            action = random.choice(choices)
            sequence.append(action)
            previous = action

        return sequence

    def generate_easier_sequence(self):
        """
        Used after too many wrong attempts.
        Decreases difficulty by generating a shorter and easier sequence.
        """
        allowed = ['LEFT', 'FRONT', 'RIGHT']
        sequence = []

        previous = None
        for _ in range(2):
            choices = [action for action in allowed if action != previous]
            action = random.choice(choices)
            sequence.append(action)
            previous = action

        return sequence

    # ----------------------------------------------------------------------
    # Main timer update
    # ----------------------------------------------------------------------
    def update(self):
        if not self.active:
            return

        now = time.time()

        if self.mode == 'FEEDBACK':
            self.update_feedback(now)

        elif self.mode == 'PLAYING':
            self.update_sequence_playback(now)

        elif self.mode == 'LISTENING':
            self.update_user_input(now)

    def update_feedback(self, now: float):
        if self.feedback_until is None:
            return

        if now >= self.feedback_until:
            action = self.pending_feedback_action
            self.feedback_until = None
            self.pending_feedback_action = None

            if action is not None:
                action() # execute next action

    # ----------------------------------------------------------------------
    # Sequence playback
    # ----------------------------------------------------------------------
    def replay_sequence(self):
        self.mode = 'PLAYING'
        self.play_index = 0
        self.next_play_time = time.time() + 1.0

        self.user_input = []
        self.last_touch_token = None
        self.current_touch_token = None
        self.current_touch_start_time = None
        self.long_hold_warned = False

        self.get_logger().info('Playing sequence.')

    def update_sequence_playback(self, now: float):
        if now < self.next_play_time:
            return

        # finished playing and now listening
        if self.play_index >= len(self.sequence):
            self.mode = 'LISTENING'
            self.last_input_time = now
            self.show_face('neutral')

            self.get_logger().info('Your turn: repeat the sequence.')
            return

        action = self.sequence[self.play_index]
        self.show_action(action)
        self.play_action_sound(action)

        self.get_logger().info(f'Played step {self.play_index + 1}: {action}')

        self.play_index += 1
        self.next_play_time = now + self.display_step_duration

    # ----------------------------------------------------------------------
    # User input handling
    # ----------------------------------------------------------------------
    def update_user_input(self, now: float):
        token = self.read_touch_token()

        # Detect long hold.
        if token is not None:
            if token != self.current_touch_token:
                self.current_touch_token = token
                self.current_touch_start_time = now
                self.long_hold_warned = False
            else:
                held_time = now - self.current_touch_start_time
                if held_time >= self.long_hold_threshold and not self.long_hold_warned:
                    self.handle_long_hold()
                    return
        else:
            self.current_touch_token = None
            self.current_touch_start_time = None
            self.long_hold_warned = False

        # Edge detection: only count a tap when input changes from nothing to something.
        if token is not None and self.last_touch_token is None:
            self.handle_tap(token)
            self.last_input_time = now

        self.last_touch_token = token

        # Timeout replay if user does nothing for too long.
        if self.last_input_time is not None and now - self.last_input_time >= self.no_input_timeout:
            self.handle_timeout()

    def handle_tap(self, token: str):
        self.user_input.append(token)
        self.get_logger().info(f'User input: {self.user_input}')

        expected_prefix = self.sequence[:len(self.user_input)]

        if self.user_input != expected_prefix:
            self.handle_wrong_sequence()
            return

        if len(self.user_input) == len(self.sequence):
            self.handle_round_success()

    def handle_round_success(self):
        self.get_logger().info(f'Round {self.round_num} correct!')

        self.show_face('happy')

        # Sound placeholder:
        # self.play_sound('correct.wav')

        self.mode = 'FEEDBACK'
        self.feedback_until = time.time() + 3.0
        self.pending_feedback_action = self.start_next_round

    def handle_wrong_sequence(self):
        self.attempts_this_round += 1

        self.get_logger().info(
            f'Incorrect sequence. Attempt {self.attempts_this_round}/{self.max_attempts_per_round}.'
        )

        self.show_face('sad')

        # Sound placeholder:
        # self.play_sound('not_quite.wav')
        # self.play_sound('let_me_play_it_again.wav')

        if self.attempts_this_round >= self.max_attempts_per_round:
            self.get_logger().info("Too many incorrect attempts. Trying an easier sequence.")
            self.sequence = self.generate_easier_sequence()
            self.attempts_this_round = 0

            # Sound placeholder:
            # self.play_sound('lets_try_a_different_sequence.wav')

        self.mode = 'FEEDBACK'
        self.feedback_until = time.time() + 3.0
        self.pending_feedback_action = self.replay_sequence

    def handle_timeout(self):
        self.get_logger().info("No input detected. Replaying sequence.")

        self.show_face('curious')

        # Sound placeholder:
        # self.play_sound('here_let_me_play_it_again.wav')

        self.mode = 'FEEDBACK'
        self.feedback_until = time.time() + 3.0
        self.pending_feedback_action = self.replay_sequence

    def handle_long_hold(self):
        self.long_hold_warned = True
        self.attempts_this_round += 1

        self.get_logger().info(
            f'Long hold detected. Attempt {self.attempts_this_round}/{self.max_attempts_per_round}.'
        )

        self.show_face('sad')

        # Sound placeholder:
        # self.play_sound('try_tapping_the_pattern.wav')
        # self.play_sound('not_holding_it_down.wav')

        if self.attempts_this_round >= self.max_attempts_per_round:
            self.get_logger().info("Too many long holds/wrong attempts. Trying an easier sequence.")
            self.sequence = self.generate_easier_sequence()
            self.attempts_this_round = 0

            # Sound placeholder:
            # self.play_sound('lets_try_a_different_sequence.wav')

        self.mode = 'FEEDBACK'
        self.feedback_until = time.time() + 3.0
        self.pending_feedback_action = self.replay_sequence

    # ----------------------------------------------------------------------
    # Touch sensor reading
    # ----------------------------------------------------------------------
    def read_touch_token(self):
        """
        Reads touch sensors and returns the current symbolic input.

        Returns:
          None if no valid input.
          LEFT, RIGHT, FRONT, LEFT_RIGHT, LEFT_FRONT, RIGHT_FRONT otherwise.
        """
        left = GPIO.input(self.left_pin) == 0
        right = GPIO.input(self.right_pin) == 0
        front = GPIO.input(self.front_pin) == 0

        count = sum([left, right, front])

        if count == 0:
            return None

        # Do not accept all 3 at once.
        if count > 1:
            return None

        if left:
            return 'LEFT'
        if front:
            return 'FRONT'
        if right:
            return 'RIGHT'

        return None

    # ----------------------------------------------------------------------
    # Display helpers
    # ----------------------------------------------------------------------

    def show_action(self, action: str):
        image_name = self.action_images.get(action)

        if image_name is None:
            self.get_logger().warn(f'No image mapped for action {action}.')
            return

        self.setImage(image_name)


    def show_face(self, face_name: str):
        image_name = self.face_images.get(face_name)

        if image_name is None:
            self.get_logger().warn(f'No face image mapped for face {face_name}.')
            return

        self.setImage(image_name)

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

        output_path = '/tmp/wakey_wakey_game_face.png'
        canvas.save(output_path, 'PNG')
        return output_path

    # ----------------------------------------------------------------------
    # Sound placeholders
    # ----------------------------------------------------------------------
    def play_action_sound(self, action: str):
        """
        Placeholder for per-direction notes.

        Later you could map:
          LEFT        -> note_left.wav
          RIGHT       -> note_right.wav
          FRONT       -> note_front.wav
          LEFT_RIGHT  -> chord_left_right.wav
          LEFT_FRONT  -> chord_left_front.wav
          RIGHT_FRONT -> chord_right_front.wav
        """
        self.get_logger().info(f'[sound placeholder] play note for {action}')

        # Example future implementation:
        #
        # import sounddevice as sd
        # import soundfile as sf
        #
        # sound_map = {
        #     'LEFT': 'note_left.wav',
        #     'RIGHT': 'note_right.wav',
        #     'FRONT': 'note_front.wav',
        #     'LEFT_RIGHT': 'chord_left_right.wav',
        #     'LEFT_FRONT': 'chord_left_front.wav',
        #     'RIGHT_FRONT': 'chord_right_front.wav',
        # }
        #
        # filename = sound_map[action]
        # sound_path = os.path.join(
        #     self.package_share,
        #     'assets',
        #     'sounds',
        #     filename
        # )
        #
        # data, fs = sf.read(sound_path)
        # sd.play(data, fs)

    # def play_sound(self, filename: str):
    #     """
    #     Placeholder for voice lines / feedback sounds.
    #     """
    #     import sounddevice as sd
    #     import soundfile as sf
    #
    #     sound_path = os.path.join(
    #         self.package_share,
    #         'assets',
    #         'sounds',
    #         filename
    #     )
    #
    #     data, fs = sf.read(sound_path)
    #     sd.play(data, fs)


def main(args=None):
    rclpy.init(args=args)

    node = GameNode()

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
