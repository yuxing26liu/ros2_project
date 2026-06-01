import math
import os
import shlex
import shutil
import signal
import subprocess
import tempfile
import wave

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


class AudioNode(Node):
    """
    Plays the wake-up alarm until Pupper is caught.

    The state machine owns the behavior transitions. This node only reacts to
    /wakey/robot_state so audio cannot block movement, detection, or the game.
    """

    ALARM_STATES = {'ALARMING', 'FLEEING'}

    def __init__(self):
        super().__init__('audio_node')

        self.declare_parameter('alarm_file', 'mixkit-classic-alarm-995.wav')
        self.declare_parameter('player_command', '')
        self.declare_parameter('restart_delay', 0.15)

        self.package_share = get_package_share_directory('wakey_wakey')
        self.alarm_file = self.get_parameter('alarm_file').value
        self.player_command = self.get_parameter('player_command').value
        self.restart_delay = self.get_parameter('restart_delay').value

        self.alarm_path = self.resolve_alarm_path(self.alarm_file)
        self.process = None
        self.alarm_active = False
        self.last_start_time = 0.0

        self.create_subscription(String, '/wakey/robot_state', self.on_state_change, 10)
        self.create_timer(0.2, self.keep_alarm_running)

        self.get_logger().info(f'Audio node ready. Alarm file: {self.alarm_path}')

    def resolve_alarm_path(self, alarm_file: str) -> str:
        if os.path.isabs(alarm_file) and os.path.isfile(alarm_file):
            return alarm_file

        packaged_path = os.path.join(
            self.package_share,
            'assets',
            'sounds',
            alarm_file
        )
        if os.path.isfile(packaged_path):
            return packaged_path

        self.get_logger().warn(
            f'Could not find {alarm_file}; generating a simple fallback alarm tone.'
        )
        return self.generate_fallback_alarm()

    def generate_fallback_alarm(self) -> str:
        output_path = os.path.join(tempfile.gettempdir(), 'wakey_wakey_alarm.wav')

        sample_rate = 44100
        duration = 2.0
        volume = 0.45
        frequency = 880.0

        with wave.open(output_path, 'w') as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)
            wav_file.setframerate(sample_rate)

            for i in range(int(sample_rate * duration)):
                t = i / sample_rate
                beep_on = (t % 0.5) < 0.32
                sample = 0
                if beep_on:
                    sample = int(
                        volume * 32767 * math.sin(2.0 * math.pi * frequency * t)
                    )
                wav_file.writeframesraw(sample.to_bytes(2, 'little', signed=True))

        return output_path

    def on_state_change(self, msg: String):
        "idle stage will play the alarm"
        should_alarm = msg.data in self.ALARM_STATES

        if should_alarm and not self.alarm_active:
            self.get_logger().info(f'State {msg.data}: starting alarm loop.')
            self.alarm_active = True
            self.start_alarm()

        elif not should_alarm and self.alarm_active:
            self.get_logger().info(f'State {msg.data}: stopping alarm loop.')
            self.alarm_active = False
            self.stop_alarm()

    def keep_alarm_running(self):
        if not self.alarm_active:
            return

        if self.process is None:
            self.start_alarm()
            return

        if self.process.poll() is not None:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_start_time >= self.restart_delay:
                self.start_alarm()

    def start_alarm(self):
        if self.process is not None and self.process.poll() is None:
            return

        command = self.build_player_command()
        if command is None:
            return

        try:
            self.process = subprocess.Popen(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True
            )
            self.last_start_time = self.get_clock().now().nanoseconds / 1e9
        except OSError as exc:
            self.get_logger().error(f'Could not start alarm audio: {exc}')
            self.process = None

    def build_player_command(self):
        if self.player_command:
            command = shlex.split(self.player_command)
        else:
            command = self.detect_player_command()

        if not command:
            self.get_logger().error(
                'No audio player found. Install aplay or set player_command.'
            )
            return None

        return command + [self.alarm_path]

    def detect_player_command(self):
        if shutil.which('aplay'):
            return ['aplay', '-q']
        if shutil.which('paplay'):
            return ['paplay']
        if shutil.which('afplay'):
            return ['afplay']
        return []

    def stop_alarm(self):
        if self.process is None:
            return

        if self.process.poll() is None:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                self.process.wait(timeout=1.0)
            except (OSError, subprocess.TimeoutExpired):
                self.process.kill()

        self.process = None

    def destroy_node(self):
        self.alarm_active = False
        self.stop_alarm()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
