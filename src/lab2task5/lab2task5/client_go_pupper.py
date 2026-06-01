########
# Name: client_go_pupper
#
# Purpose: Go Pupper Client. Client code which will communicate with the GoPupper service by
#          detecting touch sensor info from the user (e.g., front, left, right sensor) and
#          passing along the corresponding movement command to the Go Pupper Service to
#          control the robot's movement. We have added gaze cues to Pupper via displayed emojis
#          that correspond to the direction of movement.
#
# Usage: First launch the service (see lab/file). Navigate to the src/lab2task5/lab2task5 directory.
#        Then you can run the client like this:
#        ros2 run lab2task5 client
#
# Author: Terri Tai <y2tai@ucsd.edu> Yuxing Liu <yul269@ucsd.edu> Christina Xu <chx029@ucsd.edu> 
#
# Acknowledgements: Used some code from ROS 2 Tutorials, MangDang's ROS git repo, and Laurel Riek's code for testing Pupper's display
#  https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html
#  https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html#test-the-new-interfaces
#  https://github.com/mangdangroboticsclub/mini_pupper_ros/blob/ros2-dev/mini_pupper_dance/mini_pupper_dance/dance_server.py
#  https://drive.google.com/file/d/1PzpO8Qhm-6oTizqU8Y7eomLyHxNoES1U/view
#
# Date: 13 May 2026
########

# Import the ROS2 interface we wrote, called GoPupper. This specifies the message type.
from pupper_interfaces.srv import GoPupper

# Lets us read arguments from the command line as needed
import sys

# Packages to let us create nodes and spin them up
import rclpy
from rclpy.node import Node

# Package for working with RPi GPIO pins for touch sensors
import RPi.GPIO as GPIO

# Package to measure time
import time

from MangDang.mini_pupper.display import Display, BehaviorState
from resizeimage import resizeimage  # library for image resizing
from PIL import Image, ImageDraw, ImageFont


###
# Name: Minimal Client Async
#
# Purpose: "The MinimalClientAsync class constructor initializes the node with the name minimal_client_async."
#          "The constructor definition creates a client with the same type and name as the service node.
#          The type and name must match for the client and service to be able to communicate."
#
# Prof Riek Notes: You can call this method whatever you like, this is just the modified ROS tutorial code.
######
class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(GoPupper, 'pup_command')

        # GPIO pin numbers on RPi wired to the touch sensors
        self.front_pin = 6
        self.left_pin = 3
        self.right_pin = 16

        # BCM pin number scheme
        GPIO.setmode(GPIO.BCM)

        # Configure the touch sensor pins as input pins
        GPIO.setup(self.front_pin, GPIO.IN)
        GPIO.setup(self.left_pin, GPIO.IN)
        GPIO.setup(self.right_pin, GPIO.IN)

        # "The while loop in the constructor checks if a service matching the type and name of the client
        # is available once a second."
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # "Finally it creates a new request object."
        self.req = GoPupper.Request()

    ###
    # Name: sensor_data
    # Purpose: Read input from touch sensors and convert into movement commands
    # Arguments: self (reference the current class)
    ####
    def sensor_data(self):
        # Reading pins: 0 if touched, 1 if not touched
        front = GPIO.input(self.front_pin)
        left = GPIO.input(self.left_pin)
        right = GPIO.input(self.right_pin)

        # Set the image/gaze cue on the display based on which sensor is triggered
        # Return corresponding movement command to send to the GoPupper service
        # If none of the sensors were triggered, returns None
        if front == 0:
            self.setImage("img/forward.png")
            return "move_forward"
        if left == 0:
            self.setImage("img/left.png")
            return "move_left"
        if right == 0:
            self.setImage("img/right.png")
            return "move_right"
        
        return None

    ###
    # Name: send_move_request
    # Purpose: send_move_request method, send request and spin until receive response or fail
    # Arguments:  self (reference the current class), move_command (the command we plan to send to the server)
    #####
    def send_move_request(self, move_command):
        self.req = GoPupper.Request()
        self.req.command = move_command
        print("In send_move_request, command is: %s" % self.req.command)
        self.future = self.cli.call_async(self.req)  # send the command to the server
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    ###
    # Name: setImage
    # Purpose: Resize an image given the image URL and push it to the pupper's display
    # Referencing the downloaded sample display code (pupper_display_test.py)
    # Arguments: img_path (path to the source image)
    ####
    def setImage(self, img_path):
        disp = Display()
        MAX_WIDTH = 320

        imgFile = Image.open(img_path)
        if imgFile.format == 'PNG':
            if imgFile.mode != 'RGBA':
                imgFile = imgFile.convert("RGBA")

        imgFile = resizeimage.resize_width(imgFile, MAX_WIDTH)

        newFileLoc = 'img/face.png'

        imgFile.save(newFileLoc, imgFile.format)
        disp.show_image(newFileLoc)

###
# Name: Main
# Purpose: "Constructs a MinimalClientAsync object, sends the request using 
#           the detected sensor inputs, and logs the results."
#####
def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()

    # Variable tracking last movement command sent
    last_state = None

    # Variable tracking last time a command was sent
    last_time = 0

    # Time buffer to prevent the same state from being spammed if touch sensor is held down
    cooldown = 1.0

    # This spins up a client node, checks if it's done, throws an exception of there's an issue
    # (Probably a bit redundant with other code and can be simplified. But right now it works, so ¯\_(ツ)_/¯)
    while rclpy.ok():
        # Retrieve current state (movement command) based on touch sensor input
        state = minimal_client.sensor_data()

        if state is not None:
            current_time = time.time() # Get current time

            # If a new state is triggered or the same sensor has been held down for over the cooldown time,
            # send a new movement command to the service
            if state != last_state or current_time - last_time > cooldown:
                print("Send command: %s" % state)

                # Send command to the GoPupper service (sends Twist message to the robot)
                minimal_client.send_move_request(state)

                # Update the last state and time for the most recent command sent
                last_state = state
                last_time = current_time
        else:
            # If no sensors are triggered, set display to the idle image and update the last state detected
            if last_state is not None:
                minimal_client.setImage("img/idle.png")
                last_state = None

        rclpy.spin_once(minimal_client, timeout_sec=0.1)
    
    # Clean up GPIO pins safely
    GPIO.cleanup()

    # Destroy node and shut down cleanly even if interrupted
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
