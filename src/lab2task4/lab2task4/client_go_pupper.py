########
# Name: client_go_pupper
#
# Purpose: Go Pupper Client. Client code which will communicate with the GoPupper service by
#          detecting touch sensor info from the user (e.g., front, left, right sensor) and
#          passing along the corresponding movement command to the Go Pupper Service to
#          control the robot's movement.
#
# Usage: First launch the service (see lab/file). Then you can run the client like this:
#        ros2 run lab2task4 client
#
# Author: Terri Tai <y2tai@ucsd.edu> Yuxing Liu <yul269@ucsd.edu>  Christina Xu <chx029@ucsd.edu> 
#
# Acknowledgements: Used some code from ROS 2 Tutorials and MangDang's ROS git repo 
#  https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html
#  https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html#test-the-new-interfaces
#  https://github.com/mangdangroboticsclub/mini_pupper_ros/blob/ros2-dev/mini_pupper_dance/mini_pupper_dance/dance_server.py
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

###
# Name: Minimal Client Async
#
# Purpose: "The MinimalClientAsync class constructor initializes the node with the name minimal_client_async. "
#          "The constructor definition creates a client with the same type and name as the service node. 
#          The type and name must match for the client and service to be able to communicate."
#
# Prof Riek Notes: You can call this method whatever you like, this is just the modified ROS tutorial code. 
######
class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        #super().__init__('client_go_pupper')
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

        # "Finally it creates a new request object.""
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

        # Return corresponding movement command to send to the GoPupper service
        # If none of the sensors were triggered, returns None
        if front == 0:
            return "move_forward"
        if left == 0:
            return "move_left"
        if right == 0:
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

        # If no sensors are triggered, do nothing
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

        rclpy.spin_once(minimal_client, timeout_sec=0.1)

    # Clean up GPIO pins safely
    GPIO.cleanup()

    # Destroy node and shut down
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
