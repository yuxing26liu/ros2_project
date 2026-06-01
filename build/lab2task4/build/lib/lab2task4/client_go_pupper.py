########
# Name: client_go_pupper
#
# Purpose: Go Pupper Client. Sample client code which will communicate with the GoPupper service by
#          passing along command line arguments form the user. (e.g., move_forward, move_backward, etc)
#
# Usage: First launch the service (see lab/file). Then you can run the client like this:
#        ros2 run go_pupper_srv client move_forward
#
# Author: Prof. Riek <lriek@ucsd.edu>
#
# Acknowledgements: Used some code from ROS 2 Tutorials and MangDang's ROS git repo 
#  https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html
#  https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html#test-the-new-interfaces
#  https://github.com/mangdangroboticsclub/mini_pupper_ros/blob/ros2-dev/mini_pupper_dance/mini_pupper_dance/dance_server.py
#
# Date: 30 Apr 2024
########

# Import the ROS2 interface we wrote, called GoPupper. This specifies the message type.
from pupper_interfaces.srv import GoPupper

# Lets us read arguments from the command line as needed
import sys

# Packages to let us create nodes and spin them up
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
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
        self.front_pin = 6
        self.left_pin = 3
        self.right_pin = 16
        # self.back_pin = 2

        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.front_pin, GPIO.IN)
        GPIO.setup(self.left_pin, GPIO.IN)
        GPIO.setup(self.right_pin, GPIO.IN)

        # "The while loop in the constructor checks if a service matching the type and name of the client 
        # is available once a second." 
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # "Finally it creates a new request object.""
        self.req = GoPupper.Request()

    def sensor_data(self):
        front = GPIO.input(self.front_pin)
        left = GPIO.input(self.left_pin)
        right = GPIO.input(self.right_pin)

        if front == 0:
            return "move_forward"
        if left == 0:
            return "move_left"
        if right == 0:
            return "move_right"
        else:
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
#           the passed-in command-line arguments, and logs the results."
#####
def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()

    last_state = None
    last_time = 0
    cooldown = 1.0
    # For testing - we're getting a move command string from the command line
    # cmd = sys.argv[1]

    # debug - comment in/our as needed
    #print("In client, got this command: %s" % cmd)

    # Call send move request (which sends cmd to server)
    # minimal_client.send_move_request(cmd)

    # This spins up a client node, checks if it's done, throws an exception of there's an issue
    # (Probably a bit redundant with other code and can be simplified. But right now it works, so ¯\_(ツ)_/¯)
    while rclpy.ok():
        state = minimal_client.sensor_data()

        if state is not None:
            current_time = time.time()

            if state != last_state or current_time - last_time > cooldown:
                print("Send command: %s" % state)
                minimal_client.send_move_request(state)

                last_state = state
                last_time = current_time

        rclpy.spin_once(minimal_client, timeout_sec=0.1)

    # Destroy node and shut down
    GPIO.cleanup()
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
