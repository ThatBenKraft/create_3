"""
Acquires information from Airtable API and publishes to Create3 motor channel.
"""

import time

import rclpy  # imports rclpy client library # type: ignore
from geometry_msgs.msg import Twist  # type:ignore
from rclpy.node import Node  # type: ignore

import base_api

__author__ = "Ben Kraft"
__copyright__ = "None"
__credits__ = "Ben Kraft, Maddie Pero"
__license__ = "MIT"
__version__ = "1.0"
__maintainer__ = "Ben Kraft"
__email__ = "benjamin.kraft@tufts.edu"
__status__ = "Prototype"


class VelocityPublisher(Node):
    """
    Allows for node to push information on motor channel.
    """

    # Defines class constructor
    def __init__(self) -> None:
        # Initializes and gives Node the name simple_publisher and inherits the Node class's attributes by using 'super()'
        super().__init__("simple_publisher")
        # Creates a publisher based on the message type "String" that has been imported from the std_msgs module above
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        # Sets initial counter to zero
        self.counter = 0

    def publish_velocities(
        self, linear: float, angular: float, display_count: bool = False
    ):
        """
        Publishes velocity values from API to motor channel. Optional flag for
        displaying publish count.
        """

        # Creates a Twist object
        new_twist = Twist()
        # Assigns linear velocity components
        new_twist.linear.x = linear
        new_twist.linear.y = 0.0
        new_twist.linear.z = 0.0
        # Assigns angular velocity components
        new_twist.angular.x = 0.0
        new_twist.angular.y = 0.0
        new_twist.angular.z = angular

        # Publishes twist to topic
        self.publisher_.publish(new_twist)
        # Prints counter to console
        if display_count:
            self.get_logger().info(f"Publish #{self.counter + 1}")
        # Increments counter
        self.counter += 1


def main() -> None:
    """
    Runs default publisher actions using Airtable API.
    """
    # Initializes ROS2 communication and allows Nodes to be created
    rclpy.init(args=None)
    # Creates the SimplePublisher Node using the motor control channel
    simple_publisher = VelocityPublisher()
    try:
        # Defines factors
        LINEAR_FACTOR = 0.2
        ANGULAR_FACTOR = -1
        WAIT_TIME = 2

        while True:
            # Gets velocity components from API
            linear, angular = base_api.get_velocities("Linear", "Angular")
            # Publishes API velocities to motors
            simple_publisher.publish_velocities(
                linear=linear * LINEAR_FACTOR,
                angular=angular * ANGULAR_FACTOR,
                display_count=True,
            )
            # Waits
            time.sleep(WAIT_TIME)
    # Stops the code if CNTL-C is pressed on the keyboard
    except KeyboardInterrupt:
        print("\nCaught Keyboard Interrupt")
        # Destroys the node that was created
        simple_publisher.destroy_node()
        # Shuts down rclpy
        rclpy.shutdown()


if __name__ == "__main__":
    # Runs the main function
    main()
