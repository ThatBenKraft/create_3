"""
by Maddie Pero 

In this example we will get data from Airtable.
"""
import time

import base_api
import rclpy  # imports rclpy client library # type: ignore
from geometry_msgs.msg import Twist  # type:ignore
from rclpy.node import Node  # type: ignore


# Creates SimplePublisher class which is a subclass of Node
class SimplePublisher(Node):
    """
    Allows for node to push information on motor channel.
    """

    # Defines class constructor
    def __init__(self) -> None:

        # Initializes and gives Node the name simple_publisher and inherits the Node class's attributes by using 'super()'
        super().__init__("simple_publisher")

        MOTOR_CHANNEL = "cmd_vel"

        # Creates a publisher based on the message type "String" that has been imported from the std_msgs module above
        self.publisher_ = self.create_publisher(Twist, MOTOR_CHANNEL, 10)
        # Sets initial counter to zero
        self.counter = 0

    def publish_velocities(self):
        """
        Publishes velocity values from API to motor channel.
        """
        # Gets velocity components from API
        linear, angular = base_api.get_velocities("Linear", "Angular")
        # Defines velocity factors
        LINEAR_FACTOR = 0.2
        ROTATIONAL_FACTOR = -1.5

        # Creates a Twist object
        new_twist = Twist()
        # Assigns linear velocity components
        new_twist.linear.x = linear * LINEAR_FACTOR
        new_twist.linear.y = 0.0
        new_twist.linear.z = 0.0
        # Assigns angular velocity components
        new_twist.angular.x = 0.0
        new_twist.angular.y = 0.0
        new_twist.angular.z = angular * ROTATIONAL_FACTOR

        # Publishes twist to topic
        self.publisher_.publish(new_twist)
        # Prints counter to console
        self.get_logger().info(f"Publish #{self.counter + 1}")
        # Increments counter
        self.counter += 1


def main() -> None:
    # Initializes ROS2 communication and allows Nodes to be created
    rclpy.init(args=None)

    # Creates the SimplePublisher Node
    simple_publisher = SimplePublisher()

    try:
        # Spins the Node to activate the callbacks
        # rclpy.spin(simple_publisher)

        while True:

            simple_publisher.publish_velocities()

            time.sleep(1)

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
