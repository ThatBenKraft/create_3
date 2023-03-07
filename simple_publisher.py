"""
by Maddie Pero 

In this example we will get data from Airtable.
"""
import time

import rclpy  # imports rclpy client library # type: ignore
import requests  # you may need to run 'pip install requests' to install this library
from geometry_msgs.msg import Twist  # type:ignore
from rclpy.node import Node  # type: ignore

""" This function makes a get request to the airtable API which will tell us how fast to spin the wheels"""

BASE_ID = "appWNQwSNORmWZJQH"
TABLE_NAME = "Robot Data"
API_KEY = "keyquD11xV0tMOZxM"

URL = (
    "https://api.airtable.com/v0/" + BASE_ID + "/" + TABLE_NAME + "?api_key=" + API_KEY
)

"""
The get request data comes in as a json package. We will convert this json package to a python dictionary so that it can be parsed
"""

# And this is what you call excessive type-hinting
def get_api_velocities() -> tuple[dict[str, float], ...]:
    """
    Acquires X, Y, and Z components of linear and rotational velocities from
    API.
    """
    # Acquires raw data from url
    raw_records: list[dict[str, dict[str, str]]] = requests.get(URL).json()["records"]
    # print(data)
    new_records = []
    for record in raw_records:
        # Grads record field and gets rid of name
        important_data = record["fields"]
        important_data.pop("Name")
        # Creates new dictionary with new
        new_records.append({k: float(v) for k, v in important_data.items()})

    return tuple(new_records)


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

        # Set delay in seconds
        # TIMER_PERIOD = 1

        # Creates a timer that triggers a callback function after the set timer_period
        # self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        # Sets initial counter to zero
        self.counter = 0

    def publish_velocities(self):

        linear, rotational = get_api_velocities()

        curr_twist: Twist = Twist()
        curr_twist.linear.x = linear["X"]
        curr_twist.linear.y = 0.0
        curr_twist.linear.z = 0.0

        curr_twist.angular.x = 0.0
        curr_twist.angular.y = 0.0
        curr_twist.angular.z = rotational["Z"]

        # Publishes `msg` to topic
        self.publisher_.publish(curr_twist)

        # Prints `msg.data` to console
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
