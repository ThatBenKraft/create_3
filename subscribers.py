"""
Allows for use of ROS subscriber nodes. Includes IR subscriber.
"""

import time
from statistics import mean

import rclpy  # type: ignore
from irobot_create_msgs.msg import IrIntensityVector  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.qos import qos_profile_sensor_data  # type: ignore

__author__ = "Ben Kraft"
__copyright__ = "None"
__credits__ = "Ben Kraft, Kate Wujciak"
__license__ = "MIT"
__version__ = "1.0"
__maintainer__ = "Ben Kraft"
__email__ = "benjamin.kraft@tufts.edu"
__status__ = "Prototype"


class IRSubscriber(Node):
    """
    The IRSubscriber class is created which is a subclass of Node.
    The Node is subscribing to the /ir_intensity topic.
    """

    def __init__(self):
        """
        The following line calls the Node class' constructor and declares a node name,
        which is 'IR_subscriber' in this case.
        """
        super().__init__("ir_subscriber")
        """
        This line indicates that the node is subscribing to the IrIntensityVector
        type over the '/ir_intensity' topic. 
        """
        self.subscription = self.create_subscription(
            IrIntensityVector,
            "/ir_intensity",
            self.listener_callback,
            qos_profile_sensor_data,
        )

    def listener_callback(self, msg: IrIntensityVector):
        """
        The subscriber's callback listens and as soon as it receives the message,
        this function runs.
        This callback function is basically printing what it hears. It runs the data
        it receives in your terminal (msg).
        """
        self.average_reading = mean(sensor.value for sensor in msg.readings[2:5])

    def get_average(self, display: bool = False) -> float:
        # Takes the average of the three middle sensor readings

        rclpy.spin_once(self)

        new_reading = round(self.average_reading, 1)

        if display:
            print(new_reading)

        return new_reading


def main() -> None:
    # Initializes the rclpy library
    rclpy.init()
    # Creates the node
    ir_sensor = IRSubscriber()
    # The node is then "spun" so its callbacks are called.
    try:
        while True:
            # rclpy.spin_once(IR_subscriber)

            ir_sensor.get_average(True)

            time.sleep(1)
    except KeyboardInterrupt:
        print("\nCaught keyboard interrupt")
    finally:
        """Destroying the node acts as a "reset" so we don't run into
        any problems when we try and run again"""
        print("Done")
        ir_sensor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
