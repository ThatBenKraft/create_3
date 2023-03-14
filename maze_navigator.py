import os
import sys
import time

import cv2

from publishers import MotorPublisher

# from learn_turn import ImageModel, preload_data


TURN_DEGREE_FACTOR = 0.8
MOVE_DISTANCE_FACTOR = 5.0

TURN_LOOP_COUNT = 4

MODEL_FILEPATH = "keras_model.h5"
SAMPLE_IMAGE_NAME = "sight_sample.jpg"

LEFT = -1
RIGHT = 1

OBJECTS = {
    "foo": RIGHT,
    "bar": LEFT,
    "baz": LEFT,
}


def main():
    """
    Runs default navigation actions.
    """

    # Defines a motor publisher
    publisher = MotorPublisher()

    time.sleep(1)

    turn_direction(publisher, LEFT)

    time.sleep(1)

    turn_direction(publisher, RIGHT)

    # try:
    #     model = ImageModel(MODEL_FILEPATH, OBJECTS)
    #     WRITE_IMAGE = True

    #     while True:
    #         # Takes picture and saves it to disk
    #         image_data = camera.take_picture(True)
    #         # Writes image to disk if specified
    #         if WRITE_IMAGE:
    #             cv2.imwrite(SAMPLE_IMAGE_NAME, image_data)
    #         # Creates a model
    #         ### TBD CONDITION TO ASSES IMAGES ###
    #         turn = True

    #         if turn:
    #             # Acquires direction from model
    #             direction = model.predict_direction(image_data)
    #             # Turns that way
    #             turn_direction(publisher, direction)

    #         time.sleep(5)

    # except KeyboardInterrupt:

    #     print("\nCaught Keyboard Interrupt")

    publisher.destroy_node()


def turn_direction(publisher: MotorPublisher, direction: int):
    """
    Turns robot in specified direction.
    """
    for _ in range(TURN_LOOP_COUNT):
        # Gives no linear and scaled angular velocity
        publisher.publish_velocities(0.0, direction * TURN_DEGREE_FACTOR)
        time.sleep(0.5)


def move_forward(publisher, distance: int):
    """
    Moves robot forward specified amount.
    """
    for _ in range(distance):
        # Gives scaled linear and no angular velocity
        publisher.publish_velocities(MOVE_DISTANCE_FACTOR, 0.0)
        time.sleep(0.5)


if __name__ == "__main__":
    main()
