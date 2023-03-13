import time

import cv2
from learn_turn import ImageModel

import camera
from simple_publisher import MotorPublisher

TURN_DEGREE_FACTOR = 1
MOVE_DISTANCE_FACTOR = 5

TURN_LOOP_COUNT = 3

MODEL_FILEPATH = "keras_model.h5"
IMAGE_NAME = "sight_sample.jpg"

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

    try:
        model = ImageModel(MODEL_FILEPATH, OBJECTS)

        while True:
            # Takes picture and saves it to disk
            image = camera.take_picture()
            cv2.imwrite(IMAGE_NAME, image)
            # Creates a model
            ### TBD CONDITION TO ASSES IMAGES ###
            turn = True

            if turn:
                # Acquires direction from model
                direction = model.predict_direction(IMAGE_NAME)
                # Turns that way
                turn_direction(publisher, direction)

            time.sleep(5)

    except KeyboardInterrupt:

        print("\nCaught Keyboard Interrupt")

        publisher.destroy_node()


def turn_direction(publisher, direction: int):
    """
    Turns robot in specified direction.
    """
    for _ in range(TURN_LOOP_COUNT):
        # Gives no linear and scaled angular velocity
        publisher.publish_velocities(0, direction * TURN_DEGREE_FACTOR)

        time.sleep(0.5)


def move_forward(publisher, distance: int):
    """
    Moves robot forward specified amount.
    """
    for _ in range(distance):
        # Gives scaled linear and no angular velocity
        publisher.publish_velocities(MOVE_DISTANCE_FACTOR, 0)

        time.sleep(0.5)


if __name__ == "__main__":
    main()
