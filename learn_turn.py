import os
import time

from ml import camera
from ml.modeling import ImageModel
from publishers import MotorPublisher
from subscribers import IRSubscriber

STOP_DISTANCE = 110

LEFT = 1
RIGHT = -1

CLASS_DIRECTIONS = {
    "elephant": LEFT,
    "mario": LEFT,
    "mug": RIGHT,
    "tractor": RIGHT,
    "kiwi": LEFT,
    "rubiks": RIGHT,
    "bear": RIGHT,
}


def main():

    motors = MotorPublisher()
    ir_sensors = IRSubscriber()
    keras_filepath = os.path.join("ml", "keras_model.h5")
    image_model = ImageModel(keras_filepath, tuple(CLASS_DIRECTIONS.keys()))
    print("\nInitiating movement sequence. . .\n")

    while True:

        motors.move_distance(1)

        average_distance = ir_sensors.get_average()
        print(f"AVERAGE DISTANCE: {average_distance}")

        if average_distance >= STOP_DISTANCE:

            motors.move_distance(-2)

            time.sleep(0.75)

            print("TAKING PICTURE")
            image_data = camera.take_picture()

            time.sleep(0.75)

            motors.move_distance(2)

            predicted_class = image_model.predict_class(image_data)

            predicted_direction = CLASS_DIRECTIONS[predicted_class]

            print(f"Predicted object: {predicted_class}")
            print(f"Predicted direction: {predicted_direction}")

            motors.turn_direction(predicted_direction)

        time.sleep(2)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
