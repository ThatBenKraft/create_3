import os
import time

from ml import camera
from ml.modeling import ImageModel
from publishers import MidiPublisher, MotorPublisher
from subscribers import IRSubscriber

STOP_DISTANCE = 110
OBJECT_COUNT = 7

LEFT = 1
RIGHT = -1

CLASS_DIRECTIONS = {
    "elephant": LEFT,
    "mario": LEFT,
    "mug": RIGHT,
    "tractor": RIGHT,
    "kiwi": LEFT,
    "rubiks": RIGHT,
    "bear": LEFT,
}

WIN_SONG = os.path.join("music", "Test Robot Song.mid")
ALERT_SONG = os.path.join("music", "Alert Robot Sound.mid")

keras_filepath = os.path.join("ml", "keras_model.h5")
image_model = ImageModel(keras_filepath, tuple(CLASS_DIRECTIONS.keys()))
motors = MotorPublisher()
ir_sensors = IRSubscriber()
midi = MidiPublisher(WIN_SONG, ALERT_SONG)


def main():
    """
    Runs primary actions.
    """
    object_count = 0

    while True:
        # If number of objects has been reached, win
        if object_count >= OBJECT_COUNT:
            win_sequence(motors, midi)
            break
        # Get IR sensor readings
        average_distance = ir_sensors.get_average()
        print(f"Average distance: {average_distance}")
        # If distance is less than specified stopping:
        if average_distance >= STOP_DISTANCE:
            # Move back
            motors.move_distance(-3)
            time.sleep(1)
            # Take a picture
            print("Taking picture...")
            midi.play_sequence(ALERT_SONG)
            image_data = camera.take_picture()
            time.sleep(1)
            # Move forward
            motors.move_distance(3)
            # Predict class from image
            predicted_class = image_model.predict_class(image_data)
            midi.play_sequence(ALERT_SONG)
            predicted_direction = CLASS_DIRECTIONS[predicted_class]
            print(f"Predicted object: {predicted_class}")
            direction_name = "LEFT" if predicted_direction == 1 else "RIGHT"
            print(f"Predicted direction: {direction_name}")
            # Move in predicted direction
            motors.turn_direction(predicted_direction)
            # Increment number of objects seen
            object_count += 1
        # If no wall/object seen:
        else:
            # Move forward
            travel = 3 if average_distance < STOP_DISTANCE / 3 else 1
            motors.move_distance(travel)

        time.sleep(1)


def win_sequence(motors: MotorPublisher, midi: MidiPublisher) -> None:

    print("MAZE COMPLETED!!!")

    motors.move_distance(3)

    time.sleep(1)

    midi.play_sequence(WIN_SONG)

    for _ in range(8):
        motors.turn_direction(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
