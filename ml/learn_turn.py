import os
import time

import camera
import cv2
import numpy as np
from keras.models import Model, load_model
from numpy import ndarray
from pyparsing import Any

# Pluralsight.com

# Disable scientific notation for clarity
np.set_printoptions(suppress=True)

IMAGE_SIZE = 224

LEFT = -1
RIGHT = 1

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
    """
    Runs default actions.
    """
    time.sleep(2)

    model = ImageModel("keras_model.h5", tuple(CLASS_DIRECTIONS.keys()))

    while True:

        _, image_data = camera.take_picture()

        predicted_class = model.predict_class(image_data)

        print(f"Predicted object: {predicted_class}")
        print(f"Predicted direction: {CLASS_DIRECTIONS[predicted_class]}")


class ImageModel:
    """
    Allows for use of h5 machine learning model for predictions.
    """

    def __init__(self, filepath: str, classes: tuple[Any]) -> None:
        model: Model = load_model(filepath)  # type:ignore
        model.compile(
            loss="binary_crossentropy",
            optimizer="rmsprop",
            metrics=["accuracy"],
        )
        self.model = model
        self.classes = classes

    # def get_prediction(self):

    def predict_class(self, image_data: ndarray) -> Any:
        """
        Predicts which class the image file corresponds to. Returns an item
        from class list
        """
        # Acquires prediction confidences using model and preloaded data
        prediction_condfidences = self.model.predict(preload_data(image_data))
        # Finds index of maximum value of confidences
        prediction_index = np.argmax(prediction_condfidences)
        # Checks that class can be found in list
        if prediction_index >= len(self.classes):
            raise ValueError("Class list and prediction do not match!")
        # Returns class at index
        return self.classes[prediction_index]


def preload_data(image_data: ndarray, display: bool = False) -> ndarray:
    """
    Prepares data for use in prediciton model through cropping and
    normalization. Optional flag to display processed images.
    """
    resized_image = cv2.resize(
        image_data, (IMAGE_SIZE, IMAGE_SIZE), interpolation=cv2.INTER_AREA
    )

    normalized_image_array = (resized_image.astype(np.float32) / 127.0) - 1

    # Display the image if desired
    if display:
        cv2.imshow("Original Image", cv2.cvtColor(image_data, cv2.COLOR_BGR2HSV))
        cv2.imshow("Resized Image", resized_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # Return the normalized image as a numpy array of shape (1, 224, 224, 3)
    return np.expand_dims(normalized_image_array, axis=0)


if __name__ == "__main__":
    main()
