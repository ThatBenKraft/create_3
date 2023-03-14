import os

import cv2
import numpy as np
from keras.models import Model, load_model
from numpy import ndarray

# Pluralsight.com

# Disable scientific notation for clarity
np.set_printoptions(suppress=True)

IMAGE_SIZE = 224

LEFT = -1
RIGHT = 1

CLASS_DIRECTIONS = {
    "foo": RIGHT,
    "bar": LEFT,
    "baz": LEFT,
}


def main():
    """
    Runs default actions.
    """
    filepaths = (
        os.path.join("Samples", "nothing.jpg"),
        os.path.join("Samples", "case.jpg"),
        os.path.join("Samples", "bottle.jpg"),
    )
    # Creates image model
    model = ImageModel("keras_model.h5", CLASS_DIRECTIONS)
    # For each filepath
    for filepath in filepaths:
        # Checks for errors
        if not os.path.isfile(filepath):
            raise FileNotFoundError(f"Image file {filepath} does not exist")
        image = cv2.imread(filepath)
        if image is None:
            raise ValueError(f"Unable to load image file {filepath}")
        # Acquires data from image as array
        image_data = np.array(image)
        # Predicts direction from image
        print(model.predict_direction(image_data))


class ImageModel:
    """
    Allows for use of h5 machine learning model for predictions.
    """

    def __init__(self, filepath: str, class_directions: dict[str, int]) -> None:
        model: Model = load_model(filepath)  # type:ignore
        model.compile(
            loss="binary_crossentropy",
            optimizer="rmsprop",
            metrics=["accuracy"],
        )
        self.model = model
        self.class_directions = class_directions
        # Makes a list of classes from keys to allow for indexing
        self.classes = tuple(class_directions.keys())

    def predict_direction(self, image_data: ndarray) -> int:
        """
        Predicts which direction the image file corresponds to. Returns an
        integer representing a direction.
        """
        resized_data = preload_data(image_data)
        # Makes a prediction with model
        prediction: ndarray = self.model.predict(resized_data)
        # Finds index of greatest probability
        # max_confidence =
        predicted_class_index = int(np.argmax(prediction))
        # Acquires corresponding direction from class list
        return self.class_directions[self.classes[predicted_class_index]]


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
