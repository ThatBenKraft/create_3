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
    filenames = (
        os.path.join("Samples", "nothing.jpg"),
        os.path.join("Samples", "case.jpg"),
        os.path.join("Samples", "bottle.jpg"),
    )

    # Load the model
    model: Model = load_model("keras_model.h5")  # type:ignore
    model.compile(
        loss="binary_crossentropy",
        optimizer="rmsprop",
        metrics=["accuracy"],
    )

    for filename in filenames:

        print(predict_direction(model, filename, CLASS_DIRECTIONS))


def predict_direction(
    model: Model, filename: str, class_directions: dict[str, int]
) -> int:
    """
    Uses model to predict which direction the image file corresponds to.
    Returns an integer representing a direction.
    """
    # Acquires image info in correct format
    image_data = load_image(filename)
    # Makes a prediction with model
    prediction: ndarray = model.predict(image_data)
    # Finds index of greatest probability
    predicted_class_index = int(np.argmax(prediction))
    # Makes a list of classes from keys to allow for indexing
    classes = tuple(class_directions.keys())
    # Acquires corresponding direction from class list
    return class_directions[classes[predicted_class_index]]


def load_image(filename: str, display: bool = False) -> np.ndarray:
    """
    Prepares image for use in prediciton model through cropping and
    normalization. Optional flag to display processed images.
    """
    # Load the image using OpenCV
    if not os.path.isfile(filename):
        raise FileNotFoundError(f"Image file {filename} does not exist")
    image = cv2.imread(filename)
    if image is None:
        raise ValueError(f"Unable to load image file {filename}")

    # Resize the image to the desired size and crop from the center
    resized_image = cv2.resize(
        image, (IMAGE_SIZE, IMAGE_SIZE), interpolation=cv2.INTER_AREA
    )

    # Convert the image to a numpy array and normalize
    normalized_image_array = (resized_image.astype(np.float32) / 127.0) - 1

    # Display the image if desired
    if display:
        cv2.imshow("Original Image", image)
        cv2.imshow("Resized Image", resized_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # Return the normalized image as a numpy array of shape (1, 224, 224, 3)
    return np.expand_dims(normalized_image_array, axis=0)


if __name__ == "__main__":
    main()
