#!/usr/bin/env python
# camera.py
"""
Provides support for Picamera v3. Limited functionality: used to find a blue 
line and return error away from center of frame.
"""
import os
import time

import cv2
from libcamera.controls import AfModeEnum  # type:ignore
from numpy import ndarray
from picamera2 import Picamera2  # type:ignore

__author__ = "Ben Kraft"
__copyright__ = "None"
__credits__ = "Ben Kraft"
__license__ = "MIT"
__version__ = "1.2"
__maintainer__ = "Ben Kraft"
__email__ = "benjamin.kraft@tufts.edu"
__status__ = "Prototype"

picam = Picamera2()  # assigns camera variable
picam.set_controls({"AfMode": AfModeEnum.Continuous})  # sets auto focus mode
picam.start()  # activates camera


def main() -> None:
    """
    Runs default library acions.
    """
    time.sleep(2)

    iteration = 0
    # Acquires object from command line input
    object = input("\nEnter an object to be sampled: ")
    destination_path = os.path.join("ml", "Samples", object)
    print(f"\nTaking picture samples for export in: [ {destination_path} ]\n")

    while True:
        # Input advance
        input("[Enter] to advance: ")
        # Takes picture and converts to color space
        image_data = take_picture()
        bgr_space = cv2.cvtColor(image_data, cv2.COLOR_RGB2BGR)

        # Makes directory if does not already exist
        if not os.path.isdir(destination_path):
            os.mkdir(destination_path)
        # Creates a path for image to be written to
        image_path = os.path.join(destination_path, f"sample{iteration}.jpg")
        cv2.imwrite(image_path, bgr_space)
        print(f"Wrote image to: [ {image_path} ]")
        # Displays image
        cv2.imshow("Latest Image", bgr_space)
        cv2.waitKey(1)

        iteration += 1


def take_picture(display: bool = False) -> ndarray:
    """
    Takes a picture and returns it as a numpy array.
    """
    image = picam.capture_array("main")

    if display:
        # Converts the image from RGB to BGR format
        image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        # Display the image
        cv2.imshow("Raw Image", image_bgr)
        cv2.waitKey(1)
        # cv2.destroyAllWindows()

    return image


def crop(
    image_data: ndarray, size: tuple[int, int], position: tuple[int, int]
) -> ndarray:
    """
    Returns data cropped to size and at position.
    """
    return image_data[
        position[0] : (position[0] + size[0]),
        position[1] : (position[1] + size[1]),
    ]


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("")
