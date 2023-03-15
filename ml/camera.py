#!/usr/bin/env python
# camera.py
"""
Provides support for Picamera v3. Limited functionality: used to find a blue 
line and return error away from center of frame.
"""
import os
import sys
import time

import cv2
from libcamera import controls  # type:ignore
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
picam.set_controls({"AfMode": controls.AfModeEnum.Continuous})  # sets auto focus mode
picam.start()  # activates camera


def main() -> None:
    """
    Runs default library acions.
    """
    time.sleep(1.5)

    do_display = not "-d" in sys.argv

    iteration = 0
    # Acquires object from command line input
    object = input("\nEnter an object to be sampled: ")
    destination_path = os.path.join("Samples", object)
    print(f"\nTaking picture samples for export in: [ {destination_path} ]\n")

    while True:
        # Input advance
        input("[Enter] to advance: ")
        # Takes picture and converts to color space
        _, image_bgr = take_picture(do_display)
        # Makes directory if does not already exist
        if not os.path.isdir(destination_path):
            os.mkdir(destination_path)
        # Creates a path for image to be written to
        image_path = os.path.join(destination_path, f"sample{iteration}.jpg")
        cv2.imwrite(image_path, image_bgr)
        print(f"Wrote image to: [ {image_path} ]")

        iteration += 1


def take_picture(display: bool = False) -> tuple[ndarray, ndarray]:
    """
    Takes a picture and returns it as a numpy array.
    """
    image_data = picam.capture_array("main")
    # Converts the image from RGB to BGR format for displays
    image_bgr = cv2.cvtColor(image_data, cv2.COLOR_RGB2BGR)

    if display:
        # Display the image
        cv2.imshow("Raw Image", image_bgr)
        cv2.waitKey(1)

    return image_data, image_bgr


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
