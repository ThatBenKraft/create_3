#!/usr/bin/env python
# camera.py
"""
Provides support for Picamera v3. Limited functionality: used to find a blue 
line and return error away from center of frame.
"""
import time

import cv2
import numpy as np
from libcamera import controls  # type:ignore
from numpy import ndarray
from picamera2 import Picamera2  # type:ignore

__author__ = "Einsteinium Studios, Briana Bouchard"
__copyright__ = "None"
__credits__ = "Briana Bouchard, Kenneth Siu, Ben Kraft"
__license__ = "MIT"
__version__ = "1.1"
__maintainer__ = "Ben Kraft"
__email__ = "benjamin.kraft@tufts.edu"
__status__ = "Prototype"

picam = Picamera2()  # assigns camera variable
picam.set_controls({"AfMode": controls.AfModeEnum.Continuous})  # sets auto focus mode
picam.start()  # activates camera

FULL_WIDTH = 640
FULL_HEIGHT = 480

CROP_WIDTH = 600
CROP_HEIGHT = 400

X_OFFSET = (FULL_WIDTH - CROP_WIDTH) // 2
Y_OFFSET = (FULL_HEIGHT - CROP_HEIGHT) // 2

CENTER_CROP_POSITION = (CROP_WIDTH // 2, CROP_HEIGHT // 2)


def main() -> None:
    """
    Runs default library acions.
    """
    time.sleep(1)

    while True:

        print(take_picture(display=True))

        time.sleep(1)


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
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return image


if __name__ == "__main__":
    main()
