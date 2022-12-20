## Image analysis testing with dumb octopus
import cv2 as cv
import time
import numpy as np

def takePicture():
    capture = cv.VideoCapture(0)

    isTrue, frame = capture.read()
    while not np.any(frame>0):
        isTrue, frame = capture.read()

    cv.imshow('Image', frame)

    capture.release()

    return frame

## take pictures of something
takePicture()
