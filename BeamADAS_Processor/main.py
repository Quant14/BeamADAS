# This script will process the sensor data coming from the host and send back a response
# including the adjustments that need to be made to the user input

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import glob
import os

def bird_eye_view(img):
    img_size = (img.shape[1], img.shape[0])
    offset = 450

    # Source points taken from images with straight lane lines, these are to become parallel after the warp transform
    # Needs to be updated with accurate lane coordinates
    src = np.array([
        (190, 720), # bottom-left corner
        (596, 447), # top-left corner
        (685, 447), # top-right corner
        (1125, 720) # bottom-right corner
    ], dtype='f')
    dst = np.array([
        [offset, img_size[1]],             # bottom-left corner
        [offset, 0],                       # top-left corner
        [img_size[0]-offset, 0],           # top-right corner
        [img_size[0]-offset, img_size[1]]  # bottom-right corner
    ], dtype='f')

    # Calculate the transformation matrix and it's inverse transformation
    M = cv2.getPerspectiveTransform(src, dst)
    M_inv = cv2.getPerspectiveTransform(dst, src)
    warped = cv2.warpPerspective(img, M, img_size)
   
    return warped, M_inv

def binary_threshold(img):
    # Transform to gray scale
    gray = cv2.cvtColor(img, cv2.COLOR_BAYER_BGGR2GRAY)
    # Apply sobel in x direction (detect vertical lines)
    sobelx = cv2.Sobel(gray, 6, 1, 0)
    # todo

img = cv2.imread('C:\\Users\\RUI1SF\\Pictures\\highway_hood.png')
bird_eye_view(img)