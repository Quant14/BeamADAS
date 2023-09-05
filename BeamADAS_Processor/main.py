# This script will process the sensor data coming from the host and send back a response
# including the adjustments that need to be made to the user input

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import glob
import os

img = cv2.imread('.\\test_curve1.png')

def birdeye_view(img):
    img_size = (img.shape[1], img.shape[0])
    offset = 300

    # Source points taken from images with straight lane lines, these are to become parallel after the warp transform
    # Needs to be updated with accurate lane coordinates
    src = np.array([
        (430, 520), # bottom-left corner
        (630, 360), # top-left corner
        (645, 360), # top-right corner
        (875, 520) # bottom-right corner
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

# Test 1 - successful
birdeye_img, M_inv = birdeye_view(img)
plt.imsave('birdeye_img.png', birdeye_img)

def binary_threshold(img):
    # Transform to gray scale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Apply sobel in x direction (detect vertical lines)
    sobelx = cv2.Sobel(gray, 6, 1, 0)
    abs_sobelx = np.absolute(sobelx)

    scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
    sx_binary = np.zeros_like(scaled_sobel)
    
    sx_binary[(scaled_sobel >= 120) & (scaled_sobel <= 255)] = 1

    # Detect white pixels
    white_binary = np.zeros_like(gray)
    white_binary[(gray > 160) & (gray <= 255)] = 1

    # Convert to HLS
    hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
    H = hls[:,:,0]
    S = hls[:,:,2]

    # Detect high saturation
    sat_binary = np.zeros_like(S)
    sat_binary[(S > 50) & (S <= 255)] = 1

    # Detect yellow pixels
    hue_binary = np.zeros_like(H)
    hue_binary[(H > 15) & (H <= 25)] = 1

    # Combine results
    binary_1 = cv2.bitwise_or(sx_binary, white_binary)
    # binary_2 = cv2.bitwise_or(hue_binary, sat_binary)
    
    # return cv2.bitwise_or(binary_1, binary_2)
    return binary_1

# Test 2 - successful
binary = binary_threshold(img)
out_img = np.dstack((binary, binary, binary))*255 # type: ignore
plt.imsave('binary_img_no_yellow.png', out_img)

def detect_lane_lines(binary_birdeye):
    # Make histogram of bottom half of img
    histogram = np.sum(binary_birdeye[binary_birdeye.shape[0]//2:,:], axis=0)

    # Find lanes starting points
    midpoint = np.int32(histogram.shape[0]//2)
    left_base = np.argmax(histogram[:midpoint])
    right_base = np.argmax(histogram[midpoint:]) + midpoint

    nwindows = 9
    margin = 100
    minpix = 50

    window_h = np.int32(binary_birdeye.shape[0]//nwindows)

    nonzero = binary_birdeye.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    left_curr = left_base
    right_curr = right_base

    left_lane = []
    right_lane = []

    for window in range(nwindows):
        win_y_low = binary_birdeye.shape[0] - (window + 1) * window_h
        win_y_high = binary_birdeye.shape[0] - window * window_h
        win_left_low = left_curr - margin
        win_left_high = left_curr + margin
        win_right_low = right_curr - margin
        win_right_high = right_curr + margin

        # Identiry nonzero pixels within the window
        good_left_lane = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high)
                          & (nonzerox >= win_left_low) & (nonzerox < win_left_high)).nonzero()[0]
        good_right_lane = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high)
                           & (nonzerox >= win_right_low) & (nonzerox < win_right_high)).nonzero()[0]
        
        left_lane.append(good_left_lane)
        right_lane.append(good_right_lane)

        # Recenter window if needed
        if len(good_right_lane) > minpix:
            left_curr = np.int32(np.mean(nonzerox[good_left_lane]))
        if len(good_right_lane) > minpix:
            right_curr = np.int32(np.mean(nonzerox[good_right_lane]))

    # Not needed?
    # try:
    #     left_lane = np.concatenate(left_lane)
    #     right_lane = np.concatenate(right_lane)
    # except ValueError:
    #     pass

    return nonzerox[left_lane], nonzeroy[left_lane], nonzerox[right_lane], nonzeroy[right_lane]

def fit_poly(binary_birdeye, leftx, lefty, rightx, righty):
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)

    # Generate values for plotting
    ploty = np.linspace(0, binary_birdeye.shape[0] - 1, binary_birdeye.shape[0])
    try:
        left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]
    except TypeError:
        print("Failed to fit lane!")
        left_fitx = ploty**2 + ploty
        right_fitx = ploty**2 + ploty

    return left_fit, right_fit, left_fitx, right_fitx, ploty

# def draw_poly_lines(binary_warped, left_fitx, right_fitx, ploty):     
#     # Create an image to draw on and an image to show the selection window
#     out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
#     window_img = np.zeros_like(out_img)
        
#     margin = 100
#     # Generate a polygon to illustrate the search window area
#     # And recast the x and y points into usable format for cv2.fillPoly()
#     left_line_window1 = np.array([np.transpose(np.vstack([left_fitx-margin, ploty]))])
#     left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx+margin, 
#                               ploty])))])
#     left_line_pts = np.hstack((left_line_window1, left_line_window2))
#     right_line_window1 = np.array([np.transpose(np.vstack([right_fitx-margin, ploty]))])
#     right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx+margin, 
#                               ploty])))])
#     right_line_pts = np.hstack((right_line_window1, right_line_window2))

#     # Draw the lane onto the warped blank image
#     cv2.fillPoly(window_img, np.int_([left_line_pts]), (100, 100, 0))
#     cv2.fillPoly(window_img, np.int_([right_line_pts]), (100, 100, 0))
#     result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)
    
#     # Plot the polynomial lines onto the image
#     plt.plot(left_fitx, ploty, color='green')
#     plt.plot(right_fitx, ploty, color='blue')
#     ## End visualization steps ##
#     return result
    
binary_birdeye, M_inv = birdeye_view(binary)
plt.imsave('binary_birdeye.png', binary_birdeye)
# leftx, lefty, rightx, righty = find_lane_pixels_using_histogram(binary_warped)
# left_fit, right_fit, left_fitx, right_fitx, ploty = fit_poly(binary_warped,leftx, lefty, rightx, righty)
# print(left_fit)
# out_img = draw_poly_lines(binary_warped, left_fitx, right_fitx, ploty)
# plt.imshow(out_img)

# plt.imshow(out_img, cmap='gray')
# plt.show()
# todo