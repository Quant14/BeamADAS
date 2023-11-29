# This script will process the sensor data coming from the host and send back a response
# including the adjustments that need to be made to the user input

import matplotlib.pyplot as plt
import numpy as np
import cv2

class LaneCurve:
    def __init__(self):
        self.prev_left_fit = np.array([])
        self.prev_right_fit = np.array([])

        self.left_fit_hist = np.array([])
        self.right_fit_hist = np.array([])

    def birdeye_view(self, img):
        img_size = (img.shape[1], img.shape[0])
        offset = 450

        # Source points taken from images with straight lane lines
        src = np.array([
            (390, 547), # bottom-left corner
            (627, 371), # top-left corner
            (654, 371), # top-right corner
            (903, 547) # bottom-right corner
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
    # birdeye_img, M_inv = birdeye_view(img)
    # plt.imsave('birdeye_img.png', birdeye_img)

    def binary_threshold(self, img):
        # Apply sobel in x direction (detect vertical lines)
        blur = cv2.GaussianBlur(img, (5, 5), 0)
        # sobelx = cv2.Sobel(blur, cv2.CV_64F, 1, 0) # type: ignore
        # abs_sobelx = np.absolute(sobelx)

        # scaled_sobel = np.array(255 * abs_sobelx / np.max(abs_sobelx), dtype=np.uint8)
        # sx_binary = np.zeros_like(scaled_sobel)
        
        # sx_binary[(scaled_sobel >= 20) & (scaled_sobel <= 60)] = 1

        # # Test different edge finding technique
        # new_binary = cv2.Canny(blur, 120, 200)

        # plt.imshow(sx_binary)
        # plt.imshow(new_binary)
        # plt.show()
        # exit()

        # Detect white pixels
        white_binary = np.zeros_like(blur)
        white_binary[(blur > 190) & (blur <= 240)] = 1

        return white_binary
        # return cv2.bitwise_or(new_binary, white_binary)

    # Test 2 - successful
    # binary = binary_threshold(img)
    # out_img = np.dstack((binary, binary, binary))*255 # type: ignore
    # plt.imsave('binary_img_no_yellow.png', out_img)

    # Test 3 - successful
    # binary_birdeye, M_inv = birdeye_view(binary)
    # plt.imsave('binary_birdeye.png', binary_birdeye)
    # binary_birdeye_2, M_inv_2 = birdeye_view(binary_threshold(img2))

    def detect_lane_lines(self, binary_birdeye):
        # Make histogram of bottom half of img
        histogram = np.sum(binary_birdeye[binary_birdeye.shape[0] // 2:,:], axis=0)

        # Find lanes starting points
        midpoint = np.int32(histogram.shape[0] // 2)
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

            # Add lane data and recenter windows if needed
            if good_left_lane.size != 0:
                left_lane.append(good_left_lane)
                if len(good_right_lane) > minpix:
                    left_curr = np.int32(np.mean(nonzerox[good_left_lane]))
            if good_right_lane.size != 0:
                right_lane.append(good_right_lane)
                if len(good_right_lane) > minpix:
                    right_curr = np.int32(np.mean(nonzerox[good_right_lane]))

        try:
            left_lane = np.concatenate(left_lane)
            right_lane = np.concatenate(right_lane)
        except ValueError:
            plt.imshow(binary_birdeye)
            plt.show()
            exit("No lines detected!")

        return nonzerox[left_lane], nonzeroy[left_lane], nonzerox[right_lane], nonzeroy[right_lane] # type: ignore

    def fit_poly(self, binary_birdeye, leftx, lefty, rightx, righty):
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

        weight = 0
        if left_fitx[0] > left_fitx[719] and right_fitx[0] > right_fitx[719]:
            weight = 1

        return left_fit, right_fit, left_fitx, right_fitx, ploty, weight

    def draw_poly_lines(self, binary_birdeye, left_fitx, right_fitx, ploty):     
        # Create an image to draw on and an image to show the selection window
        out_img = np.dstack((binary_birdeye, binary_birdeye, binary_birdeye))*255
        window_img = np.zeros_like(out_img)
            
        margin = 100
        # Generate a polygon to illustrate the search window area
        # And recast the x and y points into usable format for cv2.fillPoly()
        left_line_window1 = np.array([np.transpose(np.vstack([left_fitx-margin, ploty]))])
        left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx+margin, 
                                ploty])))])
        left_line_pts = np.hstack((left_line_window1, left_line_window2))
        right_line_window1 = np.array([np.transpose(np.vstack([right_fitx-margin, ploty]))])
        right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx+margin, 
                                ploty])))])
        right_line_pts = np.hstack((right_line_window1, right_line_window2))

        # Draw the lane onto the warped blank image
        cv2.fillPoly(window_img, np.int_([left_line_pts]), (100, 100, 0)) # type: ignore
        cv2.fillPoly(window_img, np.int_([right_line_pts]), (100, 100, 0)) # type: ignore
        result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)
        
        # Plot the polynomial lines onto the image
        plt.plot(left_fitx, ploty, color='green')
        plt.plot(right_fitx, ploty, color='blue')
        ## End visualization steps ##
        return result
        
    # Test 4 - successful
    # leftx, lefty, rightx, righty = detect_lane_lines(binary_birdeye)
    # left_fit, right_fit, left_fitx, right_fitx, ploty = fit_poly(binary_birdeye, leftx, lefty, rightx, righty)
    # out_img = draw_poly_lines(binary_birdeye, left_fitx, right_fitx, ploty)
    # plt.imsave("birdeye_lines_detected.png", out_img)
    # plt.imshow(out_img)
    # plt.show()

    # prev_left_fit, prev_right_fit = left_fit, right_fit

    def find_lane_pixels_using_prev_poly(self, binary_birdeye):
        # Possible margin to prev frame
        margin = 150

        # Retrieve activated pixels from prev frame
        nonzero = binary_birdeye.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        # Set search area based on prev frame
        left_lane = ((nonzerox > (self.prev_left_fit[0] * (nonzeroy**2) + self.prev_left_fit[1] * nonzeroy + 
                        self.prev_left_fit[2] - margin)) & (nonzerox < (self.prev_left_fit[0] * (nonzeroy**2) + 
                        self.prev_left_fit[1] * nonzeroy + self.prev_left_fit[2] + margin))).nonzero()[0]
        right_lane = ((nonzerox > (self.prev_right_fit[0] * (nonzeroy**2) + self.prev_right_fit[1] * nonzeroy + 
                        self.prev_right_fit[2] - margin)) & (nonzerox < (self.prev_right_fit[0] * (nonzeroy**2) + 
                        self.prev_right_fit[1] * nonzeroy + self.prev_right_fit[2] + margin))).nonzero()[0]
        
        return nonzerox[left_lane], nonzeroy[left_lane], nonzerox[right_lane], nonzeroy[right_lane]

    # Test 5 - successful
    # leftx, lefty, rightx, righty = find_lane_pixels_using_prev_poly(binary_birdeye_2)
    # left_fit, right_fit, left_fitx, right_fitx, ploty = fit_poly(binary_birdeye_2, leftx, lefty, rightx, righty)
    # out_img = draw_poly_lines(binary_birdeye_2, left_fitx, right_fitx, ploty)
    # plt.imsave("birdeye_lines_detected_2.png", out_img)

    def measure_curvature(self, left_fitx, right_fitx, ploty):
        # Define meters per pixel
        ym_ppx = 30 / 720
        xm_ppx = 3.7 / 700

        left_fit_cr = np.polyfit(ploty * ym_ppx, left_fitx * xm_ppx, 2)
        right_fit_cr = np.polyfit(ploty * ym_ppx, right_fitx * xm_ppx, 2)

        y_eval = np.max(ploty)

        # Calculate curve radius
        left_rad = ((1 + (2 * left_fit_cr[0] * y_eval * ym_ppx + left_fit_cr[1])**2)**1.5) / np.absolute(2 * left_fit_cr[0])
        right_rad = ((1 + (2 * right_fit_cr[0] * y_eval * ym_ppx + right_fit_cr[1])**2)**1.5) / np.absolute(2 * right_fit_cr[0])

        return left_rad, right_rad

    # Test 6 - successful
    # left_curve, right_curve = measure_curvature(left_fitx, right_fitx, ploty)
    # print('Left curve: ' + str(left_curve) + '; Right curve: ' + str(right_curve))

    def measure_pos(self, binary_birdeye, left_fit, right_fit):
        # Define meters per pixel
        xm_per_pix = 3.7 / 700

        y_max = binary_birdeye.shape[0]

        # Calculate left and right lane pos
        left_x_pos = left_fit[0] * y_max**2 + left_fit[1] * y_max + left_fit[2]
        right_x_pos = right_fit[0] * y_max**2 + right_fit[1] * y_max + right_fit[2]

        # Calculate lane center
        center_lanes_x_pos = (left_x_pos + right_x_pos) // 2

        # Calculate the car's offset relative to the lane center
        return ((binary_birdeye.shape[1] // 2) - center_lanes_x_pos) * xm_per_pix

    # Test 7 - successful
    # offset = measure_pos(binary_birdeye_2, left_fit, right_fit)
    # print('Offset: ' + str(offset))

    def lane_pipeline(self, img):
        binary = self.binary_threshold(img)
        binary_birdeye, _ = self.birdeye_view(binary)
        weight = 0

        if (len(self.left_fit_hist) == 0):
            leftx, lefty, rightx, righty = self.detect_lane_lines(binary_birdeye)
            left_fit, right_fit, left_fitx, right_fitx, ploty, weight = self.fit_poly(binary_birdeye, leftx, lefty, rightx, righty)

            self.left_fit_hist = np.array(left_fit)
            self.right_fit_hist = np.array(right_fit)
            new_left_fit = np.array(left_fit)
            new_right_fit = np.array(right_fit)
            self.left_fit_hist = np.vstack([self.left_fit_hist, new_left_fit])
            self.right_fit_hist = np.vstack([self.right_fit_hist, new_right_fit])
        else:
            self.prev_left_fit = [np.mean(self.left_fit_hist[:,0]), np.mean(self.left_fit_hist[:,1]), np.mean(self.left_fit_hist[:,2])]
            self.prev_right_fit = [np.mean(self.right_fit_hist[:,0]), np.mean(self.right_fit_hist[:,1]), np.mean(self.right_fit_hist[:,2])]
            leftx, lefty, rightx, righty = self.find_lane_pixels_using_prev_poly(binary_birdeye)

            if (len(lefty) == 0 or len(righty) == 0):
                leftx, lefty, rightx, righty = self.detect_lane_lines(binary_birdeye)
            left_fit, right_fit, left_fitx, right_fitx, ploty, weight = self.fit_poly(binary_birdeye,leftx, lefty, rightx, righty)             

            new_left_fit = np.array(left_fit)
            new_right_fit = np.array(right_fit)
            self.left_fit_hist = np.vstack([self.left_fit_hist, new_left_fit])
            self.right_fit_hist = np.vstack([self.right_fit_hist, new_right_fit])
            
            if (len(self.left_fit_hist) > 5):
                self.left_fit_hist = np.delete(self.left_fit_hist, 0,0)
                self.right_fit_hist = np.delete(self.right_fit_hist, 0,0)
                   
        # DEBUG - remove for max performance
        # a = self.draw_poly_lines(binary_birdeye, left_fitx, right_fitx, ploty)            
        # plt.imsave('./BeamADAS_Processor/cam_res/proc_img' + str(i) + '.png', a)
        # ----------------------------------
        
        left_rad, right_rad =  self.measure_curvature(left_fitx, right_fitx, ploty)
        pos = self.measure_pos(binary_birdeye, left_fit, right_fit)

        if weight:
            return right_rad, pos
        else:
            return left_rad, pos