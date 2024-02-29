# This script will process the sensor data coming from the host and send back a response
# including the adjustments that need to be made to the user input

import numpy as np
import cv2

class LaneCurve:
    def __init__(self):
        self.prev_left_fit = np.array([])
        self.prev_right_fit = np.array([])

        self.left_fit_hist = np.array([])
        self.right_fit_hist = np.array([])

        self.last_time = 0.0

    def birdeye_view(self, img):
        img_size = (img.shape[1], img.shape[0])
        offset = 400

        src = np.array([
            (390, 547), # bottom-left corner
            (617, 383), # top-left corner
            (677, 383), # top-right corner
            (903, 547) # bottom-right corner
        ], dtype='f')
        dst = np.array([
            [offset, img_size[1]],             # bottom-left corner
            [offset, 0],                       # top-left corner
            [img_size[0]-offset, 0],           # top-right corner
            [img_size[0]-offset, img_size[1]]  # bottom-right corner
        ], dtype='f')

        M = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(img, M, img_size)

        return warped

    def binary_threshold(self, img):
        blur = cv2.GaussianBlur(img, (5, 5), 0)
        avg = cv2.mean(img[650:,:])[0]
        thresh = avg + (240 - avg) * 0.7

        white_binary = np.zeros_like(blur)
        white_binary[(blur > thresh) & (blur <= 240)] = 1

        return white_binary

    def detect_lane_lines(self, binary_birdeye):
        histogram = np.sum(binary_birdeye[650:,:], axis=0)

        nonzero_ind = np.nonzero(histogram)[0]
        peak_bounds = np.split(nonzero_ind, np.where(np.diff(nonzero_ind) > 1)[0] + 1)

        if len(peak_bounds) < 2:
            return None, None, None, None

        gap_sizes = np.diff([peak[-1] for peak in peak_bounds])
        max_gap_index = np.argmax(gap_sizes)

        if gap_sizes[max_gap_index] < 200:
            return None, None, None, None

        midpoint = (peak_bounds[max_gap_index][-1] + peak_bounds[max_gap_index + 1][0]) // 2 # type: ignore
        left_base = np.argmax(histogram[:midpoint])
        right_base = np.argmax(histogram[midpoint:]) + midpoint # type: ignore

        nwindows = 15
        margin = 100
        minpix = 20

        window_h = np.int32(binary_birdeye.shape[0]//nwindows)

        nonzero = binary_birdeye.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        left_curr = left_base
        right_curr = right_base

        left_lane = []
        right_lane = []

        complete_left = False
        complete_right = False

        for window in range(nwindows):
            if complete_left and complete_right: break

            win_y_low = binary_birdeye.shape[0] - (window + 1) * window_h # type: ignore
            win_y_high = binary_birdeye.shape[0] - window * window_h # type: ignore

            if not complete_left:
                win_left_low = left_curr - margin # type: ignore
                win_left_high = left_curr + margin # type: ignore
                good_left_lane = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high)
                                & (nonzerox >= win_left_low) & (nonzerox < win_left_high)).nonzero()[0]
                if good_left_lane.size != 0:
                    left_lane.append(good_left_lane)
                    if len(good_left_lane) > minpix:
                        left_curr = np.int32(np.mean(nonzerox[good_left_lane]))
                        if left_curr - margin <= 0 or left_curr + margin >= 1280: # type: ignore
                            complete_left = True

            if not complete_right:
                win_right_low = right_curr - margin # type: ignore
                win_right_high = right_curr + margin # type: ignore
                good_right_lane = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high)
                                & (nonzerox >= win_right_low) & (nonzerox < win_right_high)).nonzero()[0]
                if good_right_lane.size != 0:
                    right_lane.append(good_right_lane)
                    if len(good_right_lane) > minpix:
                        right_curr = np.int32(np.mean(nonzerox[good_right_lane]))
                        if right_curr - margin <= 0 or right_curr + margin >= 1280: # type: ignore
                            complete_right = True

        try:
            left_lane = np.concatenate(left_lane)
            right_lane = np.concatenate(right_lane)
        except ValueError:
            print('No lines detected!')
            return None, None, None, None

        return nonzerox[left_lane], nonzeroy[left_lane], nonzerox[right_lane], nonzeroy[right_lane] # type: ignore

    def fit_poly(self, binary_birdeye, leftx, lefty, rightx, righty):
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        ploty = np.linspace(0, binary_birdeye.shape[0] - 1, binary_birdeye.shape[0])
        try:
            left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
            right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]
        except TypeError:
            print("Failed to fit lane!")
            return None, None, None, None, None

        if np.isclose(left_fitx, right_fitx, atol=250).any():
            return None, None, None, None, None

        return left_fit, right_fit, left_fitx, right_fitx, ploty

    def find_lane_pixels_using_prev_poly(self, binary_birdeye):
        margin = 150

        nonzero = binary_birdeye.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        left_lane = ((nonzerox > (self.prev_left_fit[0] * (nonzeroy**2) + self.prev_left_fit[1] * nonzeroy +
                        self.prev_left_fit[2] - margin)) & (nonzerox < (self.prev_left_fit[0] * (nonzeroy**2) +
                        self.prev_left_fit[1] * nonzeroy + self.prev_left_fit[2] + margin))).nonzero()[0]
        right_lane = ((nonzerox > (self.prev_right_fit[0] * (nonzeroy**2) + self.prev_right_fit[1] * nonzeroy +
                        self.prev_right_fit[2] - margin)) & (nonzerox < (self.prev_right_fit[0] * (nonzeroy**2) +
                        self.prev_right_fit[1] * nonzeroy + self.prev_right_fit[2] + margin))).nonzero()[0]

        return nonzerox[left_lane], nonzeroy[left_lane], nonzerox[right_lane], nonzeroy[right_lane]

    def measure_curvature(self, left_fitx, right_fitx, ploty):
        ym_ppx = 30 / 720
        xm_ppx = 3.7 / 700

        left_fit_cr = np.polyfit(ploty * ym_ppx, left_fitx * xm_ppx, 2)
        right_fit_cr = np.polyfit(ploty * ym_ppx, right_fitx * xm_ppx, 2)

        y_eval = np.max(ploty)

        left_rad = ((1 + (2 * left_fit_cr[0] * y_eval * ym_ppx + left_fit_cr[1])**2)**1.5) / np.absolute(2 * left_fit_cr[0])
        right_rad = ((1 + (2 * right_fit_cr[0] * y_eval * ym_ppx + right_fit_cr[1])**2)**1.5) / np.absolute(2 * right_fit_cr[0])

        return left_rad, right_rad

    def delete_hist(self, complete):
        if complete == True:
            self.left_fit_hist = np.array([])
            self.right_fit_hist = np.array([])
        else:
            self.left_fit_hist = np.delete(self.left_fit_hist, 0,0)
            self.right_fit_hist = np.delete(self.right_fit_hist, 0,0)

    def lane_pipeline(self, img, timestamp):
        if timestamp - self.last_time > 5:
            self.delete_hist(True)

        binary = self.binary_threshold(img)
        binary_birdeye = self.birdeye_view(binary)

        if len(self.left_fit_hist) == 0:
            leftx, lefty, rightx, righty = self.detect_lane_lines(binary_birdeye)
            if leftx is None:
                return None

            left_fit, right_fit, left_fitx, right_fitx, ploty = self.fit_poly(binary_birdeye, leftx, lefty, rightx, righty)
            if left_fit is None:
                return None

            self.left_fit_hist = np.array(left_fit)
            self.right_fit_hist = np.array(right_fit)
        else:
            self.prev_left_fit = [np.mean(self.left_fit_hist[:,0]), np.mean(self.left_fit_hist[:,1]), np.mean(self.left_fit_hist[:,2])]
            self.prev_right_fit = [np.mean(self.right_fit_hist[:,0]), np.mean(self.right_fit_hist[:,1]), np.mean(self.right_fit_hist[:,2])]
            leftx, lefty, rightx, righty = self.find_lane_pixels_using_prev_poly(binary_birdeye)

            if len(lefty) == 0 or len(righty) == 0:
                self.delete_hist(len(self.left_fit_hist) == 2)
                return None
            left_fit, right_fit, left_fitx, right_fitx, ploty= self.fit_poly(binary_birdeye,leftx, lefty, rightx, righty)

            if left_fit is None:
                self.delete_hist(len(self.left_fit_hist) == 2)
                return None
            if len(self.left_fit_hist) > 9:
                self.delete_hist(False)

        new_left_fit = np.array(left_fit)
        new_right_fit = np.array(right_fit)
        self.left_fit_hist = np.vstack([self.left_fit_hist, new_left_fit])
        self.right_fit_hist = np.vstack([self.right_fit_hist, new_right_fit])

        self.last_time = timestamp

        left_rad, right_rad =  self.measure_curvature(left_fitx, right_fitx, ploty)

        return np.mean([left_rad, right_rad])