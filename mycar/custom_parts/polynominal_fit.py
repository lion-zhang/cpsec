import time
import numpy as np
import cv2
import matplotlib.pyplot as plt

class find_line(object):
    def __init__(self, poll_delay =0.01):
        self.on = True
        self.poll_delay = poll_delay        
        # Choose the number of sliding windows
        self.nwindows = 10
        # Set the width of the windows +/- margin
        self.margin = 10
        # margin = 100
        # Set minimum number of pixels found to recenter window
        self.minpix = 60
        # minpix = 50
        # Create empty lists to receive left and right lane pixel indices
        #self.right_lane_inds = 0
        #self.left_lane_inds = 0
        self.left_fit = 0
        self.right_fit = 0
        self.comp_array = np.ones((160, 120))*200

    
    
    
    def run(self, binary_warped):
        self.binary_warped = binary_warped
        self.poll()
        return self.left_fit, self.right_fit, self.left_lane_inds, self.right_lane_inds
    # Call in the control loop
    # Works when threaded=False
    # Input is parameters, Return your output
    
    def shutdown(self):
        self.on = False
        time.sleep(0.2)
    # Call once before stop the part
    
    def update(self):
        while self.on:
            self.poll()
            time.sleep(self.poll_delay)
    # your thread
    # Works when threaded=True

    def run_threaded(self, binary_warped):
        self.binary_warped = binary_warped
        return self.left_fit, self.right_fit, self.left_lane_inds, self.right_lane_inds
    # Call in the control loop
    # Works when threaded=True
    # Similar as run function
    
    def poll(self):
        self.left_lane_inds = np.array([], dtype='i')
        self.right_lane_inds = np.array([], dtype='i')
        self.binary_warped_filter = np.zeros((160, 120))
        self.binary_warped_filter[self.binary_warped > self.comp_array] = 1
        #np.savetxt('/home/pi/testbed/mycar/custom_parts/img/array4.txt', self.binary_warped_filter, fmt="%s")
        
        #self.binary_warped = np.array(self.binary_warped)
        #self.histogram = np.sum(self.binary_warped[self.binary_warped.shape[0]//2:, :], axis=0)
        self.histogram = np.sum(self.binary_warped_filter[:, self.binary_warped_filter.shape[1]//2:], axis=1)


        '''
        plt.figure()
        plt.plot(self.histogram)
        plt.savefig('/home/pi/testbed/mycar/custom_parts/img/histogram2.png')
        '''
        


        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        midpoint = np.int(self.histogram.shape[0]/2)
        leftx_base = np.argmax(self.histogram[:(midpoint - 20)])
        rightx_base = np.argmax(self.histogram[(midpoint + 20):]) + (midpoint + 20)
        # Set height of windows
        window_height = np.int(self.binary_warped_filter.shape[1]/self.nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = self.binary_warped_filter.nonzero()
        nonzeroy = np.array(nonzero[1])
        nonzerox = np.array(nonzero[0])
        #print('nonzerox: ', nonzerox.size)

        # Current positions to be updated for each window
        leftx_current = leftx_base
        rightx_current = rightx_base

        for window in range(self.nwindows):

            # Identify window boundaries in x and y (and right and left)
            win_y_low = self.binary_warped_filter.shape[1] - (window + 1)*window_height
            win_y_high = self.binary_warped_filter.shape[1] - window*window_height
            win_xleft_low = leftx_current - self.margin
            win_xleft_high = leftx_current + self.margin
            win_xright_low = rightx_current - self.margin
            win_xright_high = rightx_current + self.margin
            # Identify the nonzero pixels in x and y within the window
            #print(win_y_low)
            good_left_inds = ((nonzeroy >= win_y_low)&(nonzeroy < win_y_high)&(nonzerox >= win_xleft_low)&(nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low)&(nonzeroy < win_y_high)&(nonzerox >= win_xright_low)&(nonzerox < win_xright_high)).nonzero()[0]
            # Append these indices to the lists
            #self.left_lane_inds.append(good_left_inds)
            #self.right_lane_inds.append(good_right_inds)
            self.left_lane_inds = np.append(self.left_lane_inds, good_left_inds)
            self.right_lane_inds = np.append(self.right_lane_inds, good_right_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > self.minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > self.minpix:
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
        # Concatenate the arrays of indices    
        #self.left_lane_inds = np.concatenate(self.left_lane_inds)
        #self.right_lane_inds = np.concatenate(self.right_lane_inds)
        #print('left_lane_inds: ', self.left_lane_inds.size)
        # Extract left and right line pixel positions
        leftx = nonzerox[self.left_lane_inds]
        lefty = nonzeroy[self.left_lane_inds]
        rightx = nonzerox[self.right_lane_inds]
        righty = nonzeroy[self.right_lane_inds]
            
        # Fit a second order polynomial to each
        self.left_fit = np.polyfit(lefty, leftx, 2)
        self.right_fit = np.polyfit(righty, rightx, 2)
        '''
        print('left: ', self.left_fit)
        print('right: ', self.right_fit)
        '''

        




