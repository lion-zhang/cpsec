import time
import numpy as np
import matplotlib.image as mpimg
import numpy as np
from cv2 import cv2
import matplotlib.pyplot as plt


# How to call this part?
# V = Vehicle()
# t = template(p1)
# V.add(t, input=['in1', 'in2'], output=['tdata1','tdata2'], threaded=True)
# V.start()



class draw_area(object):
    def __init__(self, poll_delay=0.01):
        self.on = True
        self.poll_delay = poll_delay
        self.src = np.float32([[25, 90],[138, 90],[40, 70],[100, 70]])
        self.dst = np.float32([[37, 90],[122, 90],[40, 0],[110, 0]])   
    
    # Initiate your part here
    
    def run(self, undist, binary_warped, left_fit, right_fit):
        self.binary_warped = binary_warped
        self.right_fit = right_fit
        self.left_fit = left_fit
        self.undist = undist
        warp = self.poll()
        return warp.tolist()
    
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

    def run_threaded(self, in1, in2):
        return self.result
        # Call in the control loop
        # Works when threaded=True
        # Similar as run function
    
    def poll(self):
        self.binary_warped = np.array(self.binary_warped)
        self.undist = np.array(self.undist)
        ploty = np.linspace(0, self.binary_warped.shape[1]-1, self.binary_warped.shape[1])
        left_fitx = self.left_fit[0]*ploty**2 + self.left_fit[1]*ploty + self.left_fit[2]
        right_fitx = self.right_fit[0]*ploty**2 + self.right_fit[1]*ploty + self.right_fit[2]
        # Create an image to draw the lines on
        #warp_zero = np.zeros_like(self.binary_warped).astype(np.uint8)
        warp_zero = np.zeros((120,160)).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        
        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))
        # Draw the lane onto the warped blank image
        cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

        # Warp the blank back to original image space using inverse perspective matrix (self.Minv)
        color_warp = cv2.rotate(color_warp, cv2.ROTATE_90_CLOCKWISE)
        color_warp = cv2.flip(color_warp, 1)
        m_inv = cv2.getPerspectiveTransform(self.dst, self.src)
        
        #self.newwarp = cv2.warpPerspective(color_warp, m_inv, (160, 120), flags=cv2.INTER_LINEAR)
        
        #self.newwarp = cv2.rotate(self.newwarp, cv2.ROTATE_90_CLOCKWISE)

        #print(self.newwarp.shape)
        #self.newwarp = np.asarray(self.newwarp, np.float64)
        
        self.undist = cv2.merge([self.undist, self.undist, self.undist])
        self.undist = np.asarray(self.undist, np.float64)
        color_warp = np.asarray(color_warp, np.float64)
        # Combine the result with the original image
        #self.result = cv2.addWeighted(self.undist, 1, self.newwarp, 0.3, 0)
        self.result = cv2.addWeighted(self.undist, 1, color_warp, 0.3, 0)

        return self.result
# your actual function of the thread
