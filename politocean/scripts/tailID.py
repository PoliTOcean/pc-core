#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import imutils
import numpy as np
from errmess_publisher import *

class ShapeDetector:
	def __init__(self):
		pass
 
	def detect(self, c):
		# initialize the shape name and approximate the contour
		shape = "unidentified"
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.05 * peri, True)
        
        # count the number of vertices
		if len(approx) == 3:
			shape = "triangle"
		elif len(approx) == 4:
			shape = "rectangle"
		else:
			shape = "unidentified"
 
		return shape

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    rospy.init_node('front_tail_id', anonymous=False)
    self.image_sub = rospy.Subscriber("/raspicam_node/image",Image,self.callback)
    self.image_pub = rospy.Publisher("vision_string",String,queue_size=0)
    errMessInit() #init topics
    
    publishMessages('front_tail_id', 'Identification started')

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    resized = imutils.resize(cv_image, width=400)
    # resized = cv_image
    ratio = cv_image.shape[0] / float(resized.shape[0])

    # blur the image to remove noise and convert it to HSV for an easier color manipulation
    blurred = cv2.GaussianBlur(resized, (7, 7), 3)
    hsv_img = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # define the list of color boundaries
    # H: 0 -> 180 _ _ _ S: 0 -> 255 _ _ _ V: 0 -> 255
    color_boundaries = [
            ([80, 80, 40], [140, 255, 255], 'blue'),
            ([25, 80, 120], [45, 255, 255], 'yellow'),
             ([170, 80,140], [180, 255, 255], 'red'), # red-violet
            ([0, 80,120], [10, 255, 255], 'red') # red-orange
    ]

    # loop over the color boundaries
    for (lower, upper, color) in color_boundaries:

        # create NumPy arrays from the boundaries
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")
     
        # find the colors within the boundaries and apply the mask
        color_mask = cv2.inRange(hsv_img, lower, upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel)

        # find edges within the masked image to distinguish the real matching from other stuff
        color_masked = cv2.bitwise_or(blurred, blurred, mask=color_mask)
        color_masked = cv2.cvtColor(color_masked, cv2.COLOR_BGR2GRAY)
        color_edges = cv2.Canny(color_masked, 35, 125)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        color_edges = cv2.dilate(color_edges,kernel,iterations = 1)
        
        lower_white = np.array([0, 0, 200], dtype = "uint8")
        upper_white = np.array([180, 80, 255], dtype = "uint8")
        white_mask = cv2.inRange(hsv_img, lower_white, upper_white)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        
        # find edges within the masked image to distinguish the real matching from other stuff
        white_masked = cv2.bitwise_or(blurred, blurred, mask=white_mask)
        white_masked = cv2.cvtColor(white_masked, cv2.COLOR_BGR2GRAY)
        white_edges = cv2.Canny(white_masked, 35, 125)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        white_edges = cv2.dilate(white_edges,kernel,iterations = 1)
        
        # create the total mask in order to visualize it (DEBUG)
        edges = color_edges+white_edges
         
        # find convex contours in the masks and initialize the shape detector
        cnts = cv2.findContours(color_edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        color_hulls = [cv2.convexHull(p.reshape(-1, 1, 2)) for p in cnts]
        
        cnts = cv2.findContours(white_edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        hulls = [cv2.convexHull(p.reshape(-1, 1, 2)) for p in cnts]
        
        sd = ShapeDetector()
        
        # loop over the contours
        if len(color_hulls) > 0:
            # show the masked images
#            cv2.imshow("Color matching", color_mask)
#            cv2.imshow("White matching", white_mask)
#            cv2.imshow("Contourns", edges)
        
            for c_col in color_hulls:
                count = 0 # number of 
                # compute the center of the contour, detect the name of the shape
                M = cv2.moments(c_col)
                cX = int((M["m10"] / M["m00"]) * ratio)
                cY = int((M["m01"] / M["m00"]) * ratio)
                shape_col = sd.detect(c_col)
                
                perimeter = cv2.arcLength(c_col,True)
        
                for c in hulls:
                    M = cv2.moments(c)
                    cXt = int((M["m10"] / M["m00"]) * ratio)
                    cYt = int((M["m01"] / M["m00"]) * ratio)
                    shape = sd.detect(c)
                    
                    # compute and check the distance between the centroids
                    distance = np.sqrt((cXt-cX)**2 + (cYt-cY)**2)/perimeter;
                    
                    # print(distance)
                    
                    if (distance < 0.2 and (shape_col == "triangle" or shape_col == "rectangle") and
                            shape == "rectangle"):     # check distance vs dimensions
                        count += 1
                        
                        # multiply the contour (x, y)-coordinates by the resize ratio,
                        # then draw the contours and the name of the shape on the image
#                        c_col = c_col.astype("float")
#                        c_col *= ratio
#                        c_col = c_col.astype("int")
#                        cv2.drawContours(cv_image, [c_col], -1, (0, 255, 0), 2)
#                        cv2.putText(cv_image, color + ' ' + shape_col, (cXt, cYt), cv2.FONT_HERSHEY_SIMPLEX,
#                                    0.5, (255, 255, 255), 2)
#                        cv2.imshow("Image window", cv_image)
#                        cv2.waitKey(3)
                        
                if count == 1 and shape_col == "triangle":
                    try:
                        self.image_pub.publish(color + ' triangle')
                    except CvBridgeError as e:
#                        print(e)
                        pass
                        
                elif count == 2 and shape_col == "rectangle":
                    try:
                        self.image_pub.publish(color + ' rectangle')
                    except CvBridgeError as e:
#                        print(e)
                        pass

def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
