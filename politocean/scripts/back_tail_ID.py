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

class ShapeDetector:  #finds the shape of the image
	def __init__(self):  #initialization of the function
		pass
 
	def detect(self, c): #body of the function
		# initialize the shape name and approximate the contour
		shape = "unidentified"
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.05 * peri, True)
        
        # count the number of vertices
		if len(approx) == 2:
			shape = "two"
		elif len(approx) == 3:
			shape = "three"
		elif len(approx) == 4:
			shape = "four"
		elif len(approx) == 5:
			shape = "five"
		elif len(approx) == 6:
			shape = "six"
		elif len(approx) == 7:
			shape = "seven"
		elif len(approx) == 8:
			shape = "eight"
		elif len(approx) == 9:
			shape = "nine"
		elif len(approx) == 11:
			shape = "eleven"
		elif len(approx) == 12:
			shape = "twelve"
		elif len(approx) == 13:
			shape = "thirteen"
		else:
			shape = "unidentified"
 
		return shape

class image_converter:  #performs the pre-traitment and the recognition

	def __init__(self): #initialization of function 
    	self.image_pub = rospy.Publisher("vision_string",String) #publication of image

    	self.bridge = CvBridge()
    	self.image_sub = rospy.Subscriber("/camera/image_rect_color",Image,self.callback) #subscription of the image
 
	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
			
		structure = "unidentified"
	
    	resized = imutils.resize(cv_image, width=400) #resize image 
		resized = cv_image
		ratio = cv_image.shape[0] / float(resized.shape[0]) #find resizing ratio

		# blur the image to remove noise and convert it to HSV for an easier color manipulation
		blurred = cv2.GaussianBlur(resized, (3, 3), 2) 
		hsv_img = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

		#define the white boundaries
		lower_white = np.array([0, 0, 200], dtype = "uint8")
		upper_white = np.array([180, 80, 255], dtype = "uint8")
		# find the colors within the white boundaries and apply the mask
		white_mask = cv2.inRange(hsv_img, lower_white, upper_white)

		#call back a morphological element (ellipse 3*3)
		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
		white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel) #opening
		white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel) #closure

		#operation to highlight the white objects
		white_masked = cv2.bitwise_or(blurred, blurred, mask=white_mask)
		masked_hsv = white_masked
		
		#define color boundaries to extract white objects
		lower_hsv = np.array([180, 180, 180], dtype = "uint8")
		upper_hsv = np.array([255, 255, 255], dtype = "uint8")
		hsv_mask = cv2.inRange(masked_hsv, lower_hsv, upper_hsv)

		#Canny filtering (contours)
		white_edges = cv2.Canny(hsv_mask, 100, 200) 
		#find convex contours in the masks
		white_cnts = cv2.findContours(white_edges,2,1)
		white_cnts = white_cnts[0] if imutils.is_cv2() else white_cnts[1]
		#find convex envelopes of the white contours
		white_hulls = [cv2.convexHull(p) for p in white_cnts]

		#labels the connected elements of the image
		labels = cv2.connectedComponents(hsv_mask)[1]
		
		#initialize the shape detector
		sd = ShapeDetector()

		#looping in the white contours w
		for w in white_hulls:
	  
			white_shape = sd.detect(w)  
			
			M = cv2.moments(w)
			cX = int((M["m10"] / M["m00"]) * ratio) #find x momentum
			cY = int((M["m01"] / M["m00"]) * ratio) #find y momentum

			perimeter = cv2.arcLength(w,True)

			#looping in the labels across the image
			for l in labels:

				lCopy = np.uint8(l)
				#Canny filtering (contours)
				color_edges = cv2.Canny(lCopy, 35, 125)
				#find convex contours in the current label
				color_cnts = cv2.findContours(color_edges,2,1)
				color_cnts = color_cnts[0] if imutils.is_cv2() else color_cnts[1]
				#find convex envelopes of the current label
				color_hulls = [cv2.convexHull(p) for p in color_cnts]

				for c in color_hulls: #looping in the contours c
					
					M = cv2.moments(c)
					cXt = int((M["m10"] / M["m00"]) * ratio) #find x momentum
					cYt = int((M["m01"] / M["m00"]) * ratio) #find y momentum
					
					#find distance between white object and current label
			        distance = np.sqrt((cXt-cX)**2 + (cYt-cY)**2)/perimeter;
					
                    col_shape = sd.detect(c)

					#controls relative distance between white tail and character and based on the number of vertices of the hull recognizes the tail
				    if (distance < 0.2 and (col_shape == "four" or col_shape == "twelve" or col_shape = "two") and shape == "three"):
						structure = "red triangle"
                    elif (distance < 0.2 and (col_shape == "six" or col_shape == "three" or col_shape = "eight") and shape == "three"):
						structure = "yellow triangle" 
                    elif (distance < 0.2 and (col_shape == "nine" or col_shape == "seven" or col_shape = "five") and shape == "three"):
						structure = "blue triangle" 
                    elif (distance < 0.2 and (col_shape == "four" or col_shape == "eleven" or col_shape = "five") and shape == "four"):
						structure = "red rectangle" 
                    elif (distance < 0.2 and (col_shape == "three" or col_shape == "twelve" or col_shape = "seven") and shape == "four"):
						structure = "yellow rectangle" 
                    elif (distance < 0.2 and (col_shape == "eight" or col_shape == "six" or col_shape = "thirteen") and shape == "four"):
						structure = "blue rectangle"  
                        

                    try:
                    	self.image_pub.publish(structure)
                    except CvBridgeError as e:
                    	print(e)

def main(args):
	ic = image_converter()
	rospy.init_node('back_tail_id', anonymous=False)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
