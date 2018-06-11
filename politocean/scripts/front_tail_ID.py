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
		if len(approx) == 3:
			shape = "triangle"
		elif len(approx) == 4:
			shape = "rectangle"
		else:
			shape = "unidentified"
 
		return shape

class image_converter:  #performs the pre-traitment and the recognition

	def __init__(self): #initialization of function 
		self.image_pub = rospy.Publisher("vision_string",String, queue_size=0) #publication of image

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/image_rect_color",Image,self.callback) #subscription of the image
 
	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		resized = imutils.resize(cv_image, width=400) #resize image 
		# resized = cv_image
		ratio = cv_image.shape[0] / float(resized.shape[0]) #find resizing ratio

		# blur the image to remove noise and convert it to HSV for an easier color manipulation
		blurred = cv2.GaussianBlur(resized, (7, 7), 3) 
		hsv_img = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

		# define the list of color boundaries
		# H: 0 -> 180 _ _ _ S: 0 -> 255 _ _ _ V: 0 -> 255
		color_boundaries = [
				([80, 80, 40], [140, 255, 255], 'blue'),
				([25, 80, 120], [45, 255, 255], 'yellow'),
		#        ([170, 80,140], [180, 255, 255], 'red'), # red-violet
				([0, 80,120], [10, 255, 255], 'red') # red-orange
		]

		# loop over the color boundaries
		for (lower, upper, color) in color_boundaries:

			# create NumPy arrays from the boundaries
			lower = np.array(lower, dtype = "uint8")
			upper = np.array(upper, dtype = "uint8")

			# find the colors within the boundaries and apply the mask
			color_mask = cv2.inRange(hsv_img, lower, upper)  
			kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)) #call back a morphological element (ellipse 7*7)
			color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel) #opening
			color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel) #closure

			# find edges within the masked image to distinguish the real matching from other stuff
			color_masked = cv2.bitwise_or(blurred, blurred, mask=color_mask)
			color_masked = cv2.cvtColor(color_masked, cv2.COLOR_BGR2GRAY)
			color_edges = cv2.Canny(color_masked, 35, 125) #Canny filtering (contours)
			kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)) #call back a morphological element (ellipse 3*3)
			color_edges = cv2.dilate(color_edges,kernel,iterations = 1) #dilatation

			lower_white = np.array([0, 0, 200], dtype = "uint8")
			upper_white = np.array([180, 80, 255], dtype = "uint8")
			white_mask = cv2.inRange(hsv_img, lower_white, upper_white)
			white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel) #opening
			white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel) #closure

			# find edges within the masked image to distinguish the real matching from other stuff
			white_masked = cv2.bitwise_or(blurred, blurred, mask=white_mask)
			white_masked = cv2.cvtColor(white_masked, cv2.COLOR_BGR2GRAY)
			white_edges = cv2.Canny(white_masked, 35, 125) #Canny filtering (contours)
			kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)) #call back a morphological element (ellipse 3*3)
			white_edges = cv2.dilate(white_edges,kernel,iterations = 1) #dilatation

			# create the total mask in order to visualize it (DEBUG)
			edges = color_edges+white_edges  #sum the edges found from the two processes
			 
			#find convex contours in the masks and initialize the shape detector
			cnts = cv2.findContours(color_edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #find contours of coloured edges
			cnts = cnts[0] if imutils.is_cv2() else cnts[1]
			color_hulls = [cv2.convexHull(p.reshape(-1, 1, 2)) for p in cnts]

			cnts = cv2.findContours(white_edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #find contours of coloured edges
			cnts = cnts[0] if imutils.is_cv2() else cnts[1]
			hulls = [cv2.convexHull(p.reshape(-1, 1, 2)) for p in cnts]

			sd = ShapeDetector()

			# loop over the contours
			if len(color_hulls) > 0: #if the hulls are more than 1
		    
				for c in hulls: #looping in the contours c
					# compute the center of the contour, detect the name of the shape
					M = cv2.moments(c)
					cX = int((M["m10"] / M["m00"]) * ratio) #find x momentum
					cY = int((M["m01"] / M["m00"]) * ratio) #find y momentum
					shape = sd.detect(c)

					perimeter = cv2.arcLength(c,True)

					for c_col in color_hulls:

						M = cv2.moments(c_col)
						cXt = int((M["m10"] / M["m00"]) * ratio) #find x momentum
						cYt = int((M["m01"] / M["m00"]) * ratio) #find y momentum
						shape_col = sd.detect(c_col)

						# compute and check the distance between the centroids
						distance = np.sqrt((cXt-cX)**2 + (cYt-cY)**2)/perimeter;

						#check if recognition is successful
						if (distance < 0.2 and (shape_col == "triangle" or shape_col == "rectangle") and shape == "rectangle"):     

							c_col = c_col.astype("float")
							c_col *= ratio
							c_col = c_col.astype("int")
							cv2.drawContours(cv_image, [c_col], -1, (0, 255, 0), 2)
							cv2.putText(cv_image, color + ' ' + shape_col, (cXt, cYt), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                    		try:
                    			self.image_pub.publish(color + ' ' + shape_col)
                    		except CvBridgeError as e:
                    			print(e)
           

def main(args):
	ic = image_converter()
	rospy.init_node('front_tail_id', anonymous=False)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
