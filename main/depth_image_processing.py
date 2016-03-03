#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy 
import roslib  
import time  
import cv2
import os
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import random
import pylab
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
#from image_converter import ToOpenCV
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
#from tf2_msgs import TFMessage  #this is for tf_static


class image_converter:

	#function for callback image listener
	def callback(self,data):
	  try:
	      global d1
	      global X
	      global Z
	      global grid_ZX
	      global d1_1
	      cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
	      depth_array = np.array(cv_image, dtype=np.float32) 
	      depth_array.shape = (480,640)    #convert 3d depth array to 2d for kinect
      	      depth_image = cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
	      self.image_pub.publish(self.bridge.cv2_to_imgmsg(depth_image, "32FC1"))
	      #self.image_sub.unregister()			      
	      #ret,th1 = cv2.threshold(depth_image,0,255,cv2.THRESH_BINARY)

	      height, width = depth_image.shape
              depth_image_new = depth_image[0:height/2:1,0:width/2:1]
	      depth_image_final = cv2.resize(depth_image_new,(width,height))
              #cv2.imshow("Image window", depth_image_final)
	      #cv2.imwrite("depth_camera_msg.jpg", depth_image_final * 255)
              crop_img = depth_image_final[240:400, 0:640]
              cv2.imshow("Image window", crop_img)
	      cv2.imwrite("cropped_depth_camera_msg.jpg", crop_img * 255)
      	      cv2.waitKey(3)
	      #d1 = depth_image.tolist()	# Now after shaping it is a 2D list tested, dont worry, if you want check by len(numrows(d1)) and len(numcols(d1)) 
	      d1 = crop_img.tolist()
	      #d1 = depth_image_final.tolist()
	      #print d1
	      rows = len(d1)
	      cols = len(d1[0])
	      M = 0.0021  #NUI of the kinect
	      #M = 1.4826
	      Z = []
	      b = []
  	      for col in range(len(d1[0])):
		for row in range(len(d1)):
			if d1[row-1][col] > d1[row][col]:
				a = d1[row][col]
			else:
				a = d1[row-1][col]
		Z.append(a)
		
	      
	      X = []
	      for col in range(len(Z)):
		 X.append(Z[col] * ((col - (len(Z)/2)) * (float(len(Z)/2)/float(len(Z))) * M))	    
		     #################
	      #				    #
	      #	     X = 3Z	    #
	      #				    #
	      #################
	      Z1 = []
	      for col in range(len(Z)):
		if Z[col]!=0:
			Z1.append(Z[col])
              #print Z1
	      X1 = []
	      for col in range(len(Z1)):
		 X1.append(Z1[col] * ((col - (len(Z1)/2)) * (float(len(Z1)/2)/float(len(Z1))) * M))	      			        
	      fig = plt.figure()
	      ax = fig.gca()
	      p = 0.03
	      q = 0.01
	      ax.set_xticks(np.arange(0,1,p))
	      ax.set_yticks(np.arange(-1,1,q))
	      plt.scatter(Z1,X1)
	      plt.grid()
	      pylab.xlim([0,0.2])
	      pylab.ylim([-0.05,0.05])
	      plt.savefig('fig.png')
	      plt.show()
	   except CvBridgeError, e:
	      print e

  #function subscribing to image
	def __init__(self):
	    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)
	    cv2.namedWindow("Image window", 1)
	    self.bridge = CvBridge()
	    self.image_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.callback)
	    time.sleep(0.5)    #ideally it should be 0.33 since 30 frames/sec but due to the imperfection of the python sleep the minimum is 0.5
	    self.image_sub.unregister()	
