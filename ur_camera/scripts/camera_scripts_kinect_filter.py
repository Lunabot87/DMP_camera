#! /usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import sys
import message_filters
import math
import numpy as np
import tf
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion, quaternion_from_euler
 
class depth_estimater:
	WIDTH = 10
	HEIGHT = 10
	EULER = [0,0,0]
 
	def __init__(self):
 
		rospy.init_node('depth_estimater', anonymous=True)
		self.bridge = CvBridge()
		self.image_pub = rospy.Publisher("image_topic",Image, queue_size=5)
		#self.depth_pub = rospy.Publisher("depth_topic",Int16, queue_size=5)
		sub_rgb = message_filters.Subscriber("camera/rgb/image_color",Image)
		sub_depth = message_filters.Subscriber("camera/depth/image",Image)
		self.mf = message_filters.ApproximateTimeSynchronizer([sub_rgb, sub_depth], 100, 10.0)
		self.mf.registerCallback(self.ImageCallback)
		self.color_hsv = {}
		self.color_hsv['target'] = [18, 127, 76, 117, 224, 125]
		self.color_hsv['goal'] = [19, 123, 100, 37, 255, 205]
		self.br = tf.TransformBroadcaster()
 
	def ImageCallback(self, rgb_data , depth_data):
		try:
			color_image = self.bridge.imgmsg_to_cv2(rgb_data, 'passthrough')
			depth_image = self.bridge.imgmsg_to_cv2(depth_data, 'passthrough')
		except CvBridgeError, e:
			rospy.logerr(e)
 
		color_image.flags.writeable = True
		h, w, c = color_image.shape
		frame, centrol, angle, ratio = self.Camera_filter(color_image)

		x1 = centrol[0] - self.WIDTH
		x2 = centrol[0] + self.WIDTH
		y1 = centrol[1] - self.HEIGHT
		y2 = centrol[1] + self.HEIGHT

		sum = 0.0
		pixel_count = 0
		
		for i in range(y1, y2):
			for j in range(x1, x2):
				color_image.itemset((i, j, 0), 0)
				color_image.itemset((i, j, 1), 0)
				#color_image.itemset((100,100,2), 0)
 
				if depth_image.item(i,j) == depth_image.item(i,j):
					pixel_count += 1
					#print(depth_image.item(i,j))
					sum += depth_image.item(i,j)
 
		#ave = sum / ((self.WIDTH * 2) * (self.HEIGHT * 2))
		if pixel_count == 0:
			pixel_count = 1
		ave = sum / pixel_count
		self.EULER[0] = ave
		self.EULER[1] = ((w/2) - centrol[0])*ratio
		self.EULER[2] = ((h/2) - centrol[1])*ratio 
		quat = quaternion_from_euler(angle - 1.5708, 0, 0)
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "8UC3"))
			self.br.sendTransform(self.EULER,quat,rospy.Time.now(),"carrot","world")
		except CvBridgeError, e:
			print e
		
		print("%f [m]" % ave + '%f [rad]' % angle)
		print(self.EULER)

	def Camera_filter(self, frame):

		lower_hsv = np.array(self.color_hsv['target'][:3])
		upper_hsv = np.array(self.color_hsv['target'][3:6])
		frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		
		hsv_mask = cv2.inRange(frame_hsv, lower_hsv, upper_hsv)
		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
		ret, thr = cv2.threshold(hsv_mask, 127, 255, 0)
		_, contours, _ = cv2.findContours(thr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		x_l = 0
		x_r = 0
		y_l = 0
		y_r = 0

		if len(contours) > 0:
			for i in range(len(contours)):
				area = cv2.contourArea(contours[i])
				if area > 1000:
					rect = cv2.minAreaRect(contours[i])
					box = cv2.boxPoints(rect)
					box = np.int0(box)
					box = sorted(box, key=lambda box: box[1])

					x_up = (box[0][0] + box[1][0]) / 2
					y_up = (box[0][1] + box[1][1]) / 2
					x_down = (box[2][0] + box[3][0]) / 2
					y_down = (box[2][1] + box[3][1]) / 2
					
					dy = y_down - y_up
					dx = x_down - x_up
					
					dis_ratio = 0.15/abs(dy)

					angle = math.atan2(dy, dx)

					centrol = int((x_up+x_down)/2), int((y_up+y_down)/2)

					
					cv2.line(frame, (x_down, y_down), (x_up, y_up), (0, 255, 0), 2)
					cv2.circle(frame,(int((x_up+x_down)/2),int((y_up+y_down)/2)), 10, (0,0,255), 1)
					
					return frame, centrol, angle, dis_ratio
 
if __name__ == '__main__':
	try:
		de = depth_estimater()
		rospy.spin()
	except rospy.ROSInterruptException: pass