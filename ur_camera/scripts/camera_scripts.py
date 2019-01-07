#! /usr/bin/env python
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image

class Detection:

	def __init__(self):
		self.color_hsv = []
		self.cam = cv2.VideoCapture(0)
		#self.cam_kinect = np.array()
		self.color_hsv = [81, 110, 0, 255, 255, 255]
		rospy.init_node('cam')

	def Callback_kinect(self, ros_data):
		np_arr = np.fromstring(ros_data.data, np.int8)
		self.cam_kinect = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		print('sub')

	def Camera_begin(self):
		lower_hsv = np.array(self.color_hsv[:3])
		upper_hsv = np.array(self.color_hsv[3:6])

		while True:
			ret_val, frame = self.cam.read()
			sub = rospy.Subscriber('/camera/rgb/image_color', Image, self.Callback_kinect)
			#frame = self.cam
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

						x_l = (box[0][0] + box[1][0]) / 2
						y_l = (box[0][1] + box[1][1]) / 2
						x_r = (box[2][0] + box[3][0]) / 2
						y_r = (box[2][1] + box[3][1]) / 2
						
						cv2.line(frame, (x_l, y_l), (x_r, y_r), (0, 255, 0), 2)
			cv2.imshow("Cam Viewer",frame)
			if cv2.waitKey(1) == 27:
				break


if __name__ == '__main__':
	run = Detection()
	run.Camera_begin()
