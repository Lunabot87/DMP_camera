#! /usr/bin/env python
import numpy as np
import cv2
import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Image, PointCloud2, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import	message_filters

class Detection:

	def __init__(self):
		self.color_hsv = []

		self.temp_depth = Int16(0)

		self.image_pub = rospy.Publisher("image_topic",Image, queue_size=5)
		self.depth_pub = rospy.Publisher("depth_topic",Int16, queue_size=5)
		self.cam = cv2.VideoCapture(0)
		#self.cam_kinect = np.array()
		self.cam_kinect = Image()
		self.bridge = CvBridge()
		self.color_hsv = [81, 110, 0, 255, 255, 255]
		rospy.init_node('cam', anonymous=True)

	def Callback_kinect(self, ros_data):
		self.cam_kinect = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
		#np_arr = np.fromstring(ros_data.data, np.int8)
		#cam_kinect = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		#self.cam_depth = rospy.wait_for_message('/camera/depth/points', PointCloud2)
		#np_arr = np.fromstring(ros_data.data, np.int8)
		#self.cam_depth = cv2.imdecode(np_arr, cv2.IMREAD_DEPTH)
		self.Camera_filter()

	def Callback_depth(self,ros_data):
		#print(ros_data.data)
		#self.cam_depth = self.bridge.imgmsg_to_cv2(ros_data, "8UC1")
		#print(self.cam_depth)

		#self.cam_dapth = np.array(self.cam_depth, dtype=np.float32)
		#print('--------------')
		#print(self.cam_depth)
		
		np_arr = np.fromstring(ros_data.data, np.uint8)
		print(np_arr.shape)
		self.cam_depth = cv2.imdecode(np_arr, cv2.IMREAD_ANYDEPTH)
		print(self.cam_depth)
		#self.cam_depth = np.array(self.cam_depth, dtype=np.float32)
		#self.cam_depth = cv2.normalize(self.cam_depth, self.cam_depth, 0, 1, cv2.NORM_MINMAX)
  
		

	def Camera_filter(self):
		try:
			frame = self.cam_kinect
		except CvBridgeError, e:
			print e	
		lower_hsv = np.array(self.color_hsv[:3])
		upper_hsv = np.array(self.color_hsv[3:6])
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
					cv2.circle(frame,(x_r,int((y_r+y_l)/2)), 10, (0,0,255), 1)
					#self.temp_depth = self.cam_depth[x_r+int((y_r+y_l)/2)*480]
					#self.temp_depth = self.cam_depth[0][0]
			
		#cv2.DestroyAllWindows()
		try:
		 	self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "8UC3"))
		 	self.depth_pub.publish(self.temp_depth)
		except CvBridgeError, e:
		 	print e
		#cv2.imshow("Cam Viewer",frame)
		

	def main(self):
		image_raw = rospy.Subscriber('/camera/rgb/image_color', Image, self.Callback_kinect)
		depth_raw = rospy.Subscriber('/camera/depth/image_raw/compressed', CompressedImage, self.Callback_depth)
		rospy.spin()



if __name__ == '__main__':
	run = Detection()
	run.main()
