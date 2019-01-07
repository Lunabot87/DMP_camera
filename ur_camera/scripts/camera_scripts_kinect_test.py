#! /usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import sys
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
 
class depth_estimater:
    WIDTH = 10
    HEIGHT = 10
 
    def __init__(self):
 
        rospy.init_node('depth_estimater', anonymous=True)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("image_topic",Image, queue_size=5)
        sub_rgb = message_filters.Subscriber("camera/rgb/image_color",Image)
        sub_depth = message_filters.Subscriber("camera/depth/image",Image)
        self.mf = message_filters.ApproximateTimeSynchronizer([sub_rgb, sub_depth], 100, 10.0)
        self.mf.registerCallback(self.ImageCallback)
 
    def ImageCallback(self, rgb_data , depth_data):
        try:
            color_image = self.bridge.imgmsg_to_cv2(rgb_data, 'passthrough')
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, 'passthrough')
        except CvBridgeError, e:
            rospy.logerr(e)
 
        color_image.flags.writeable = True
        h, w, c = color_image.shape
 
        x1 = (w/2) - self.WIDTH
        x2 = (w/2) + self.WIDTH
        y1 = (h / 2) - self.HEIGHT
        y2 = (h /2 ) + self.HEIGHT
        sum = 0.0
        pixel_count = 0
 
        for i in range(y1, y2):
            for j in range(x1, x2):
                color_image.itemset((i, j, 0), 0)
                color_image.itemset((i, j, 1), 0)
                #color_image.itemset((100,100,2), 0)
 
                if depth_image.item(i,j) == depth_image.item(i,j):
                    #print(depth_image.item(i,j))
                    pixel_count += 1
                    sum += depth_image.item(i,j)
 
        ave = sum / pixel_count

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(color_image, "8UC3"))

        print("%f [m]" % ave)
 
        #cv2.normalize(depth_image, depth_image, 0, 1, cv2.NORM_MINMAX)
        #cv2.namedWindow("color_image")
        #cv2.namedWindow("depth_image")
        #cv2.imshow("color_image", color_image)
        #cv2.imshow("depth_image", depth_image)
        #cv2.waitKey(10)
 
if __name__ == '__main__':
    try:
        de = depth_estimater()
        rospy.spin()
    except rospy.ROSInterruptException: pass