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
    WIDTH = 5
    HEIGHT = 5
    EULER = [0.0,0.0,0.0]
 
    def __init__(self):
 
        rospy.init_node('depth_estimater', anonymous=True)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("image_topic",Image, queue_size=5)
        sub_rgb = message_filters.Subscriber("camera/rgb/image_color",Image)
        sub_depth = message_filters.Subscriber("camera/depth/image",Image)
        self.mf = message_filters.ApproximateTimeSynchronizer([sub_rgb, sub_depth], 100, 10.0)
        self.mf.registerCallback(self.ImageCallback)
        self.color_hsv = {}
        self.color_hsv['initial'] = [71, 133, 108, 120, 255, 255]
        self.color_hsv['goal'] = [19, 64, 209, 60, 255, 255]
        self.color_hsv['obstacle'] = [118, 166, 63, 171, 255, 202]
        self.br = tf.TransformBroadcaster()
        self.ave = 0.0
 
    def ImageCallback(self, rgb_data , depth_data):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(rgb_data, 'passthrough')
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_data, 'passthrough')
        except CvBridgeError, e:
            rospy.logerr(e)
 
        self.color_image.flags.writeable = True
        

        goal_depth= self.image_filter(self.color_image, 'goal')
        target_depth= self.image_filter(self.color_image, 'initial')
        obstacle_depth= self.image_filter(self.color_image, 'obstacle')

        self.tr_transform(target_depth, 'initial')
        self.tr_transform(goal_depth, 'goal')
        self.tr_transform(obstacle_depth, 'obstacle')

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.color_image, "8UC3"))
        quat = quaternion_from_euler(0, -1.5708, 0)
        self.br.sendTransform([1.098, 0.115, -0.42],quat,rospy.Time.now(),'ur3_base',"kinect")

        #print("%f [m], goal" % goal_depth[1])
        #print("%f [m], target" % target_depth[0])


    def tr_transform(self, euler, name):
        h, w, c = self.color_image.shape
        ratio = (1.002/510)*(euler[0]/1.093) #
        self.EULER[0] = euler[0]
        self.EULER[1] = ((w/2) - euler[1])*ratio
        self.EULER[2] = ((h/2) - euler[2])*ratio 
        print(self.EULER)
        #print(self.EULER[1])
        #print("y")
        #print(self.EULER[2])
        quat = quaternion_from_euler(0, -1.5708, 0)
        self.br.sendTransform(self.EULER,quat,rospy.Time.now(),name,"kinect")

 
    def image_filter(self, frame, color):

        lower_hsv = np.array(self.color_hsv[color][:3])
        upper_hsv = np.array(self.color_hsv[color][3:6])

        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        hsv_mask = cv2.inRange(frame_hsv, lower_hsv, upper_hsv)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        ret, thr = cv2.threshold(hsv_mask, 127, 255, 0)
        _, contours, _ = cv2.findContours(thr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            for i in range(len(contours)):
                area = cv2.contourArea(contours[i])
                if area > 50:
                    rect = cv2.minAreaRect(contours[i])
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    box = sorted(box, key=lambda box: box[1])

                    x_up = (box[0][0] + box[1][0]) / 2
                    y_up = (box[0][1] + box[1][1]) / 2
                    x_down = (box[2][0] + box[3][0]) / 2
                    y_down = (box[2][1] + box[3][1]) / 2

                    centrol = int((x_up+x_down)/2), int((y_up+y_down)/2)

                    cv2.line(frame, (x_down, y_down), (x_up, y_up), (0, 255, 0), 2)
                    cv2.circle(frame,(int((x_up+x_down)/2),int((y_up+y_down)/2)), 10, (0,0,255), 1)

                    x1 = centrol[0] - self.WIDTH
                    x2 = centrol[0] + self.WIDTH
                    y1 = centrol[1] - self.HEIGHT
                    y2 = centrol[1] + self.HEIGHT

                    origin2point = ((320-centrol[0])**2 + (240-centrol[1])**2)**0.5
                    ratio = origin2point/400
                    ratio = ratio*25
                    ratio = math.cos(ratio*(math.pi/180))

                    #print("%f [], ratio" % ratio)

                    sum = 0.0
                    pixel_count = 0
             
                    for i in range(y1, y2):
                        for j in range(x1, x2):
                            self.color_image.itemset((i, j, 0), 0)
                            self.color_image.itemset((i, j, 1), 0)
                            #self.color_image.itemset((100,100,2), 0)
             
                            if self.depth_image.item(i,j) == self.depth_image.item(i,j):
                                pixel_count += 1
                                #print(self.depth_image.item(i,j))
                                sum += self.depth_image.item(i,j)

                    temp = self.ave
                    if pixel_count == 0:
                        pixel_count = 1
                    self.ave = sum / pixel_count
                    if self.ave <= 0.5:
                        self.ave = temp

                    euler = [self.ave*ratio,centrol[0],centrol[1]]
                    
                    return euler
            else:
                euler = [float('nan'), float('nan'),  float('nan')]
                return euler


 
if __name__ == '__main__':
    try:
        de = depth_estimater()
        rospy.spin()
    except rospy.ROSInterruptException: pass