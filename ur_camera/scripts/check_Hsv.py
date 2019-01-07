#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import simple_cv

def onTrackbarActivity(x):
    global show
    show = True
    pass
    
def check_Hsv():
    rospy.init_node('check_hsv', anonymous=True)
    font = cv2.FONT_HERSHEY_SIMPLEX
    #video = cv2.VideoCapture(1)
    video = simple_cv.get_video()

    initialX = 50
    initialY = 50

    # creating windows to display images
    cv2.namedWindow('P-> Previous, N-> Next', cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('SelectHSV', cv2.WINDOW_AUTOSIZE)

    # moving the windows to stack them horizontally
    cv2.moveWindow('P-> Previous, N-> Next', initialX, initialY)
    cv2.moveWindow('SelectHSV', initialX, initialY)
    # creating trackbars to get values for HSV
    cv2.createTrackbar('HMin', 'SelectHSV', 0, 180, onTrackbarActivity)
    cv2.createTrackbar('HMax', 'SelectHSV', 0, 180, onTrackbarActivity)
    cv2.createTrackbar('SMin', 'SelectHSV', 0, 255, onTrackbarActivity)
    cv2.createTrackbar('SMax', 'SelectHSV', 0, 255, onTrackbarActivity)
    cv2.createTrackbar('VMin', 'SelectHSV', 0, 255, onTrackbarActivity)
    cv2.createTrackbar('VMax', 'SelectHSV', 0, 255, onTrackbarActivity)

    cv2.namedWindow("Corrected Perspective", cv2.WINDOW_AUTOSIZE)

    while True:

        k = cv2.waitKey(1) & 0xFF

        # Get values from the HSV trackbar
        HMin = cv2.getTrackbarPos('HMin', 'SelectHSV')
        SMin = cv2.getTrackbarPos('SMin', 'SelectHSV')
        VMin = cv2.getTrackbarPos('VMin', 'SelectHSV')
        HMax = cv2.getTrackbarPos('HMax', 'SelectHSV')
        SMax = cv2.getTrackbarPos('SMax', 'SelectHSV')
        VMax = cv2.getTrackbarPos('VMax', 'SelectHSV')


        OriginalFrame = video
        img_hsv = cv2.cvtColor(OriginalFrame, cv2.COLOR_BGR2HSV)

        lower_color = np.array([HMin, SMin, VMin])
        upper_color = np.array([HMax, SMax, VMax])

        mask = cv2.inRange(img_hsv, lower_color, upper_color)
        img_result = cv2.bitwise_and(OriginalFrame,OriginalFrame, mask= mask)

        #cv2.imshow("Main Frame", OriginalFrame)
        cv2.imshow('P-> Previous, N-> Next', OriginalFrame)
        cv2.imshow("Corrected Perspective", img_result)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break

    video.release()
    cv2.destroyAllWindows()
if __name__ == '__main__':
    try:
        check_Hsv()
    except rospy.ROSInterruptException:
        pass
