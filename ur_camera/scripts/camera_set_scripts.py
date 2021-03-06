#! /usr/bin/env python
# -*- coding: utf-8 -*

import cv2
import numpy as np
import time

class Camera_set:

    def __init__(self):
        self.color = {}
        self.cap = cv2.VideoCapture(0)
        self.color['cli'] = []
        self.color['target'] = []
        self.color['goal'] = []

    def nothing(self):
        pass

    def data_read(self):
        try:
            data_f = open("cli.txt",'r')

            while True:
                line = data_f.readline()
                if not line: break
                self.color['cli'].append(int(line))
            data_f.close()
        except:
            pass


        try:
            data_f = open("target.txt",'r')

            while True:
                line = data_f.readline()
                if not line: break
                self.color['target'].append(int(line))
            data_f.close()
        except:
            pass


        try:
            data_f = open("goal.txt",'r')

            while True:
                line = data_f.readline()
                if not line: break
                self.color['goal'].append(int(line))
            data_f.close()
        except:
            pass

    def camera_begin(self):
        cv2.namedWindow('cli')
        cv2.namedWindow('target')
        cv2.namedWindow('goal')

        cv2.createTrackbar('low H','cli',0,255,self.nothing)
        cv2.createTrackbar('high H','cli',0,255,self.nothing)
        cv2.createTrackbar('low S','cli',0,255,self.nothing)
        cv2.createTrackbar('high S','cli',0,255,self.nothing)
        cv2.createTrackbar('low V','cli',0,255,self.nothing)
        cv2.createTrackbar('high V','cli',0,255,self.nothing)
        
        cv2.createTrackbar('low H','target',0,255,self.nothing)
        cv2.createTrackbar('high H','target',0,255,self.nothing)
        cv2.createTrackbar('low S','target',0,255,self.nothing)
        cv2.createTrackbar('high S','target',0,255,self.nothing)
        cv2.createTrackbar('low V','target',0,255,self.nothing)
        cv2.createTrackbar('high V','target',0,255,self.nothing)
        
        cv2.createTrackbar('low H','goal',0,255,self.nothing)
        cv2.createTrackbar('high H','goal',0,255,self.nothing)
        cv2.createTrackbar('low S','goal',0,255,self.nothing)
        cv2.createTrackbar('high S','goal',0,255,self.nothing)
        cv2.createTrackbar('low V','goal',0,255,self.nothing)
        cv2.createTrackbar('high V','goal',0,255,self.nothing)


#        if len(self.color['cli']) != 0:
#                cv2.setTrackbarPos('low H','cli',self.color['cli'][0])
#                cv2.setTrackbarPos('high H','cli',self.color['cli'][1])
#                cv2.setTrackbarPos('low S','cli',self.color['cli'][2])
#                cv2.setTrackbarPos('high S','cli',self.color['cli'][3])
#                cv2.setTrackbarPos('low V','cli',self.color['cli'][4])
#                cv2.setTrackbarPos('high V','cli',self.color['cli'][5])

#
#        if len(self.color['target']) != 0:
#                cv2.setTrackbarPos('low H','target',self.color['target'][0])
#                cv2.setTrackbarPos('high H','target',self.color['target'][1])
#                cv2.setTrackbarPos('low S','target',self.color['target'][2])
#                cv2.setTrackbarPos('high S','target',self.color['target'][3])
#                cv2.setTrackbarPos('low V','target',self.color['target'][4])
#                cv2.setTrackbarPos('high V','target',self.color['target'][5])
#

#        if len(self.color['goal']) != 0:
#                cv2.setTrackbarPos('low H','goal',self.color['goal'][0])
#                cv2.setTrackbarPos('high H','goal',self.color['goal'][1])
#                cv2.setTrackbarPos('low S','goal',self.color['goal'][2])
#                cv2.setTrackbarPos('high S','goal',self.color['goal'][3])
#                cv2.setTrackbarPos('low V','goal',self.color['goal'][4])
#                cv2.setTrackbarPos('high V','goal',self.color['goal'][5])



    def main(self):
        prevTime = 0
        self.data_read()
        self.camera_begin()
        ret, frame = self.cap.read()

        frame = cv2.GaussianBlur(frame, (5,5), 0)
        H_low = cv2.getTrackbarPos('low_H', 'cli')
        H_high = cv2.getTrackbarPos('high_H', 'cli')
        S_low = cv2.getTrackbarPos('low_S', 'cli')
        S_high = cv2.getTrackbarPos('high_S', 'cli')
        V_low = cv2.getTrackbarPos('low_V', 'cli')
        V_high = cv2.getTrackbarPos('high_V', 'cli')

        H_low_t = cv2.getTrackbarPos('low_H', 'target')
        H_high_t = cv2.getTrackbarPos('high_H', 'target')
        S_low_t = cv2.getTrackbarPos('low_S', 'target')
        S_high_t = cv2.getTrackbarPos('high_S', 'target')
        V_low_t = cv2.getTrackbarPos('low_V', 'target')
        V_high_t = cv2.getTrackbarPos('high_V', 'target')
        
        H_low_g = cv2.getTrackbarPos('low_H', 'goal')
        H_high_g = cv2.getTrackbarPos('high_H', 'goal')
        S_low_g = cv2.getTrackbarPos('low_S', 'goal')
        S_high_g = cv2.getTrackbarPos('high_S', 'goal')
        V_low_g = cv2.getTrackbarPos('low_V', 'goal')
        V_high_g = cv2.getTrackbarPos('high_V', 'goal')

        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        self.color['cli'] = {H_low, H_high, S_low, S_high, V_low, V_high}
        self.color['target'] = {H_low_t, H_high_t, S_low_t, S_high_t, V_low_t, V_high_t}
        self.color['goal'] = {H_low_g, H_high_g, S_low_g, S_high_g, V_low_g, V_high_g}

        cv2.imshow('kinect', frame)

        img_h, img_s, img_v = cv2.split(img_hsv)
        cv2.imshow('h', img_h)
        cv2.imshow('s', img_s)
        cv2.imshow('v', img_v)

        lower_blue = np.array([H_low, S_low, V_low])
        upper_blue = np.array([H_high, S_high, V_high])

        lower_red = np.array([H_low_t, S_low_t, V_low_t])
        upper_red = np.array([H_high_t, S_high_t, V_high_t])

        lower_p=np.array([H_low_g, S_low_g, V_low_g])
        upper_p=np.array([H_high_g, S_high_g, V_high_g])

        mask_1 = cv2.inRange(img_hsv, lower_blue, upper_blue)
        mask_2 = cv2.inRange(img_hsv, lower_red, upper_red)
        mask_3 = cv2.inRange(img_hsv, lower_p, upper_p)

        img_result_1 = cv2.bitwise_and(frame, frame, mask=mask_1)
        img_result_2=cv2.bitwise_and(frame,frame,mask=mask_2)
        img_result_3=cv2.bitwise_and(frame,frame,mask=mask_3)

        curTime=time.time()
        sec = curTime-prevTime
        prevTime = curTime
        fps = 1/(sec)
        str = "FPS : %0.1f" % fps

        cv2.putText(img_result_1, str, (0,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0))
        cv2.imshow('cli', img_result_1)
        cv2.imshow('target', img_result_2)
        cv2.imshow('goal', img_result_3)

        if cv2.waitKey(30) & 0xFF == ord('q'):
           
            data_f = open("cli.txt",'w')

            for i in range(0,6):
                data_f.write(str(self.color['cli']))
                data_f.write('\n')
            data_f.close()
            

            data_f = open("target.txt",'w')

            for i in range(0,6):
                data_f.write(str(self.color['target']))
                data_f.write('\n')
            data_f.close()

            data_f = open("goal.txt",'w')

            for i in range(0,6):
                data_f.write(str(self.color['goal']))
                data_f.write('\n')
            data_f.close()
                    
                    
if __name__ == '__main__':
    a = Camera_set()
    while True: 
        a.main()
