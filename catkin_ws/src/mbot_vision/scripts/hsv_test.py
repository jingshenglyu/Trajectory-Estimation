#!/usr/bin/env python
# coding=utf-8
'''
Author       : Jingsheng Lyu
Date         : 2021-05-29 14:50:43
LastEditors  : Jingsheng Lyu
LastEditTime : 2021-09-25 17:47:57
FilePath     : /undefined/home/jingsheng/catkin_ws/src/mbot_vision/scripts/hsv_test.py
Github       : https://github.com/jingshenglyu
Web          : https://jingshenglyu.github.io/
E-Mail       : jingshenglyu@gmail.com
'''
# -*- coding:utf-8 -*-
    
import cv2

img = cv2.imread('image.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


def mouse_click(event, x, y, flags, para):
    if event == cv2.EVENT_LBUTTONDOWN:  # Left mouse click
        #print('PIX:', x, y)
        print("BGR:", img[y, x])
        #print("GRAY:", gray[y, x])
        #print("HSV:", hsv[y, x])


if __name__ == '__main__':
    cv2.namedWindow("img")
    cv2.setMouseCallback("img", mouse_click)
    while True:
        cv2.imshow('img', img)
        if cv2.waitKey() == ord('q'):
            break
    cv2.destroyAllWindows()
