#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import cv2 
import numpy as np

class CameraPreprocessor:
    def __init__(self):
        self.init_color()
    def init_color(self):
        self.yellow_lower = np.array([15,128,0])
        self.yellow_upper = np.array([40,255,255])
        self.white_lower = np.array([0,0,192])
        self.white_upper = np.array([179,64,255])
    def BEV_img_warp(self,filtered_img,y,x):
        delta = 25
        src_point1 = [-delta,400]
        src_point2 = [180,300] # 180
        src_point3 = [x-180,300] # 180
        src_point4 = [x+delta,400]
        src_points = np.float32([src_point1,src_point2,src_point3,src_point4])
        
        dst_point1 = [x//8,480]
        dst_point2 = [x//8,0]
        dst_point3 = [x//8*7,0]
        dst_point4 = [x//8*7,480]
        dst_points = np.float32([dst_point1,dst_point2,dst_point3,dst_point4])
        
        matrix = cv2.getPerspectiveTransform(src_points,dst_points)
        warped_img = cv2.warpPerspective(filtered_img,matrix,[x,y])
        # cv2.imwrite("warped_img.png", warped_img) # 저장하기 위한 코드
        return warped_img        

    def detect_color_yAndw(self,img,img_hsv):
        yellow_range = cv2.inRange(img_hsv,self.yellow_lower,self.yellow_upper)
        white_range = cv2.inRange(img_hsv,self.white_lower,self.white_upper)
        yellow_filtered_img = cv2.bitwise_and(img,img,mask=yellow_range)
        white_filtered_img = cv2.bitwise_and(img,img,mask=white_range)
        # cv2.imwrite("filtered_img.png", filtered_img) # 저장하기 위한 코드
        return yellow_filtered_img, white_filtered_img   

    def img_binary_yAndw(self,yellow_filtered_img, white_filtered_img):
        yellow_grayed_img = cv2.cvtColor(yellow_filtered_img,cv2.COLOR_BGR2GRAY)
        yellow_bin_img = np.zeros_like(yellow_grayed_img)
        yellow_bin_img[yellow_grayed_img>50] = 255
        white_grayed_img = cv2.cvtColor(white_filtered_img,cv2.COLOR_BGR2GRAY)
        white_bin_img = np.zeros_like(white_grayed_img)
        white_bin_img[white_grayed_img>50] = 255
        # cv2.imwrite("bin_img.png", bin_img) # 저장하기 위한 코드 
        return yellow_bin_img,white_bin_img
