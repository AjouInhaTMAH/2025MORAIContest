#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2 
import numpy as np
from std_msgs.msg import Float64
from time import *
import path_lane_ctrl

class lane_dec:
    def __init__(self):
        # ROS 노드 초기화
        print(f"PerCamera start")
        self.init_pubSub()
        self.init_color()
        self.init_sliding()
        self.ctrl_path = path_lane_ctrl.ctrl_oath()
        
    def init_pubSub(self):
        rospy.init_node("lane_dec")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.CB_view_BEV, queue_size=1)
        self.img = CompressedImage()
        self.img = None
        self.bridge = CvBridge()
    def init_color(self):
        self.yellow_lower = np.array([15,128,0])
        self.yellow_upper = np.array([40,255,255])
        self.white_lower = np.array([0,0,192])
        self.white_upper = np.array([179,64,255])
    def init_sliding(self):
        self.midpoint = None
        self.is_left_lane = True
        self.nwindows = 10
        self.img_y, self.img_x = 0, 0
        
        self.window_height = None
        self.nonzero = None
        self.nonzeroy = None
        self.nonzerox = None
        self.left_blocked_flag = 0
        self.right_blocked_flag = 0
        self.alpha = 1
        
        self.left_lane_start = None
        self.left_lane_end = None
        self.right_lane_start = None
        self.right_lane_end = None
        
        self.left_lanes =[]
        self.right_lanes =[]
        
    def CB_view_BEV(self,msg):
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg)
    
    def BEV_img_warp(self,filtered_img,y,x):
        src_point1 = [0,420]
        src_point2 = [275,260]
        src_point3 = [x - 275,260]
        src_point4 = [x,420]
        
        # src_point1 = [40,400]
        # src_point2 = [180,300]
        # src_point3 = [x-180,300]
        # src_point4 = [x-40,400]
        
        # src_point1 = [0,420]
        # src_point2 = [100,270]
        # src_point3 = [x-100,270]
        # src_point4 = [x-0,420]
        
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
    def extract_stop_line(self,white_bin_img, threshold=30000):
        # y축 방향으로 sum → 수평선은 y축 기준으로 sum값이 커짐
        horizontal_sum = np.sum(white_bin_img, axis=1)  # shape: (height,)

        # threshold 이상인 위치 찾기
        stop_line_indices = np.where(horizontal_sum > threshold)[0]
        if len(stop_line_indices) > 0:
            min_y = stop_line_indices[0]
            max_y = stop_line_indices[-1]
            return min_y, max_y
        else:
            return []
        
    def sliding_window_lane_calculation(self,x_current,y_current,prev_margin,x_prev,blocked_flag, binary_img, is_left_lane , color=(0,255,0)):
        flag = blocked_flag
        win_y_low = y_current - self.window_height // 2
        win_y_high = y_current + self.window_height // 2
        win_x_low = x_current - self.margin - prev_margin
        win_x_high = x_current + self.margin + prev_margin
        cv2.rectangle(binary_img, (win_x_low, win_y_low), (win_x_high, win_y_high), color, 2)
        good_inds = ((self.nonzeroy >= win_y_low) & (self.nonzeroy < win_y_high) &
                            (self.nonzerox >= win_x_low) & (self.nonzerox < win_x_high)).nonzero()[0]
        if len(good_inds) > self.minpix:
            mean_x = np.mean(self.nonzerox[good_inds])
            min_x = np.min(self.nonzerox[good_inds])
            max_x = np.max(self.nonzerox[good_inds])
            
            delta = mean_x + (mean_x - x_prev)
            x_current = int(self.alpha * mean_x + (1 - self.alpha) * delta)
            y_current -= self.window_height
            prev_margin = (max_x - min_x) // 4
            x_prev = x_current
            cv2.circle(binary_img, (x_current, y_current), radius=5, color=(0, 0, 255), thickness=-1)
            flag = 0
            if is_left_lane:
                self.left_lanes.append(x_current)
            else:
                self.right_lanes.append(x_current)
                
        else:
            y_current -= self.window_height
            flag += 1
        return x_current,y_current, flag, prev_margin, x_prev
    def sliding_window_adaptive(self,binary_img, nwindows=15, margin=80, minpix=100):
        binary_img_color = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
        histogram = np.sum(binary_img[binary_img.shape[0]//2:, :], axis=0)
        self.left_blocked_flag = 0
        self.right_blocked_flag = 0
        self.left_lanes =[]
        self.right_lanes =[]
        
        self.margin = margin
        self.minpix = minpix
        self.midpoint = histogram.shape[0] // 2
        self.left_lane_start = 160
        self.right_lane_start = 480

        self.window_height = binary_img.shape[0] // nwindows
        self.nonzero = binary_img.nonzero()
        self.nonzeroy = np.array(self.nonzero[0])
        self.nonzerox = np.array(self.nonzero[1])
        
        prev_left_margin = margin
        prev_right_margin = margin

        leftx_current = self.left_lane_start
        leftx_prev = self.left_lane_start
        lefty_current = binary_img.shape[0] - self.window_height // 2

        rightx_current = self.right_lane_start
        rightx_prev = self.right_lane_start
        righty_current = binary_img.shape[0] - self.window_height // 2

        for window in range(nwindows):
            # 윈도우 범위 (적응형 이동)
            if self.left_blocked_flag < 4:
                leftx_current, lefty_current, self.left_blocked_flag, prev_left_margin, leftx_prev = self.sliding_window_lane_calculation(leftx_current, lefty_current,
                                                                                                                                prev_left_margin, leftx_prev, 
                                                                                                                                self.left_blocked_flag, binary_img_color,self.is_left_lane)
            if self.right_blocked_flag < 4:
                rightx_current, righty_current, self.right_blocked_flag, prev_right_margin, rightx_prev  = self.sliding_window_lane_calculation(rightx_current, righty_current,
                                                                                                                                prev_right_margin, rightx_prev, 
                                                                                                                                self.right_blocked_flag, binary_img_color,not self.is_left_lane,(255,0,0))
        # print(self.left_lane_start)
        # print(self.right_lane_start)
        
        # print(self.left_lanes)
        # print(self.right_lanes)
        # print(self.left_lane_start < self.left_lane_end)
        # print(self.right_lane_start < self.right_lane_end)
        return binary_img_color, self.left_lanes, self.right_lanes

    def processing(self):
        while not rospy.is_shutdown():
            if self.img is None:
                continue
            # img =msg
            self.img_y, self.img_x = self.img.shape[0:2]
            self.window_height = np.int(self.img_y / self.nwindows)
            warped_img = self.BEV_img_warp(self.img,self.img_y,self.img_x)
            warped_img_hsv = cv2.cvtColor(warped_img,cv2.COLOR_BGR2HSV)
            yellow_filtered_img, white_filtered_img = self.detect_color_yAndw(warped_img,warped_img_hsv)
            yellow_bin_img,white_bin_img = self.img_binary_yAndw(yellow_filtered_img, white_filtered_img)
            stop_line = self.extract_stop_line(white_bin_img)
            yellow_lane_img, yellow_left_lane, yellow_right_lane = self.sliding_window_adaptive(yellow_bin_img)
            white_lane_img, white_left_lane, white_right_lane = self.sliding_window_adaptive(white_bin_img)
            
            
            self.dataset = [stop_line,yellow_left_lane, yellow_right_lane,white_left_lane, white_right_lane]
            self.ctrl_path.ctrl_move(self.dataset)            
            # self.ctrl_path.ctrl_move(self.dataset)
            
            # print(f"yellow {yellow_left_lane} {yellow_right_lane}")
            # print(f"white {white_left_lane} {white_right_lane}")
            
            # lane_detect = self.window_search(bin_img)
            # # 이미지를 확인하는 용도
            cv2.imwrite("img.png", self.img)
            cv2.imshow("img.png",self.img)
            cv2.imshow("warped_img",warped_img)
            cv2.imshow("yellow_filtered_img",yellow_filtered_img)
            cv2.imshow("white_filtered_img",white_filtered_img)
            # cv2.imshow("yellow_bin_img",yellow_bin_img)
            # cv2.imshow("white_bin_img",white_bin_img)
            # print(f"stop_line {stop_line}")
            cv2.imshow("yellow_lane_img",yellow_lane_img)
            if stop_line != []:
                min_y, max_y = stop_line
                # 흰색 이진 이미지를 컬러로 변환 (BGR) – 선 그리기 위해
                # 빨간색 수평선 그리기 (min_y, max_y 위치에)
                cv2.line(white_lane_img, (0, min_y), (white_lane_img.shape[1], min_y), (0, 0, 255), 2)
                cv2.line(white_lane_img, (0, max_y), (white_lane_img.shape[1], max_y), (0, 0, 255), 2)
            cv2.imshow("white_lane_img",white_lane_img)
            cv2.waitKey(1)
            # print(f"time1 {end - start1} ")
            self.img = None
        

def main():
    try:
        lane = lane_dec()
        while not rospy.is_shutdown():
            lane.processing()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()