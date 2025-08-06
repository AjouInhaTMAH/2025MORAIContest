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
import json
from std_msgs.msg import String
from utils import check_timer

class PerCamera:
    def __init__(self):
        # ROS 노드 초기화
        print(f"PerCamera start")
        self.init_pubSub()
        self.init_color()
        self.init_sliding()
        
    def init_pubSub(self):
        rospy.init_node("lane_dec")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.CB_view_BEV, queue_size=1)
        self.img = CompressedImage()
        self.img = None
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/perception/camera', String, queue_size=10)

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
        delta = 0
        src_point1 = [-delta,400]
        src_point2 = [180,300] # 180
        src_point3 = [x-180,300] # 180
        src_point4 = [x+delta,400]
        
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
    
    def extract_stop_line(self, white_bin_img, threshold=300):
        # y축 방향으로 sum → 수평선은 y축 기준으로 sum값이 커짐
        horizontal_sum = np.sum(white_bin_img, axis=1) // 255  # shape: (height,)
        down_hist = horizontal_sum[self.img_y//4*2:]
        
        # threshold 이상인 위치 찾기
        stop_line_indices = np.where(down_hist > threshold)[0] + self.img_y//4*2
        if len(stop_line_indices) > 0:
            min_y = stop_line_indices[0]
            max_y = stop_line_indices[-1]
            
            # 디텍팅된 구간을 검은색(0)으로 만듦
            white_bin_img[min_y:max_y+1, :] = 0

            return [int(min_y), int(max_y)], white_bin_img
        else:
            return [], white_bin_img

        
    def sliding_window_lane_calculation(self, x_current, y_current, prev_margin, x_prev,
                                        blocked_flag, binary_img, is_left_lane, color=(0, 255, 0)):
        flag = blocked_flag
        win_y_low = y_current - self.window_height // 2
        win_y_high = y_current + self.window_height // 2
        win_x_low = x_current - self.margin - prev_margin
        win_x_high = x_current + self.margin + prev_margin

        # 이미지 가로 중간값 (중앙 기준 나누기 위해)
        img_center_x = binary_img.shape[1] // 2

        # 좌/우 차선에 따라 검색 범위 제한
        if is_left_lane:
            if win_x_high > img_center_x:
                win_x_high = img_center_x
            if win_x_low > img_center_x:
                return x_current, y_current - self.window_height, flag + 1, prev_margin, x_prev
        else:  # 오른쪽 차선
            if win_x_low < img_center_x:
                win_x_low = img_center_x
            if win_x_high < img_center_x:
                return x_current, y_current - self.window_height, flag + 1, prev_margin, x_prev

        # 윈도우 그리기
        cv2.rectangle(binary_img, (win_x_low, win_y_low), (win_x_high, win_y_high), color, 2)

        # 해당 윈도우 내 픽셀 찾기
        good_inds = ((self.nonzeroy >= win_y_low) & (self.nonzeroy < win_y_high) &
                    (self.nonzerox >= win_x_low) & (self.nonzerox < win_x_high)).nonzero()[0]

        if len(good_inds) > self.minpix:
            min_x = np.min(self.nonzerox[good_inds])
            max_x = np.max(self.nonzerox[good_inds])

            if (max_x - min_x) > 100:  # 윈도우가 너무 넓으면 무시
                y_current -= self.window_height
                flag += 1
                return x_current, y_current, flag, self.margin, x_prev

            center_y = y_current
            mean_x = np.mean(self.nonzerox[good_inds])
            delta = mean_x + (mean_x - x_prev)
            x_current = int(self.alpha * mean_x + (1 - self.alpha) * delta)
            y_current = center_y - self.window_height
            prev_margin = min((max_x - min_x) // 4, 20)
            x_prev = x_current

            cv2.circle(binary_img, (x_current, y_current), radius=5, color=(0, 0, 255), thickness=-1)
            flag = 0

            if is_left_lane:
                self.left_lanes.append((x_current, y_current))
            else:
                self.right_lanes.append((x_current, y_current))

        else:
            y_current -= self.window_height
            flag += 1

        return x_current, y_current, flag, prev_margin, x_prev
    
    def sliding_window_adaptive(self,binary_img, nwindows=15, margin=80, minpix=100):
        # margin=80
        margin= 75
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
        self.right_lane_start = 480 #480

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
            # print(f"rightx_current {rightx_current}")
            if self.left_blocked_flag < 8:
                leftx_current, lefty_current, self.left_blocked_flag, prev_left_margin, leftx_prev = self.sliding_window_lane_calculation(leftx_current, lefty_current,
                                                                                                                                prev_left_margin, leftx_prev, 
                                                                                                                                self.left_blocked_flag, binary_img_color,self.is_left_lane)
            if self.right_blocked_flag < 8:
                rightx_current, righty_current, self.right_blocked_flag, prev_right_margin, rightx_prev  = self.sliding_window_lane_calculation(rightx_current, righty_current,
                                                                                                                                prev_right_margin, rightx_prev, 
                                                                                                                                self.right_blocked_flag, binary_img_color,False,(255,0,0)) #not self.is_left_lane
        # print(self.left_lane_start)
        # print(self.right_lane_start)
        
        #print(self.left_lanes)
        #print(self.right_lanes)
        # print(self.left_lane_start < self.left_lane_end)
        # print(self.right_lane_start < self.right_lane_end)
        return binary_img_color, self.left_lanes, self.right_lanes

    def processing(self):
        while not rospy.is_shutdown():
            if self.img is None:
                continue
            # img =msg
            self.img_y, self.img_x = self.img.shape[0:2]
            self.window_height = int(self.img_y / self.nwindows)
            warped_img = self.BEV_img_warp(self.img,self.img_y,self.img_x)
            warped_img_hsv = cv2.cvtColor(warped_img,cv2.COLOR_BGR2HSV)
            yellow_filtered_img, white_filtered_img = self.detect_color_yAndw(warped_img,warped_img_hsv)
            yellow_bin_img,white_bin_img = self.img_binary_yAndw(yellow_filtered_img, white_filtered_img)
            stop_line, white_bin_img= self.extract_stop_line(white_bin_img)
            yellow_lane_img, yellow_left_lane, yellow_right_lane = self.sliding_window_adaptive(yellow_bin_img)
            white_lane_img, white_left_lane, white_right_lane = self.sliding_window_adaptive(white_bin_img)
            
            self.dataset = [stop_line, yellow_left_lane, yellow_right_lane,white_left_lane, white_right_lane]           
            #self.lane_cotrol.ctrl_decision(self.dataset)
            self.dataset = [stop_line,yellow_left_lane, yellow_right_lane,white_left_lane, white_right_lane]
            json_str = json.dumps(self.dataset)
            self.pub.publish(json_str)

            combined_img = cv2.bitwise_or(white_lane_img,yellow_lane_img)
            if stop_line != []:
                cross_threshold = 35
                min_y, max_y = stop_line
                cross_diff = (max_y - min_y)
                if cross_threshold < cross_diff:
                    cv2.rectangle(combined_img,[0,min_y],[self.img_x, max_y],[0,0,255],3)
            #cv2.imshow("white_lane_img",white_lane_img)
            cv2.imshow("lane_img",combined_img)
            cv2.imshow("self.img",self.img)
            cv2.waitKey(1)
            # print(f"time1 {end - start1} ")
            self.img = None
     
        
if __name__ == '__main__':
    node = PerCamera()
    node.processing()   # spin 대신 processing 돌기
    
