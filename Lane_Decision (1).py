#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import rospy
#from sensor_msgs.msg import CompressedImage
#from cv_bridge import CvBridge
#import cv2 
import numpy as np
#from std_msgs.msg import Float64
import time
import Lane_Controller
from Traffic import Traffic_Signal
import Motor

class lane_dec:
    def __init__(self):
        self.lane_ctrl = Lane_Controller.lane_ctrl()
        self.traffic_signal = Traffic_Signal()
        self.motor = Motor.motor_ctrl()
        self.center_pixel = 320
        self.center_index = 0
        #self.stop_flag = self.perception_node.cross_flag_check()
        self.LANE_WIDTH_PIXELS = 260 # 적절히 조정
        
        self.sf4_end_time = None
        self.prev_stop_flag_num = None

        self.run_timer = None
        self.run_end_time = None
        self.sf4_latched = False

    def calculate_curvature(self, left_lane, right_lane):
        x_vals, y_vals = [], []

        if left_lane:
            for pt in left_lane:
                x_vals.append(pt[0])
                y_vals.append(pt[1])

        elif right_lane:
            for pt in right_lane:
                x_vals.append(pt[0])
                y_vals.append(pt[1])

        # 최소 데이터 개수 확인 (2개 미만이면 근사 자체가 불가능함)
        if len(x_vals) < 4:
            return 1e4  # 데이터 부족 시 직선으로 간주

        try:
            # 2차 다항 근사 (y에 대한 x 곡선으로)
            fit = np.polyfit(y_vals, x_vals, 2)
            A, B = fit[0], fit[1]

            y_eval = np.max(y_vals)  # 일반적으로 이미지 하단 기준

            denominator = abs(2*A)
            if denominator < 1e-4:
                denominator = 1e-4  # 분모 폭발 방지

            curvature = ((1 + (2*A*y_eval + B)**2)**1.5) / denominator

            # 비정상적으로 큰 곡률 제한 (클수록 직선으로 처리)
            if curvature > 10000:
                curvature = 10000

            return curvature

        except Exception as e:
            rospy.logwarn(f"[Curvature Calculation Error] {e}")
            return 1e4  # 계산 실패 시 직선으로 간주

    def ctrl_decision(self, dataset):
        stop_flag, yellow_left, yellow_right, white_left, white_right = dataset

        # 한 쪽 차선만 들어오는 경우를 위한 통합 처리
        left_white_lane = white_left
        left_yellow_lane = yellow_left
        right_lane = white_right
        
        if left_yellow_lane and stop_flag:
            return self.move_forward(left_yellow_lane)
        
        elif left_white_lane and right_lane:
            return self.which_lane(left_white_lane, right_lane)
        
        elif left_yellow_lane and right_lane:
            return self.which_lane(left_yellow_lane, right_lane)
        
        elif left_white_lane and not right_lane:
            return self.follow_left_lane(left_white_lane)

        elif left_yellow_lane and not right_lane:
            return self.follow_left_lane(left_yellow_lane)

        elif right_lane and not left_white_lane:
            return self.follow_right_lane(right_lane)

        else:
            return self.forward()
        
    def mode_order(self, mode, dataset):
        stop_flag, yellow_left, yellow_right, white_left, white_right = dataset

        left_lane = yellow_left if yellow_left else white_left
        right_lane = white_right
        

        if mode == "move_to_1st_lane":
            self.center_index = 160
            return left_lane, right_lane
        elif mode == "move_to_2nd_lane":
            self.center_index = 480
            return left_lane, right_lane
            
        elif mode == "follow_left_lane":
            if not left_lane:
                self.center_index = self.center_pixel
            else:
                left_index = (left_lane[0][0] + left_lane[-1][0]) // 2
                self.center_index = left_index + self.LANE_WIDTH_PIXELS // 2  # 가상의 중심선
            return left_lane, []
        elif mode == "follow_right_lane":
            if not right_lane:
                self.center_index = self.center_pixel
            else:
                right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
                self.center_index = right_index - self.LANE_WIDTH_PIXELS // 2  # 가상의 중심선
            return [], right_lane
        
        elif mode == "stop":
            self._cancel_all_timers()
            self.motor.publish(0,0.5)
            return [], []
            
    def which_lane(self,left_lane, right_lane):
        left_index = (left_lane[0][0] + left_lane[-1][0]) // 2
        right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
        weight_left = len(left_lane)
        weight_right = len(right_lane)
        
        if weight_left > weight_right:
            left_index = (left_lane[0][0] + left_lane[-1][0]) // 2
            self.center_index = left_index + self.LANE_WIDTH_PIXELS // 2  # 가상의 중심선
            return "first_lane", left_lane, []        
        else:
            right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
            self.center_index = right_index - self.LANE_WIDTH_PIXELS // 2  # 가상의 중심선
            return "second_lane", [], right_lane

    def follow_left_lane(self, left_lane):
        left_index = (left_lane[0][0] + left_lane[-1][0]) // 2
        self.center_index = left_index + self.LANE_WIDTH_PIXELS // 2  # 가상의 중심선
        return "left_guided", left_lane, []

    def follow_right_lane(self, right_lane):
        right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
        self.center_index = right_index - self.LANE_WIDTH_PIXELS // 2  # 가상의 중심선
        return "right_guided", [], right_lane
        
    def forward(self):
        self.center_index = self.center_pixel  # 정중앙
        return "go_straight", [], []

    def move_forward(self,left_lane):
        self.center_index = self.center_pixel  # 정중앙
        print("movemove")
        return "move_straight", left_lane, []

    def decision(self,dataset):
        stop_flag = dataset[0]
        stop_flag_num = self.traffic_signal.flag_CB(stop_flag)
            
        if stop_flag_num == 6:
            left_lane, right_lane = self.mode_order("follow_left_lane",dataset)
    
        elif stop_flag and signal in (33,16) and stop_flag_num > 5: 
            left_lane, right_lane = self.mode_order("follow_left_lane", dataset)
        
        elif stop_flag and signal in (1,4) and stop_flag_num > 5:
            self.mode_order("stop", dataset)
            return
        
        else:
            state, left_lane, right_lane = self.ctrl_decision(dataset)
            if state == "move_straight":
               self.start_run_for(3.0,1200,0.5)
               return
            
        curvature = self.calculate_curvature(left_lane,right_lane)
        
        if stop_flag_num > 5 and curvature < 1000:
            self.timed_steer_sequence(
                duration=3,
                time_points=[0, 0.3, 2],
                steer_values=[0.55, 0.9, 0.6],
                speed=1200
            )
            return
                
        self.lane_ctrl.ctrl_move(curvature, self.center_index)

        rospy.loginfo(f"weight_left : {left_lane} / weight_right : {right_lane}")
        rospy.loginfo(f"stop_flag_num:{stop_flag_num} / stop_flag : {stop_flag} / signal : {signal}")

