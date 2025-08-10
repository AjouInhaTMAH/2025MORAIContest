#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))  # dec_mission_all.py가 있는 폴더
# src/drive_decision 까지 상대 경로로 올라갔다가 내려가기 (예: 현재 파일 위치 기준)
drive_decision_path = os.path.abspath(os.path.join(current_dir, '..', '..', 'src', 'drive_decision'))

if drive_decision_path not in sys.path:
    sys.path.insert(0, drive_decision_path)
    
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from time import *
import math
from drive_decision.ctrl import ctrl_motor_servo

class DecLaneDistance:
    def __init__(self,CtrlMotorServo):
        """_summary_
        현재 방법이 2가지로 나누었기에 함수에 이 이름들을 일단 넣도록 한다.
        방법 1, pt01 - 곡률 계산
        방법 2, pt02 - lane으로 일정 거리 계산 
        """
        print(f"DecLaneDistance create")
        self.init_both_pth()
        self.init_pth02()
        self.init_processing(CtrlMotorServo)
        
    def init_processing(self,CtrlMotorServo:ctrl_motor_servo.CtrlMotorServo):
        self.CtrlMotorServo = CtrlMotorServo
        
    def init_both_pth(self):
        self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane = [],None,None,None,None
        self.center_index = 0
        self.center_pixel = 320
        self.steer_per_pixel = 2 / 640  # 수정 가능
        self.max_steer = 1 #19.5
        self.min_steer = 0 #-19.5
        
    def init_pth02(self):
        self.right_lane_delta = 134
        self.left_lane_delta = 163 + 20
        self.total_steer = self.max_steer - self.min_steer

    
    def set_camera_info(self,stop_line,yellow_left_lane,yellow_right_lane,white_left_lane,white_right_lane):
        self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane = stop_line,yellow_left_lane,yellow_right_lane,white_left_lane,white_right_lane


        
    def normalize_angle(self, angle):
        # [-pi, pi] 범위로 정규화
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def chose_center_right(self):
        stop_line, yellow_left, yellow_right, white_left, white_right = self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane
        left_lane = yellow_left
        right_lane = white_right
        self.right_lane_delta = 134
        self.left_lane_delta = 163 + 20
        if white_right:
            self.center_index = right_lane[0][0] - self.right_lane_delta
        elif left_lane:
            self.center_index = left_lane[0][0] - self.right_lane_delta
        else:
            self.center_index = 320
        print(f"self.center_index {self.center_index}")
        
    def ctrl_moveByLine_right(self):
        # self.center_pixel = 320
        # self.steer_per_pixel = 2 / 640  # 수정 가능
        # steer = (steer + 1) / 2.0
        # print(f"[INFO] steer: {steer:.2f} deg")
        pixel_error = self.center_index - self.center_pixel  # +면 우측, -면 좌측
        print(f"[INFO] pixel_error: {pixel_error:.2f} deg")
        steer = pixel_error * self.steer_per_pixel * self.total_steer * 2 + 0.5
        print(f"[INFO] pixel_error: {steer:.2f} deg")
        # steer = self.pid.compute(pixel_error)
        # 클리핑 (조향각 범위 제한)
        steer = max(min(steer, self.max_steer), self.min_steer)
        # steer_ratio = abs(steer) / self.max_steer  # 0 ~ 1
        # speed = self.max_speed * (1 - steer_ratio)  # 회전 클수록 속도 감소
        # speed = max(speed, self.min_speed)
        speed = 400
        # 5. 결과 저장 혹은 publish
        self.CtrlMotorServo.pub_move_motor_servo(speed,steer)
        print(f"[INFO] steer: {steer:.2f} deg, speed: {speed:.2f} km/h")
        
    def chose_center_left(self):
        stop_line, yellow_left, yellow_right, white_left, white_right = self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane
        left_lane = white_left
        right_lane = white_right
        self.right_lane_delta = 134
        self.left_lane_delta = 163 - 10
        if left_lane:
            self.center_index = (left_lane[0][0] + left_lane[-1][0]) // 2 + self.left_lane_delta
        elif right_lane:
            self.center_index = right_lane[0][0] + self.left_lane_delta
        else:
            self.center_index = 320
        print(f"self.center_index {self.center_index}")

