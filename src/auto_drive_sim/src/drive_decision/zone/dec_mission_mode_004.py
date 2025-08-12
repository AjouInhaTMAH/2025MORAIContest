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
from drive_decision.ctrl import ctrl_motor_servo
from drive_decision.lane import dec_lane_curvature
import rospy
MAX_Y = 1
class DecLaneMode_004:
    def __init__(self,CtrlMotorServo, DecLaneCurvature):
        self.init_goal()
        self.init_processing(CtrlMotorServo,DecLaneCurvature)
    def init_goal(self):
        self.hold_until_ts = 0
    def init_processing(self, CtrlMotorServo:ctrl_motor_servo.CtrlMotorServo,
                        DecLaneCurvature:dec_lane_curvature.DecLaneCurvature):
        self.CtrlMotorServo = CtrlMotorServo
        self.DecLaneCurvature = DecLaneCurvature
    def set_camera_info(self,stop_line):
        self.stop_line = stop_line

        
    def handle_zone_goal_02(self,stop_line):
        now_ts = time()  # PID는 기존 time() 계속 사용
        # 홀드가 아니면 의사결정
        mode, left_lane, right_lane = self.DecLaneCurvature.pth01_ctrl_decision()
        stop_flag = self.stop_line
        
        # if mode == "traffic":
        #     steer, speed = self.traffic_control.traffic_action(stop_flag)  # 인자 필수
        #     self.publish(speed, steer)
        #     return
        
        # 3초 홀드 중이면 무조건 직진 유지
        if now_ts < self.hold_until_ts:
            self.CtrlMotorServo.pub_move_motor_servo(1000, 0.5)
            rospy.loginfo("[HOLD] go straight (%.2fs left)" % (self.hold_until_ts - now_ts))
            return
        
        # go_straight가 트리거된 '그 순간'에만 3.5초 홀드 시작
        if mode == "move_straight" and now_ts >= self.hold_until_ts:
            self.hold_until_ts = now_ts + 3.5
            self.CtrlMotorServo.pub_move_motor_servo(1000, 0.5)
            rospy.loginfo("[HOLD start] 3s straight")
            return
        
        
        # 그 외에는 정상 제어
        self.DecLaneCurvature.ctrl_move_goal(mode, left_lane, right_lane, stop_flag)
