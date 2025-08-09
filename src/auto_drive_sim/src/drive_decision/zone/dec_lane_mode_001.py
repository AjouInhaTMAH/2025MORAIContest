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
from drive_decision.lane import dec_lane_amcl, dec_lane_curvature, dec_lane_distance
MAX_Y = 1
class DecLaneMode_001:
    def __init__(self,CtrlMotorServo, DecLaneCurvature,DecLaneDistance):
        self.init_mission4()
        self.init_processing(CtrlMotorServo, DecLaneCurvature,DecLaneDistance)
    
    def init_mission4(self):
        self.mi4_stop_flag = False
        self.mi4_in_flag = False
        self.mi4_out_flag = False
    def init_processing(self,CtrlMotorServo:ctrl_motor_servo.CtrlMotorServo, DecLaneCurvature:dec_lane_curvature.DecLaneCurvature,DecLaneDistance:dec_lane_distance.DecLaneDistance):
        self.CtrlMotorServo = CtrlMotorServo
        self.DecLaneCurvature = DecLaneCurvature
        self.DecLaneDistance = DecLaneDistance

    def set_lidar_info(self,left_obstacle,front_obstacle,right_obstacle):
        self.left_obstacle, self.front_obstacle, self.right_obstacle = left_obstacle,front_obstacle,right_obstacle

    def stop_time(self,time = 2):
        self.CtrlMotorServo.pub_move_motor_servo(0, 0.5)
        sleep(time)
        
    def handle_zone_mission4(self,stop_line):
        if self.mi4_out_flag:
            print(f"5")
            mode, left_lane, right_lane = self.DecLaneCurvature.pth01_ctrl_decision_left()
            self.DecLaneCurvature.pth01_ctrl_move(mode, left_lane, right_lane)
        elif self.mi4_in_flag:
            self.CtrlMotorServo.pub_move_motor_servo(2400,-0.7179)
            sleep(0.16)
            self.CtrlMotorServo.pub_move_motor_servo(2100,1)
            sleep(0.425)
            self.mi4_out_flag = True
            print(f"4")
            self.stop_time()
        elif self.mi4_stop_flag:
            print(f"3")
            if self.check_obstacle_rotary():
                return
            print(f"no obs")
            self.CtrlMotorServo.pub_move_motor_servo(2400,0.5)
            # sleep(0.275)
            sleep(0.55)
            self.CtrlMotorServo.pub_move_motor_servo(2400,1)
            sleep(0.4)
            self.mi4_in_flag = True
        elif stop_line != [] and stop_line[MAX_Y] > 240:
            print(f"2")
            self.mi4_stop_flag =True
            self.stop_time(2)
        else:
            print(f"1")
            self.DecLaneDistance.chose_center_left()
            self.DecLaneDistance.ctrl_moveByLine_right()
            # mode, left_lane, right_lane = self.ctrl_decision_left()
            # self.ctrl_move(mode, left_lane, right_lane)
        
    def check_obstacle_rotary(self):
        if self.left_obstacle or self.front_obstacle or self.right_obstacle:
            return True 
        return False