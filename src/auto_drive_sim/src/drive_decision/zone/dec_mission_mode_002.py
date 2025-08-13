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
class DecLaneMode_002:
    def __init__(self,CtrlMotorServo, DecLaneCurvature):
        self.init_mission5()
        self.init_processing(CtrlMotorServo, DecLaneCurvature)
        
    def init_processing(self, CtrlMotorServo:ctrl_motor_servo.CtrlMotorServo,
                        DecLaneCurvature:dec_lane_curvature.DecLaneCurvature):
        self.CtrlMotorServo = CtrlMotorServo
        self.DecLaneCurvature = DecLaneCurvature
    def init_mission5(self):
        self.stop_mission5_flag = False
        self.pass_mission5_flag = False
        self.is_to_go_traffic = False
    def set_is_to_go_traffic(self,flag):
        self.is_to_go_traffic = flag
    def stop_time(self,time = 2):
        self.CtrlMotorServo.pub_move_motor_servo(0, 0.5)
        sleep(time)
    def handle_zone_mission5(self,stop_line):
        if self.pass_mission5_flag:
            mode, left_lane, right_lane = self.DecLaneCurvature.pth01_ctrl_decision()
            self.DecLaneCurvature.pth01_ctrl_move(mode, left_lane, right_lane)
            # return True
        elif self.stop_mission5_flag and self.is_to_go_traffic:
            print(f"movemove")
            steer = 0.5
            speed = 800
            self.CtrlMotorServo.pub_move_motor_servo(speed,steer)
            rospy.sleep(0.5)
            steer = 0.3
            speed = 800
            self.CtrlMotorServo.pub_move_motor_servo(speed,steer)
            rospy.sleep(1.5)
            steer = 0.225
            speed = 800
            self.CtrlMotorServo.pub_move_motor_servo(speed,steer)
            rospy.sleep(2.5)
            self.pass_mission5_flag = True
            # self.stop_time(5)
        elif self.stop_mission5_flag:
            # print(f"stopstop")
            self.CtrlMotorServo.pub_move_motor_servo(0,0.5)
        elif stop_line != [] and stop_line[MAX_Y] > 100:
            print(f"stop_line[MAX_Y] {stop_line[MAX_Y]}")
            print(f"stop_line[MAX_Y] {stop_line[MAX_Y]}")
            print(f"stop_line[MAX_Y] {stop_line[MAX_Y]}")
            print(f"stop_line[MAX_Y] {stop_line[MAX_Y]}")
            print(f"stop_line[MAX_Y] {stop_line[MAX_Y]}")
            print(f"stop_line[MAX_Y] {stop_line[MAX_Y]}")
            print(f"stop_line[MAX_Y] {stop_line[MAX_Y]}")
            # self.stop_mission5_flag =True
            self.stop_time(5)
        else:
            print(f"out")
            mode, left_lane, right_lane = self.DecLaneCurvature.pth01_ctrl_decision()
            self.DecLaneCurvature.pth01_ctrl_move(mode, left_lane, right_lane)
        return False