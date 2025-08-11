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

import rospy
from time import *
from drive_decision.ctrl import ctrl_motor_servo
from drive_decision.lane import dec_lane_amcl, dec_lane_curvature, dec_lane_distance
MAX_Y = 1
class DecLaneMode_001:
    def __init__(self,CtrlMotorServo, DecLaneCurvature,DecLaneDistance,DecLaneAmcl):
        self.init_mission4()
        self.init_processing(CtrlMotorServo, DecLaneCurvature,DecLaneDistance,DecLaneAmcl)
    
    def init_mission4(self):
        self.mi4_stop_flag = False
        self.mi4_in_flag = False
        self.mi4_out_flag = False
        self.mi4_moving1_flag = False
    def init_processing(self,CtrlMotorServo:ctrl_motor_servo.CtrlMotorServo
                        , DecLaneCurvature:dec_lane_curvature.DecLaneCurvature
                        , DecLaneDistance:dec_lane_distance.DecLaneDistance
                        , DecLaneAmcl:dec_lane_amcl.DecLaneAmcl):
        self.CtrlMotorServo = CtrlMotorServo
        self.DecLaneCurvature = DecLaneCurvature
        self.DecLaneDistance = DecLaneDistance
        self.DecLaneAmcl = DecLaneAmcl

    def set_lidar_info(self,left_obstacle,front_obstacle,right_obstacle):
        self.left_obstacle, self.front_obstacle, self.right_obstacle = left_obstacle,front_obstacle,right_obstacle

    def stop_time(self,time = 2):
        self.CtrlMotorServo.pub_move_motor_servo(0, 0.5)
        sleep(time)
    
    def out_rotray(self):
        if self.mi4_moving1_flag:
            self.CtrlMotorServo.pub_move_motor_servo(2100,1)
            sleep(0.475)
            self.mi4_out_flag = True
            self.stop_time(5)
        else:
            moving = self.DecLaneAmcl.drvie_amcl2target_rotary(4.6,6.5)
            if moving:
                self.mi4_moving1_flag = True
                self.stop_time(10)
                

    def inter_rotary(self):
        start_time = rospy.Time.now().to_sec()
        last_action_time = start_time

        rate = rospy.Rate(80)  # 50Hz 루프, 콜백 계속 처리 가능

        while rospy.Time.now().to_sec() - start_time < 0.7 and not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            # print(f"now - last_action_time {now - last_action_time}")
            # 0번째 동작: 모터=2400, 서보=0.5
            if  now - last_action_time >= 0.3:
                self.CtrlMotorServo.pub_move_motor_servo(2400, 1)
            elif now - last_action_time >= 0.0:
                self.CtrlMotorServo.pub_move_motor_servo(2400, 0.5)
            rate.sleep()  # 여기서 ROS 콜백들이 처리됨
        
    def handle_zone_mission4(self,stop_line):
        if self.mi4_out_flag:
            print(f"5")
            mode, left_lane, right_lane = self.DecLaneCurvature.pth01_ctrl_decision_left()
            self.DecLaneCurvature.pth01_ctrl_move(mode, left_lane, right_lane)
        elif self.mi4_in_flag:
            # self.CtrlMotorServo.pub_move_motor_servo(2400,-0.7179)
            # sleep(0.16)
            # self.CtrlMotorServo.pub_move_motor_servo(2100,1)
            # sleep(0.425)
            # print(f"4")
            # self.mi4_out_flag = True
            self.out_rotray()
            # self.stop_time(0)
        elif self.mi4_stop_flag:
            print(f"3")
            if self.check_obstacle_rotary():
                return
            print(f"no obs")
            self.inter_rotary()
            print(f"inter rotary")
            self.mi4_in_flag = True
        elif stop_line != [] and stop_line[MAX_Y] > 240:
            print(f"2")
            self.mi4_stop_flag =True
            self.stop_time(0)
        else:
            print(f"1")
            self.DecLaneDistance.chose_center_left_white_lane()
            self.DecLaneDistance.ctrl_moveByLine()
            # mode, left_lane, right_lane = self.ctrl_decision_left()
            # self.ctrl_move(mode, left_lane, right_lane)
        
    def check_obstacle_rotary(self):
        if self.left_obstacle or self.front_obstacle or self.right_obstacle:
            return True 
        return False