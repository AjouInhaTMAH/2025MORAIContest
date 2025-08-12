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
from drive_decision.lane import dec_lane_curvature
MAX_Y = 1
class DecLaneMode_001:
    def __init__(self,CtrlMotorServo, DecLaneCurvature):
        self.init_mission4()
        self.init_processing(CtrlMotorServo, DecLaneCurvature)
    
    def init_mission4(self):
        self.mi4_stop_flag = False
        self.mi4_in_flag = False
        self.mi4_out_flag = False
        self.front_obstacle_length =None
        self.count_rotary = 0
    def init_processing(self,CtrlMotorServo:ctrl_motor_servo.CtrlMotorServo
                        , DecLaneCurvature:dec_lane_curvature.DecLaneCurvature):
        self.CtrlMotorServo = CtrlMotorServo
        self.DecLaneCurvature = DecLaneCurvature

    def set_lidar_info(self,left_obstacle,front_obstacle,right_obstacle,front_obstacle_length):
        self.left_obstacle, self.front_obstacle, self.right_obstacle, self.front_obstacle_length = left_obstacle,front_obstacle,right_obstacle,front_obstacle_length

    def stop_time(self,time = 2):
        self.CtrlMotorServo.pub_move_motor_servo(0, 0.5)
        rospy.sleep(time)
    
    def out_rotray(self):
        start_time = rospy.Time.now().to_sec()
        print("self.stop_time(10)")
        total_time = 1.0
        turn_time = 0.355
        while rospy.Time.now().to_sec() - start_time < total_time and not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            speed = 1200
            if self.front_obstacle_length is not None:
                total_time += 0.001
                if now - start_time >= turn_time:
                    turn_time += 0.001
                # turn_time += 0.03
                speed = 0
            print(f"speed {speed}")
            if  now - start_time >= turn_time: 
                self.CtrlMotorServo.pub_move_motor_servo(speed,1)
            elif now - start_time >= 0.0:
                self.CtrlMotorServo.pub_move_motor_servo(speed,0.1)
            rospy.sleep(0.01)  # 여기서 ROS 콜백들이 처리됨

    def inter_rotary(self):
        start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(80)  # 50Hz 루프, 콜백 계속 처리 가능
        total_time = 0.59
        while rospy.Time.now().to_sec() - start_time < total_time and not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            if not self.not_obs_now and now - start_time >= 0.25:
                print(f"stop rotary")
                self.CtrlMotorServo.pub_move_motor_servo(2400, 1)
                total_time = 0.64
            elif self.not_obs_now and now - start_time >= 0.1:
                print(f"nono stop rotary")
                self.CtrlMotorServo.pub_move_motor_servo(2400, 1)
            elif now - start_time >= 0.0:
                self.CtrlMotorServo.pub_move_motor_servo(2400, 0.5)
            rate.sleep()  # 여기서 ROS 콜백들이 처리됨
        # 240 ~260
    def handle_zone_mission4(self,stop_line):
        # start = time()
        # self.mi4_in_flag = True
        if self.mi4_out_flag:
            print(f"5")
            mode, left_lane, right_lane = self.DecLaneCurvature.pth01_ctrl_decision()
            self.DecLaneCurvature.pth01_ctrl_move(mode, left_lane, right_lane)
        elif self.mi4_in_flag:
            # self.stop_time(10)
            self.out_rotray()
            self.mi4_out_flag = True
        elif self.mi4_stop_flag:
            print(f"3")
            if self.check_obstacle_rotary():
                self.not_obs_now = False
                return
            print(f"no obs")
            self.inter_rotary()
            self.mi4_in_flag = True
        elif stop_line != [] and stop_line[MAX_Y] > 240:
            print(f"2")
            print(f"stop_line[MAX_Y] {stop_line[MAX_Y]}")
            self.mi4_stop_flag =True
            self.not_obs_now = True
            self.stop_time(0)
        else:
            print(f"1")
            mode, left_lane, right_lane = self.DecLaneCurvature.ctrl_decision_mission1()
            self.DecLaneCurvature.ctrl_move_mission1(mode, left_lane, right_lane)
        # print(f"??? {time() - start}")
    def check_obstacle_rotary(self):
        if self.left_obstacle or self.front_obstacle or self.right_obstacle:
            return True 
        return False