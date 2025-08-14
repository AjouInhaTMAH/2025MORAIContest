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
import numpy as np
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
        self.start_time_mi4 = None  # 미션4 시작 시간 기록용
        self.is_out_rotary = False
        
    def init_processing(self,CtrlMotorServo:ctrl_motor_servo.CtrlMotorServo
                        , DecLaneCurvature:dec_lane_curvature.DecLaneCurvature):
        self.CtrlMotorServo = CtrlMotorServo
        self.DecLaneCurvature = DecLaneCurvature

    def set_lidar_info(self,left_obstacle,front_obstacle,right_obstacle,front_obstacle_length):
        self.left_obstacle, self.front_obstacle, self.right_obstacle, self.front_obstacle_length = left_obstacle,front_obstacle,right_obstacle,front_obstacle_length
    def set_rotary_obstacle_length(self, rotary_obstacle_length):
        self.rotary_obstacle_length = rotary_obstacle_length
    def set_is_out_rotary(self,is_out_rotary):
        self.is_out_rotary = is_out_rotary
    def stop_time(self,time = 2):
        self.CtrlMotorServo.pub_move_motor_servo(0, 0.5)
        rospy.sleep(time)
    
    def out_rotray_not_obs(self):
        start_time = rospy.Time.now().to_sec()
        turn_left_time = 0.6
        turn_right_time = 1.4
        total_time = turn_left_time + turn_right_time
        while rospy.Time.now().to_sec() - start_time < total_time and not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            speed = 800
            if  now - start_time >= turn_left_time: 
                self.CtrlMotorServo.pub_move_motor_servo(speed,1)
            elif now - start_time >= 0.0:
                self.CtrlMotorServo.pub_move_motor_servo(speed,0.1)
    
    def spin_rotary(self):
        # x_lenth = 1
        # y_lenth = 0.525
        x_target = 0.55
        y_target = 0.18
        speed_min = 0.0
        speed_max = 800.0
        
        # steer 범위
        steer_min = 0.0
        steer_max = 0.5  
              
        x_min = -0.45  # 최대 속도
        x_max = 0.1   # 속도 0
        
        # error_y 범위
        y_min = -0.33  # error_y 최소값
        y_max = 0.18   # error_y 최대값
        
        while not rospy.is_shutdown():
            speed = 800
            steer = 0.09
            if self.is_out_rotary:
                return
            if self.rotary_obstacle_length != []:
                error_x = x_target + self.rotary_obstacle_length[0]
                error_y = y_target + self.rotary_obstacle_length[1]
                # print(f"self.rotary_obstacle_length {self.rotary_obstacle_length}")
                # 속도: 목표까지 남은 x 거리에 비례, 0 ~ max_speed로 제한
                # 가까우면 속도 줄이고, 멀면 최대 속도
                speed = np.interp(error_x, [x_min, x_max], [speed_max, speed_min])
                speed = max(0.0, min(speed, speed_max))  # 안전하게 범위 제한
                # error_y 값을 0~0.5 범위로 선형 매핑
                steer = np.interp(error_y, [y_min, y_max], [steer_min, steer_max])
                steer = max(steer_min, min(steer, steer_max))  # 안전 범위 제한

                # print(f"error_x={error_x:.3f}, error_y={error_y:.3f}, speed={speed:.1f}, steer={steer:.3f}")
            self.CtrlMotorServo.pub_move_motor_servo(speed,steer)
            # print(f"speed {speed}")
    def go_out_of_rotary(self):
        start_time = rospy.Time.now().to_sec()
        turn_left_time = 0.25
        turn_right_time = 1.4
        total_time = turn_left_time + turn_right_time
        while rospy.Time.now().to_sec() - start_time < total_time and not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            speed = 1200
            # print(f"speed {speed}")
            if  now - start_time >= turn_right_time: 
                self.CtrlMotorServo.pub_move_motor_servo(speed,0)
            else :
                self.CtrlMotorServo.pub_move_motor_servo(speed,1)
            
    def out_rotray_obs(self):
        self.spin_rotary()
        self.go_out_of_rotary()

                
    def inter_rotary(self):
        start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(80)  # 50Hz 루프, 콜백 계속 처리 가능
        total_time = 0.59
        while rospy.Time.now().to_sec() - start_time < total_time and not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            if self.obs_now and now - start_time >= 0.25:
                self.CtrlMotorServo.pub_move_motor_servo(2400, 1)
                total_time = 0.64
            elif not self.obs_now and now - start_time >= 0.2:
                self.CtrlMotorServo.pub_move_motor_servo(2400, 1)
            elif now - start_time >= 0.0:
                self.CtrlMotorServo.pub_move_motor_servo(2400, 0.5)
            rate.sleep()  # 여기서 ROS 콜백들이 처리됨
        # 240 ~260
    def handle_zone_mission4(self,stop_line):
                # 첫 진입 시각 기록
        if self.start_time_mi4 is None:
            self.start_time_mi4 = rospy.Time.now().to_sec()
        elapsed = rospy.Time.now().to_sec() - self.start_time_mi4
        enable_stopline_check = elapsed >= 7.5  # 5초 지나야 stop_line 검사
        # enable_stopline_check = False  # 5초 지나야 stop_line 검사
        
        if self.mi4_out_flag:
            # print(f"5")
            self.stop_time(0)
            self.DecLaneCurvature.set_speed(700,1200)
            self.DecLaneCurvature.decision()
            return True
        elif self.mi4_in_flag:
            if self.rotary_obstacle_length == []:
                print(f"obs")
                self.out_rotray_not_obs()
            else:
                print(f"obs")
                self.out_rotray_obs()
            self.mi4_out_flag = True
            
        elif self.mi4_stop_flag:
            # print(f"3")
            if self.check_obstacle_rotary():
                self.obs_now = True
                return
            if self.obs_now:
                print(f"obs")
            else:
                print(f"no obs")
            self.inter_rotary()
            self.mi4_in_flag = True
            self.stop_time(0)
        elif stop_line != [] and stop_line[MAX_Y] > 360:
            # print(f"2")
            print(f"stop_line[MAX_Y] {stop_line[MAX_Y]}")
            self.mi4_stop_flag =True
            self.obs_now = False
            self.stop_time(0)
        elif enable_stopline_check:
            # print(f"1_timer")
            self.DecLaneCurvature.set_speed(400,400)
            self.DecLaneCurvature.decision()
        else:
            # print(f"1")
            self.DecLaneCurvature.decision(0)
        return False
        # print(f"??? {time() - start}")
    def check_obstacle_rotary(self):
        if self.left_obstacle or self.front_obstacle or self.right_obstacle:
            return True 
        return False