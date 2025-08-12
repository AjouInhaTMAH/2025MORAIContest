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
import rospy
from drive_decision.ctrl import ctrl_motor_servo
from drive_decision.lane import dec_lane_curvature

MAX_Y = 1
class DecLaneMode_000:
    def __init__(self,CtrlMotorServo, DecLaneCurvature):
        self.init_mission2_3()
        self.init_processing(CtrlMotorServo, DecLaneCurvature)
        
    def init_processing(self,CtrlMotorServo:ctrl_motor_servo.CtrlMotorServo,
                        DecLaneCurvature:dec_lane_curvature.DecLaneCurvature):
        self.CtrlMotorServo = CtrlMotorServo
        self.DecLaneCurvature = DecLaneCurvature
    
    def set_front_near_obstacle(self, flag):
        self.front_near_obstacle = flag
    
    def init_mission2_3(self):
        self.front_near_obstacle       = False
        self.lane_mode     = "right"
        self.avoid_side       = None
        self.in_avoid_mode    = False
        self.dynamic_obs_flag = False
        self.obs_flag         = False
        self.count_stopsline = 0
        self.thick_plan = {
            1: {"type":"steer_fixed", "steer":0.5, "speed":800, "duration":1.5},
        }
    def set_lane_mode(self, lane):
        self.lane_mode = lane
    def set_dynamic_obs_flag(self,flag):
        self.dynamic_obs_flag = flag
        
    def handle_zone_mission2_3(self,stop_line):
        now = rospy.get_time()
        
        if self.dynamic_obs_flag:
            print(f"동적 장애물 발견")
            steer, speed = 0.5, 0
            self.front_near_obstacle = False
            self.obs_flag   = True
            self.waypoint_idx = -1
            self.CtrlMotorServo.pub_move_motor_servo(speed,steer)
            
        elif self.front_near_obstacle or self.in_avoid_mode:
            # print(f"lidar flag {self.front_near_obstacle}")R
            print(f"현재 차선은 {self.lane_mode}")
            if not self.in_avoid_mode:
                self.avoid_side    = "left" if self.lane_mode == "right" else "right"
                print(f"가야할 차선은 {self.avoid_side}")
                self.in_avoid_mode = True
                self.obs_flag      = True
                self.waypoint_idx  = 0
                self.last_time     = now
                self.sleep_duration= 1.0

            if now - self.last_time > self.sleep_duration:
                self.waypoint_idx += 1
                self.last_time = now

            if   self.waypoint_idx == 0:
                steer, speed = (0.1 if self.avoid_side == "left" else 0.9), 1400
                self.sleep_duration = 0.1

            elif self.waypoint_idx == 1:
                steer, speed = (0.1 if self.avoid_side == "left" else 0.9), 800
                self.sleep_duration = 0.55

            elif self.waypoint_idx == 2:
                steer, speed = (0.9 if self.avoid_side == "left" else 0.1), 1200
                # self.sleep_duration = 0.9
                self.sleep_duration = 0.55

            elif self.waypoint_idx == 3:
                steer, speed = 0.5, 600
                self.sleep_duration = 0.5

            elif self.waypoint_idx == 4:
                self.in_avoid_mode = False
                self.front_near_obstacle = False
                self.obs_flag   = False
                self.waypoint_idx = -1
                self.sleep_duration= 0.0
                self.avoid_side   = None
                steer, speed = 0.5, 800
            else:
                self.in_avoid_mode = False
                self.front_near_obstacle = False
                self.obs_flag   = False
                self.waypoint_idx = -1
                steer, speed = 0.5, 0
            self.CtrlMotorServo.pub_move_motor_servo(speed,steer)
            rospy.sleep(self.sleep_duration)
            
        elif stop_line != [] and stop_line[MAX_Y] > 440:
        # {"type":"steer_fixed", "steer":0.5, "speed":800, "duration":1.5},
            steer = float(self.thick_plan[1]["steer"])
            speed = float(self.thick_plan[1]["speed"])
            duration = float(self.thick_plan[1]["duration"])
            if self.count_stopsline != 2:
                self.CtrlMotorServo.pub_move_motor_servo(speed,steer)
                self.count_stopsline += 1
                rospy.sleep(duration)
            print(f"self.count_stopsline {self.count_stopsline}")
            print(f"self.count_stopsline {self.count_stopsline}")
            print(f"self.count_stopsline {self.count_stopsline}")
        else:
            mode, left_lane, right_lane = self.DecLaneCurvature.pth01_ctrl_decision()
            # self.DecLaneCurvature.pth01_ctrl_move(mode, left_lane, right_lane)
            self.DecLaneCurvature.pth01_ctrl_move_right(mode, left_lane, right_lane)
            

