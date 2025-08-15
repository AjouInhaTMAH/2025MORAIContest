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
class DecLaneMode_003:
    def __init__(self,CtrlMotorServo, DecLaneCurvature):
        self.init_goal()
        self.init_processing(CtrlMotorServo,DecLaneCurvature)
    def init_goal(self):
        self.stop_right_flag = False
        self.steer_hold_until_ts = 0.0
        self.count_right_hardcoding_follow = 0
        pass
    def init_processing(self, CtrlMotorServo:ctrl_motor_servo.CtrlMotorServo,
                        DecLaneCurvature:dec_lane_curvature.DecLaneCurvature):
        self.CtrlMotorServo = CtrlMotorServo
        self.DecLaneCurvature = DecLaneCurvature
    def set_camera_info(self,stop_line):
        self.stop_line = stop_line

        

    def right_hardcoding_follow(self):
        now_ts = time()
        print(f"right_hardcoding_follow")
        # 2초 홀드 중이면 즉시 고정 출력
        if now_ts < self.steer_hold_until_ts:
            elapsed = self.steer_hold_until_ts - now_ts  # 남은 시간
            hold_duration = 2.5
            passed = hold_duration - elapsed             # 지난 시간

            # (경과 시간, 스티어 값) 리스트
            steer_pattern = [
                (0.0, 0.5),  # 시작~0.45초
                (0.3, 0.8),  # 시작~0.45초
                (1.3, 0.9),  # 시작~0.45초
                # (2.3, 0.5),  # 0.45~2초
                (2.5, 0.5),  # 0.45~2초
            ]

            steer_val = 0.55  # 기본값
            for t_point, s_val in steer_pattern:
                if passed >= t_point:
                    steer_val = s_val
                else:
                    break

            self.CtrlMotorServo.pub_move_motor_servo(1000, steer_val)
            rospy.loginfo(f"[CURV HOLD pattern] t={passed:.2f}s steer={steer_val:.2f}")
            return False
        
        self.steer_hold_until_ts = now_ts + 2.5
        self.CtrlMotorServo.pub_move_motor_servo(1000, 0.5)
        self.count_right_hardcoding_follow += 1
        return False
        
    def handle_zone_goal(self,stop_line):
        # 홀드가 아니면 의사결정
        if self.count_right_hardcoding_follow == 2:
            self.DecLaneCurvature.decision(1)
        elif self.stop_right_flag:
            self.right_hardcoding_follow()
        elif stop_line != [] and stop_line[MAX_Y] > 400:
            print(f"stop_line[MAX_Y] {stop_line[MAX_Y]}")
            self.stop_right_flag =True
        else:
            # print(f"goal_zone")
            self.DecLaneCurvature.decision(3)
