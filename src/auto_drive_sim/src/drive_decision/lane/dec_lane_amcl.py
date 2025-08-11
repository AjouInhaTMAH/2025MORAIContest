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

class DecLaneAmcl:
    def __init__(self,CtrlMotorServo):
        """_summary_
        현재 방법이 2가지로 나누었기에 함수에 이 이름들을 일단 넣도록 한다.
        방법 1, pt01 - 곡률 계산
        방법 2, pt02 - lane으로 일정 거리 계산 
        """
        print(f"DecLaneAmcl create")
        self.init_amcl()
        self.init_processing(CtrlMotorServo)
    def init_processing(self,CtrlMotorServo:ctrl_motor_servo.CtrlMotorServo):
        self.CtrlMotorServo = CtrlMotorServo
        
    def init_amcl(self):
        self.x = 0
        self.y = None
        self.yaw = 0
        self.cross_goal_x = None
        self.cross_goal_y = None
        self.speed = 400
    def set_currentPose_info(self,x,y,yaw):
        self.x ,self.y, self.yaw = x,y,yaw

    def angle_to_target(self, x, y, w, tx, ty):
        # 현재 위치에서 목표 지점까지의 각도 차이 계산
        print(f"angle_to_target {x}")
        print(f"angle_to_target {y}")
        print(f"angle_to_target {w}")
        print(f"angle_to_target {tx}")
        print(f"angle_to_target {ty}")
        dx = - tx + x
        dy = - ty + y
        
        w_rotated = w - 90  # 90도 회전
        w_rotated = (w_rotated + 360) % 360
        heading_rad = math.radians(w)  # w는 degree (0~360 기준)
        heading_rad = math.radians(w_rotated)  # w는 degree (0~360 기준)
        heading_vector = (math.cos(heading_rad), math.sin(heading_rad))

        target_norm = math.hypot(dx, dy)
        target_vector = (dx / target_norm, dy / target_norm)
        # 내적, 외적
        dot = heading_vector[0] * target_vector[0] + heading_vector[1] * target_vector[1]
        cross = heading_vector[0] * target_vector[1] - heading_vector[1] * target_vector[0]
        # 부호 있는 각도 (라디안 → 도)
        angle_rad = math.atan2(cross, dot)
        angle_deg = math.degrees(angle_rad)

        print(f"벡터 간 부호 있는 회전 각도: {angle_deg:.2f}°")
        return angle_deg

    def compute_drive_command(self,x, y, w, tx, ty, dis_tarket=0.5,speed= 400):
        # 거리 계산
        distance = math.hypot(tx - x, ty - y)
        self.speed = speed
        # 도달 조건 (선택적으로 함수 외부에서 사용 가능)
        reached = distance < dis_tarket

        # 조향 계산
        angle_diff_deg = self.angle_to_target(x, y, w, tx, ty)
        print(f"angle_diff_deg {angle_diff_deg}")
        # [-90, 90]로 제한
        angle_diff_deg = max(min(angle_diff_deg, 30), -30)

        # -90~90 → -19.5~19.5로 선형 맵핑
        steer = (angle_diff_deg / 30) * 19.5
        
        speed = self.speed  # 고정 속도
        return speed, -steer, reached
    
    def drvie_amcl(self):
        # 현재 위치
        x = self.x
        y = self.y
        w = self.yaw
        if self.cross_goal_x is None:
            self.cross_goal_x = self.x - 3.5
            self.cross_goal_y = self.y
        tx, ty = self.cross_goal_x, self.cross_goal_y
        print(f"tx")
        speed, steer, reached = self.compute_drive_command(x, y, w, tx, ty)
        # print(f"reached {reached}")
        if reached :
            print(f"reached {reached}")
            return True
        steer = ((steer / 19.5 + 1)) /2
        print(f"speed, steer {speed} {steer}")
        self.CtrlMotorServo.pub_move_motor_servo(speed,steer)
        return False

