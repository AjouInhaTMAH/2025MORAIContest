#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import rospy
import numpy as np
from std_msgs.msg import Float64
from time import *
import Motor

class lane_ctrl:
    def __init__(self):
        self.pid = PIDController()
        self.motor = Motor.motor_ctrl()
        
        self.center_pixel = 320
        self.max_steer = 1 #19.5
        self.min_steer = 0 #-19.5
        self.steer_per_pixel = 2 / 640  # 수정 가능
        self.max_speed = 1200
        self.min_speed = 700

    def ctrl_move(self, curvature, center_index): # left_lane=None, right_lane=None
        pixel_error = center_index - self.center_pixel
        steer_error = pixel_error * self.steer_per_pixel
        #print(f"left_lane:{left_lane} / right_lane:{right_lane}")

        steer_gain = self.get_steer_gain(curvature)
        pid_output = self.pid.compute(steer_error)
        steer = steer_gain * pid_output + 0.5
                       
        steer = max(self.min_steer, min(self.max_steer, steer))
        base_speed = self.get_base_speed(curvature)
        deviation = abs(steer - 0.5)
        speed = max(self.min_speed, int(base_speed - deviation * (base_speed - self.min_speed)))

        self.motor.publish(speed, steer)

        rospy.loginfo(f"[PID] error: {steer_error:.4f}, output: {pid_output:.4f}")
        rospy.loginfo(f"[LCTRL] steer: {steer:.2f}, speed: {speed:.2f}")
        rospy.loginfo(f"[Curvature] value: {curvature:.2f}")
        rospy.loginfo(f"[steer_gain] value: {steer_gain:.2f}")

        # 부드러운 steer_gain 계산 함수
    def get_steer_gain(self, curvature):
        A = 40.0  # 최대 gain
        B = 0.002528 # 0.002528
        return max(1.2, A * np.exp(-B * curvature))

    def get_base_speed(self, curvature):
        min_speed = self.min_speed
        max_speed = self.max_speed

        # 곡률이 작을수록 (급커브) 속도 ↓ / 곡률이 클수록 (직선) 속도 ↑
        curvature = min(curvature, 10000)  # 과도한 곡률 제한

        # 정규화: 0 (급커브) → 1 (직선)
        norm = curvature / 10000.0
        norm = max(0.0, min(1.0, norm))  # 안정화

        # 보간된 속도 계산
        speed = min_speed + norm * (max_speed - min_speed)
        return int(speed)
            
class PIDController:
    def __init__(self, Kp=0.64, Ki=0.0, Kd=0.0527): # Kp = 0.64 Kd = 0.0527
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time()
        self.integral_limit = 100.0

    def compute(self, error):
        current_time = time()
        dt = current_time - self.last_time
        self.last_time = current_time

        if dt <= 0.0:
            dt = 1e-6

        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return output