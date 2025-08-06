#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2 
import numpy as np
from std_msgs.msg import Float64
import time
import real_lane_detection

class lane_ctrl:
    def __init__(self):
        self.motor_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
        self.pid = PIDController()
        self.predictor = PredictiveController()
        self.center_pixel = 320
        self.max_steer = 1 #19.5
        self.min_steer = 0 #-19.5
        self.total_steer = self.max_steer - self.min_steer
        self.steer_per_pixel = 2 / 640  # 수정 가능
        self.max_speed = 1200
        self.min_speed = 700
        self.stop_flag = False
        self.center_index = 0

    def publish(self, speed, steer):
        speed_msg = Float64()
        steer_msg = Float64()
        speed_msg.data = speed
        steer_msg.data = steer
        #speed_msg.data = speed * 300
        #steer_msg.data = ((steer / 19.5 + 1)) / 2
        self.motor_pub.publish(speed_msg)
        self.servo_pub.publish(steer_msg)

    def calculate_curvature(self, x_vals, y_vals):
        # 최소 데이터 개수 확인 (2개 미만이면 근사 자체가 불가능함)
        if len(x_vals) < 5:
            return 1e4  # 데이터 부족 시 직선으로 간주

        try:
            # 2차 다항 근사 (y에 대한 x 곡선으로)
            fit = np.polyfit(y_vals, x_vals, 2)
            A, B = fit[0], fit[1]

            y_eval = np.max(y_vals)  # 일반적으로 이미지 하단 기준

            denominator = abs(2*A)
            if denominator < 1e-4:
                denominator = 1e-4  # 분모 폭발 방지

            curvature = ((1 + (2*A*y_eval + B)**2)**1.5) / denominator

            # 비정상적으로 큰 곡률 제한 (클수록 직선으로 처리)
            if curvature > 10000:
                curvature = 10000

            return curvature

        except Exception as e:
            rospy.logwarn(f"[Curvature Calculation Error] {e}")
            return 1e4  # 계산 실패 시 직선으로 간주

    def ctrl_decision(self,dataset):
        stop_line, yellow_left, yellow_right, white_left, white_right = dataset

        # 한 쪽 차선만 들어오는 경우를 위한 통합 처리
        left_white_lane = white_left
        left_yellow_lane = yellow_left
        right_lane = white_right
        #print(f"weight_left : {left_white_lane} / weight_right : {right_lane}")
            
        LANE_WIDTH_PIXELS = 220  # 적절히 조정

        if stop_line != []:
            cross_threshold = 35
            min_y, max_y = stop_line
            cross_diff = (max_y - min_y)
            if cross_threshold < cross_diff:
                self.stop_flag = True
            return "stop", [], []
        
        elif left_white_lane and right_lane:
            left_index = (left_white_lane[0][0] + left_white_lane[-1][0]) // 2
            right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
            weight_left = len(left_white_lane)
            weight_right = len(right_lane)
            #print(f"weight_left : {weight_left} / weight_right : {weight_right}")
            #total_weight = weight_left + weight_right
            #self.center_index = int((left_index * weight_left + right_index * weight_right) / total_weight)
            #self.center_index = int((left_index + right_index)//2)
            
            right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
            self.center_index = right_index - LANE_WIDTH_PIXELS // 2  # 가상의 중심선
            
            return "lane_follow", left_white_lane, right_lane

        elif left_white_lane and not right_lane:
            left_index = (left_white_lane[0][0] + left_white_lane[-1][0]) // 2
            self.center_index = left_index + LANE_WIDTH_PIXELS // 2  # 가상의 중심선
            return "left_guided", left_white_lane, []

        elif left_yellow_lane and right_lane:
            left_index = (left_yellow_lane[0][0] + left_yellow_lane[-1][0]) // 2
            self.center_index = left_index + LANE_WIDTH_PIXELS // 2  # 가상의 중심선
            return "yellow_guided", left_yellow_lane, []

        elif right_lane and not left_white_lane:
            right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
            self.center_index = right_index - LANE_WIDTH_PIXELS // 2  # 가상의 중심선
            return "right_guided", [], right_lane

        else:
            self.center_index = self.center_pixel  # 정중앙
            return "go_straight", [], []
    
    def ctrl_move(self, mode, left_lane=None, right_lane=None):
        pixel_error = self.center_index - self.center_pixel
        steer_error = pixel_error*self.steer_per_pixel
        
        left_detected = bool(left_lane)
        right_detected = bool(right_lane)
        
        x_vals, y_vals = [], []
        if left_lane:
            for pt in left_lane:
                x_vals.append(pt[0])
                y_vals.append(pt[1])
        elif right_lane:
            for pt in right_lane:
                x_vals.append(pt[0])
                y_vals.append(pt[1])

        curvature = self.calculate_curvature(x_vals, y_vals)
        steer_gain = self.get_steer_gain(curvature)
        base_speed = self.get_base_speed(curvature)

        pid_output = self.pid.compute(steer_error)
        steer = steer_gain*pid_output + 0.5
        steer = max(self.min_steer, min(self.max_steer, steer))  # 스티어링 제한
        
        # # 4. 예측 조향 업데이트 (항상 수행)
        # predicted_steer = self.predictor.update(
        #     steer=steer,
        #     left_detected=left_detected,
        #     right_detected=right_detected
        # )

        # # 5. 한쪽이라도 인식 안 됐을 때 → 예측 조향값으로 대체
        # if not left_detected or not right_detected:
        #     steer = predicted_steer
        #     #rospy.loginfo(f"[Predictive] Using predicted steer: {steer:.3f}")    

        deviation = abs(steer-0.5)
        speed = max(self.min_speed, int(base_speed - deviation * (base_speed - self.min_speed)))
        
        #self.last_steer = steer

        self.publish(speed, steer)
        
        rospy.loginfo(f"[PID] error: {steer_error:.4f}, output: {pid_output:.4f}")
        rospy.loginfo(f"[LCTRL] steer: {steer:.2f}, speed: {speed:.2f}")
        rospy.loginfo(f"[Curvature] value: {curvature:.2f}")

    def action(self,dataset):
        mode, left_lane, right_lane = self.ctrl_decision(dataset)
        self.ctrl_move(mode, left_lane, right_lane)

        # 부드러운 steer_gain 계산 함수
    def get_steer_gain(self, curvature):
        A = 500.0  # 최대 gain
        B = 0.00227
        return max(1.0, A * np.exp(-B * curvature))

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
    def __init__(self, Kp=0.4, Ki=0.0, Kd=0.0522): # Kp = 1.0 Kd = 0.01
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.integral_limit = 100.0

    def compute(self, error):
        current_time = time.time()
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

class PredictiveController:
    def __init__(self):
        self.predicted_steer = 0.5
        self.last_steer = 0.5
        self.last_detected_time = time.time()
        self.max_no_detection_duration = 1.5
        self.decay_slow = 0.02
        self.decay_fast = 0.04

    def update(self, steer, left_detected, right_detected):
        now = time.time()
        both_detected = left_detected and right_detected

        if both_detected:
            self.last_detected_time = now
            self.last_steer = steer
            self.predicted_steer = steer
        else:
            time_elapsed = now - self.last_detected_time

            if time_elapsed < self.max_no_detection_duration:
                self.predicted_steer = self.last_steer
            else:
                delta = 0.5 - self.predicted_steer
                if abs(delta) < 0.001:
                    self.predicted_steer = 0.5
                else:
                    decay_rate = self.decay_fast if (self.predicted_steer - 0.5) * delta < 0 else self.decay_slow
                    self.predicted_steer += decay_rate * delta
                    self.predicted_steer = max(0.0, min(1.0, self.predicted_steer))

        return self.predicted_steer
