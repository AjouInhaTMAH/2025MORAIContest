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
from time import *
import real_lane_detection


class lane_ctrl:
    def __init__(self):
        self.motor_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
        self.pid = PIDController()
        self.center_pixel = 320
        self.max_steer = 1 #19.5
        self.min_steer = 0 #-19.5
        self.total_steer = self.max_steer - self.min_steer
        self.steer_per_pixel = 2 / 640  # 수정 가능
        self.max_speed = 1200
        self.min_speed = 700
        self.center_index = 0
        #self.stop_flag = self.perception_node.cross_flag_check()
        self.LANE_WIDTH_PIXELS = 260 # 적절히 조정
        #
        self.hold_until_ts = 0.0
        self.steer_hold_until_ts = 0.0 # (신규) 2초 steer=0.8 오버라이드용
        
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
        if len(x_vals) < 4:
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

    def ctrl_decision(self, dataset):
        stop_flag, yellow_left, yellow_right, white_left, white_right = dataset

        # 한 쪽 차선만 들어오는 경우를 위한 통합 처리
        left_white_lane = white_left
        left_yellow_lane = yellow_left
        right_lane = white_right
        #print(f"weight_left : {left_white_lane} / weight_right : {right_lane}")
        
        if left_yellow_lane and stop_flag:
            return self.move_forward(left_yellow_lane)
        
        elif stop_flag == True:
            return self.traffic()

        elif left_white_lane and right_lane:
            return self.which_lane(left_white_lane, right_lane)
        
        elif left_yellow_lane and right_lane:
            return self.which_lane(left_yellow_lane, right_lane)
        
        elif left_white_lane and not right_lane:
            return self.follow_left_lane(left_white_lane)

        elif left_yellow_lane and not right_lane:
            return self.follow_left_lane(left_yellow_lane)

        elif right_lane and not left_white_lane:
            return self.follow_right_lane(right_lane)

        else:
            return self.forward()
        
    def traffic(self):
        return "traffic", [], []
    
    def which_lane(self,left_lane, right_lane):
        left_index = (left_lane[0][0] + left_lane[-1][0]) // 2
        right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
        weight_left = len(left_lane)
        weight_right = len(right_lane)
        
        if weight_left > weight_right:
            left_index = (left_lane[0][0] + left_lane[-1][0]) // 2
            self.center_index = left_index + self.LANE_WIDTH_PIXELS // 2  # 가상의 중심선
            return "first_lane", left_lane, []
        
        else:
            right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
            self.center_index = right_index - self.LANE_WIDTH_PIXELS // 2  # 가상의 중심선
            return "second_lane", [], right_lane

    def follow_left_lane(self, left_lane):
        left_index = (left_lane[0][0] + left_lane[-1][0]) // 2
        self.center_index = left_index + self.LANE_WIDTH_PIXELS // 2  # 가상의 중심선
        return "left_guided", left_lane, []

    def follow_right_lane(self, right_lane):
        right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
        self.center_index = right_index - self.LANE_WIDTH_PIXELS // 2  # 가상의 중심선
        return "right_guided", [], right_lane
        
    def forward(self):
        self.center_index = self.center_pixel  # 정중앙
        return "go_straight", [], []

    def move_forward(self,left_lane):
        self.center_index = self.center_pixel  # 정중앙
        print("movemove")
        return "move_straight", left_lane, []

    def ctrl_move(self, mode, left_lane=None, right_lane=None,stop_flag = False):
        now_ts = time()

        # 2초 홀드 중이면 즉시 고정 출력
        if now_ts < self.steer_hold_until_ts:
            elapsed = self.steer_hold_until_ts - now_ts  # 남은 시간
            hold_duration = 2.0
            passed = hold_duration - elapsed             # 지난 시간

            # (경과 시간, 스티어 값) 리스트
            steer_pattern = [
                (0.0, 0.54),  # 시작~0.45초
                (0.4, 0.9),  # 0.45~2초
                (2, 0.75)   # 2~3초
            ]

            steer_val = 0.63  # 기본값
            for t_point, s_val in steer_pattern:
                if passed >= t_point:
                    steer_val = s_val
                else:
                    break

            self.publish(1000, steer_val)
            rospy.loginfo(f"[CURV HOLD pattern] t={passed:.2f}s steer={steer_val:.2f}")
            return
        
        pixel_error = self.center_index - self.center_pixel
        steer_error = pixel_error * self.steer_per_pixel
        print(f"left_lane:{left_lane} / right_lane:{right_lane} / stop_line:{stop_flag}")

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

            # stop_flag True이고 curvature 조건을 만족하면 2.5초 홀드 시작
        if stop_flag: # and curvature <= 1000
            self.steer_hold_until_ts = now_ts + 3
            # 홀드 시작 프레임에서는 즉시 첫 패턴 값 적용
            self.publish(1000, 0.55)
            rospy.loginfo("[CURV HOLD start] 3s steer pattern")
            return
        
        steer_gain = self.get_steer_gain(curvature)
        pid_output = self.pid.compute(steer_error)
        steer = steer_gain * pid_output + 0.5
                       
        steer = max(self.min_steer, min(self.max_steer, steer))
        base_speed = self.get_base_speed(curvature)
        deviation = abs(steer - 0.5)
        speed = max(self.min_speed, int(base_speed - deviation * (base_speed - self.min_speed)))

        self.publish(speed, steer)

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

    
    def action(self, dataset):
        now_ts = time()  # PID는 기존 time() 계속 사용
        # 홀드가 아니면 의사결정
        mode, left_lane, right_lane = self.ctrl_decision(dataset)
        stop_flag = dataset[0]
        
        # if mode == "traffic":
        #     steer, speed = self.traffic_control.traffic_action(stop_flag)  # 인자 필수
        #     self.publish(speed, steer)
        #     return
        
        # 3초 홀드 중이면 무조건 직진 유지
        if now_ts < self.hold_until_ts:
            self.publish(1000, 0.5)
            rospy.loginfo("[HOLD] go straight (%.2fs left)" % (self.hold_until_ts - now_ts))
            return
        
        # go_straight가 트리거된 '그 순간'에만 3.5초 홀드 시작
        if mode == "move_straight" and now_ts >= self.hold_until_ts:
            self.hold_until_ts = now_ts + 3.5
            self.publish(1000, 0.5)
            rospy.loginfo("[HOLD start] 3s straight")
            return
        
        
        # 그 외에는 정상 제어
        self.ctrl_move(mode, left_lane, right_lane, stop_flag)


            
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
