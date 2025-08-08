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
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from PIDController import PIDController
from tf.transformations import euler_from_quaternion
from utils import check_timer
import subprocess
import json
import math
import numpy as np
from morai_msgs.msg import GetTrafficLightStatus

class lane_detect:
    def __init__(self):
        """_summary_
        현재 방법이 2가지로 나누었기에 함수에 이 이름들을 일단 넣도록 한다.
        방법 1, pt01 - 곡률 계산
        방법 2, pt02 - lane으로 일정 거리 계산 
        """
        print(f"lane_detect create")
        self.init_pubSub()
        self.init_both_pth()
        self.init_pth01()
        self.init_pth02()
        
    def init_pubSub(self):
        # self.motor_pub = rospy.Publisher('/commands/motor/ctrl', Float64, queue_size=1)
        # self.servo_pub = rospy.Publisher('/commands/servo/ctrl', Float64, queue_size=1)
        self.motor_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
        self.motor_cmd_msg_pub = Float64()
        self.servo_cmd_msg_pub = Float64()
    def init_both_pth(self):
        self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane = [],None,None,None,None
        self.center_index = 0
        self.center_pixel = 320
        self.steer_per_pixel = 2 / 640  # 수정 가능
        self.max_steer = 1 #19.5
        self.min_steer = 0 #-19.5
    def init_pth01(self):
        self.stop_flag_num = 0
        self.max_speed = 1200
        self.min_speed = 700
        self.pid = PIDController()
        
        self.sequence_active = False
        self.sequence_start_time = 0
    def init_pth02(self):
        self.right_lane_delta = 134
        self.left_lane_delta = 163 + 20
        self.total_steer = self.max_steer - self.min_steer
        
        self.x = 0
        self.y = None
        self.yaw = 0
        self.cross_goal_x = None
        self.cross_goal_y = None
        
        self.left_obstacle = False
        self.front_obstacle = False
        self.right_obstacle = False
        
    def set_currentPose_info(self,x,y,yaw):
        self.x ,self.y, self.yaw = x,y,yaw
    def set_lidar_info(self,left_obstacle,front_obstacle,right_obstacle):
        self.left_obstacle, self.front_obstacle, self.right_obstacle = left_obstacle,front_obstacle,right_obstacle
    def set_camera_info(self,stop_line,yellow_left_lane,yellow_right_lane,white_left_lane,white_right_lane):
        self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane = stop_line,yellow_left_lane,yellow_right_lane,white_left_lane,white_right_lane

    def publish_move(self, speed, steer):
        speed_msg = Float64()
        steer_msg = Float64()
        speed_msg.data = speed
        steer_msg.data = steer
        self.motor_pub.publish(speed_msg)
        self.servo_pub.publish(steer_msg)

    def pth01_ctrl_decision(self):
        stop_line, yellow_left, yellow_right, white_left, white_right = self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane

        # 한 쪽 차선만 들어오는 경우를 위한 통합 처리
        left_white_lane = white_left
        left_yellow_lane = yellow_left
        right_lane = white_right
        #print(f"weight_left : {left_white_lane} / weight_right : {right_lane}")
            
        LANE_WIDTH_PIXELS = 260 # 적절히 조정
        # LANE_WIDTH_PIXELS = 270 # 적절히 조정

        if stop_line != []:
            cross_threshold = 35
            min_y, max_y = stop_line
            cross_diff = (max_y - min_y)
            if cross_threshold < cross_diff:
                self.stop_flag_num += 1
            return self.stop_flag_num, [], []
        
        elif left_white_lane and right_lane:
            left_index = (left_white_lane[0][0] + left_white_lane[-1][0]) // 2
            right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
            weight_left = len(left_white_lane)
            weight_right = len(right_lane)
            #print(f"weight_left : {weight_left} / weight_right : {weight_right}")
            #total_weight = weight_left + weight_right
            #self.center_index = int((left_index * weight_left + right_index * weight_right) / total_weight)
            #self.center_index = int((left_index + right_index)//2)
            if weight_left > weight_right:
                left_index = (left_white_lane[0][0] + left_white_lane[-1][0]) // 2
                self.center_index = left_index + LANE_WIDTH_PIXELS // 2  # 가상의 중심선
                return "first_lane", left_white_lane, []
            
            else:
                right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
                self.center_index = right_index - LANE_WIDTH_PIXELS // 2  # 가상의 중심선
                return "second_lane", [], right_lane

        elif left_white_lane and not right_lane:
            left_index = (left_white_lane[0][0] + left_white_lane[-1][0]) // 2
            self.center_index = left_index + LANE_WIDTH_PIXELS // 2  # 가상의 중심선
            return "left_guided", left_white_lane, []

        elif left_yellow_lane and right_lane:
            left_index = (left_yellow_lane[0][0] + left_yellow_lane[-1][0]) // 2
            self.center_index = left_index + LANE_WIDTH_PIXELS // 2  # 가상의 중심선
            return "first_lane", left_yellow_lane, []

        elif right_lane and not left_white_lane:
            right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
            self.center_index = right_index - LANE_WIDTH_PIXELS // 2  # 가상의 중심선
            return "right_guided", [], right_lane

        else:
            self.center_index = self.center_pixel  # 정중앙
            return "go_straight", [], []
    def pth01_ctrl_decision_left(self):
        stop_line, yellow_left, yellow_right, white_left, white_right = self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane

        # 한 쪽 차선만 들어오는 경우를 위한 통합 처리
        left_white_lane = white_left
        left_yellow_lane = yellow_left
        right_lane = white_right
        #print(f"weight_left : {left_white_lane} / weight_right : {right_lane}")
            
        LANE_WIDTH_PIXELS = 260 # 적절히 조정

        if stop_line != []:
            cross_threshold = 35
            min_y, max_y = stop_line
            cross_diff = (max_y - min_y)
            if cross_threshold < cross_diff:
                self.stop_flag_num += 1
            return self.stop_flag_num, [], []
        
        elif left_white_lane and right_lane:
            left_index = (left_white_lane[0][0] + left_white_lane[-1][0]) // 2
            right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
            left_index = (left_white_lane[0][0] + left_white_lane[-1][0]) // 2
            self.center_index = left_index + LANE_WIDTH_PIXELS // 2  # 가상의 중심선
            return "left_first_lane", left_white_lane, []

        elif left_white_lane and not right_lane:
            left_index = (left_white_lane[0][0] + left_white_lane[-1][0]) // 2
            self.center_index = left_index + LANE_WIDTH_PIXELS // 2  # 가상의 중심선
            return "left_guided", left_white_lane, []

        elif left_yellow_lane and right_lane:
            left_index = (left_yellow_lane[0][0] + left_yellow_lane[-1][0]) // 2
            self.center_index = left_index + LANE_WIDTH_PIXELS // 2  # 가상의 중심선
            return "first_lane", left_yellow_lane, []

        elif right_lane and not left_white_lane:
            right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
            self.center_index = right_index - LANE_WIDTH_PIXELS // 2  # 가상의 중심선
            return "right_guided", [], right_lane

        else:
            self.center_index = self.center_pixel  # 정중앙
            return "go_straight", [], []
    def pth01_ctrl_decision_right(self):
        stop_line, yellow_left, yellow_right, white_left, white_right = self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane

        # 한 쪽 차선만 들어오는 경우를 위한 통합 처리
        left_white_lane = white_left
        left_yellow_lane = yellow_left
        right_lane = white_right
        #print(f"weight_left : {left_white_lane} / weight_right : {right_lane}")
            
        LANE_WIDTH_PIXELS = 260 # 적절히 조정

        if stop_line != []:
            cross_threshold = 35
            min_y, max_y = stop_line
            cross_diff = (max_y - min_y)
            if cross_threshold < cross_diff:
                self.stop_flag_num += 1
            return self.stop_flag_num, [], []
        elif left_white_lane and right_lane:
            left_index = (left_white_lane[0][0] + left_white_lane[-1][0]) // 2
            right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
            weight_left = len(left_white_lane)
            weight_right = len(right_lane)

            if weight_left > weight_right:
                left_index = (left_white_lane[0][0] + left_white_lane[-1][0]) // 2
                self.center_index = left_index + LANE_WIDTH_PIXELS // 2  # 가상의 중심선
                return "first_lane", left_white_lane, []

            else:
                right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
                self.center_index = right_index - LANE_WIDTH_PIXELS // 2  # 가상의 중심선
                return "second_lane", [], right_lane

        elif left_yellow_lane and right_lane:
            weight_left = len(left_yellow_lane)
            weight_right = len(right_lane)

            if weight_left > weight_right:
                left_index = left_yellow_lane[-1][0]
                self.center_index = left_index + LANE_WIDTH_PIXELS // 2  # 가상의 중심선
                return "first_lane", left_yellow_lane, [] 

            else:
                right_index = (right_lane[0][0] + right_lane[-1][0]) // 2
                self.center_index = right_index - LANE_WIDTH_PIXELS // 2  # 가상의 중심선
                return "second_lane", [], right_lane
        else:
            self.center_index = self.center_pixel  # 정중앙
            return "go_straight", [], []
        
    def pth01_calculate_curvature(self, x_vals, y_vals):
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
    def pth01_get_steer_gain(self, curvature):
        A = 500.0  # 최대 gain
        B = 0.0022
        return max(1.0, A * np.exp(-B * curvature))
    def pth01_get_base_speed(self, curvature):
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
    def pth01_ctrl_move(self, mode, left_lane=None, right_lane=None):
        pixel_error = self.center_index - self.center_pixel
        steer_error = pixel_error*self.steer_per_pixel
        print(f"left_lane:{left_lane} / right_lane:{right_lane}")
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

        curvature = self.pth01_calculate_curvature(x_vals, y_vals)
        steer_gain = self.pth01_get_steer_gain(curvature)
        base_speed = self.pth01_get_base_speed(curvature)

        pid_output = self.pid.compute(steer_error)
        steer = steer_gain*pid_output + 0.5
        steer = max(self.min_steer, min(self.max_steer, steer))  # 스티어링 제한

        deviation = abs(steer-0.5)
        speed = max(self.min_speed, int(base_speed - deviation * (base_speed - self.min_speed)))
        
        #self.last_steer = steer

        self.publish_move(speed, steer)
        
        rospy.loginfo(f"[PID] error: {steer_error:.4f}, output: {pid_output:.4f}")
        rospy.loginfo(f"[LCTRL] steer: {steer:.2f}, speed: {speed:.2f}")
        rospy.loginfo(f"[Curvature] value: {curvature:.2f}")

    def pth01_get_steer_gain_right(self, curvature):
        A = 20.0  # 최대 gain
        B = 0.0025
        return max(1.0, A * np.exp(-B * curvature))
    def pth01_ctrl_move_right(self, mode, left_lane=None, right_lane=None):
        pixel_error = self.center_index - self.center_pixel
        steer_error = pixel_error * self.steer_per_pixel
        print(f"left_lane:{left_lane} / right_lane:{right_lane}")
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

        curvature = self.pth01_calculate_curvature(x_vals, y_vals)
        steer_gain = self.pth01_get_steer_gain_right(curvature)
        pid_output = self.pid.compute(steer_error)

        now = time()

        time_points = [0, 1, 2, 3]  # 0초부터 시작되도록 0 추가
        steer_values = [0.7, 0.8, 0.9, 0.8]  # time_points 길이 맞춤

        if not hasattr(self, "sequence_active"):
            self.sequence_active = False
            self.sequence_start_time = 0

        if curvature <= 1000 and not self.sequence_active:
            self.sequence_active = True
            self.sequence_start_time = now
            rospy.loginfo("[Sequence] Started fixed steer sequence")

        if self.sequence_active:
            elapsed = now - self.sequence_start_time
            if elapsed > time_points[-1]:
                self.sequence_active = False
                rospy.loginfo("[Sequence] Finished fixed steer sequence")
                steer = steer_gain * pid_output + 0.5
                steer = max(self.min_steer, min(self.max_steer, steer))
            else:
                # time_points에 맞춰 가장 최근의 steer 값을 선택
                steer = steer_values[0]
                for i in range(1, len(time_points)):
                    if elapsed >= time_points[i]:
                        steer = steer_values[i]

        else:
            steer = steer_gain * pid_output + 0.5
            steer = max(self.min_steer, min(self.max_steer, steer))

        base_speed = self.pth01_get_base_speed(curvature)
        deviation = abs(steer - 0.5)
        speed = max(self.min_speed, int(base_speed - deviation * (base_speed - self.min_speed)))

        self.publish_move(speed, steer)

        rospy.loginfo(f"[PID] error: {steer_error:.4f}, output: {pid_output:.4f}")
        rospy.loginfo(f"[LCTRL] steer: {steer:.2f}, speed: {speed:.2f}")
        rospy.loginfo(f"[Curvature] value: {curvature:.2f}")
        rospy.loginfo(f"[steer_gain] value: {steer_gain:.2f}")
        rospy.loginfo(f"[stop_flag_num] value: {self.stop_flag_num:.2f}")


    def stop_time(self,time = 2):
        self.publish_move(0, 0.5)
        sleep(time)
    def normalize_angle(self, angle):
        # [-pi, pi] 범위로 정규화
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    def angle_to_target(self, x, y, w, tx, ty):
        # 현재 위치에서 목표 지점까지의 각도 차이 계산
        print(f"angle_to_target {x}")
        print(f"angle_to_target {y}")
        print(f"angle_to_target {w}")
        print(f"angle_to_target {tx}")
        print(f"angle_to_target {ty}")
        dx = - tx + x
        dy = - ty + y
        dist = math.hypot(dx, dy)  # 거리 계산
        # target_angle = math.atan2(dy, dx)
        target_angle = math.atan2(dx, dy)
        target_angle_deg = math.degrees(target_angle)  # 도(degree)
        
        # print(f"dx,dy {dx} {dy}")
        # print(f"target_angle {target_angle_deg}")
        # print(f"w {w}")
        w_rotated = w - 90  # 90도 회전
        # 0~360 범위로 정리하고 싶다면:
        w_rotated = (w_rotated + 360) % 360
        # heading 각도가 w (deg)로 주어졌다면 → rad로 변환
        heading_rad = math.radians(w)  # w는 degree (0~360 기준)
        heading_rad = math.radians(w_rotated)  # w는 degree (0~360 기준)
        # heading 방향 벡터 (단위 벡터)
        heading_vector = (math.cos(heading_rad), math.sin(heading_rad))

        # 타겟 방향 벡터
        # target_vector = (dx, dy)
        target_norm = math.hypot(dx, dy)
        target_vector = (dx / target_norm, dy / target_norm)
        # heading_vector, target_vector는 단위 벡터임
        # 내적, 외적
        dot = heading_vector[0] * target_vector[0] + heading_vector[1] * target_vector[1]
        cross = heading_vector[0] * target_vector[1] - heading_vector[1] * target_vector[0]

        # 부호 있는 각도 (라디안 → 도)
        angle_rad = math.atan2(cross, dot)
        angle_deg = math.degrees(angle_rad)

        print(f"벡터 간 부호 있는 회전 각도: {angle_deg:.2f}°")
        return angle_deg
    def compute_drive_command(self,x, y, w, tx, ty):
        # 거리 계산
        distance = math.hypot(tx - x, ty - y)

        # 도달 조건 (선택적으로 함수 외부에서 사용 가능)
        reached = distance < 0.5

        # 조향 계산
        angle_diff_deg = self.angle_to_target(x, y, w, tx, ty)
        print(f"angle_diff_deg {angle_diff_deg}")
        # [-90, 90]로 제한
        angle_diff_deg = max(min(angle_diff_deg, 90), -90)

        # -90~90 → -19.5~19.5로 선형 맵핑
        steer = (angle_diff_deg / 90) * 19.5
        
        speed = 400  # 고정 속도
        return speed, -steer, reached
    def chose_center_right(self):
        stop_line, yellow_left, yellow_right, white_left, white_right = self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane
        left_lane = yellow_left
        right_lane = white_right
        self.right_lane_delta = 134
        self.left_lane_delta = 163 + 20
        if white_right:
            self.center_index = right_lane[0][0] - self.right_lane_delta
        elif left_lane:
            self.center_index = left_lane[0][0] - self.right_lane_delta
        else:
            self.center_index = 320
        print(f"self.center_index {self.center_index}")
    def ctrl_moveByLine_right(self):
        # self.center_pixel = 320
        # self.steer_per_pixel = 2 / 640  # 수정 가능
        # steer = (steer + 1) / 2.0
        # print(f"[INFO] steer: {steer:.2f} deg")
        pixel_error = self.center_index - self.center_pixel  # +면 우측, -면 좌측
        print(f"[INFO] pixel_error: {pixel_error:.2f} deg")
        steer = pixel_error * self.steer_per_pixel * self.total_steer * 2 + 0.5
        print(f"[INFO] pixel_error: {steer:.2f} deg")
        # steer = self.pid.compute(pixel_error)
        # 클리핑 (조향각 범위 제한)
        steer = max(min(steer, self.max_steer), self.min_steer)
        # steer_ratio = abs(steer) / self.max_steer  # 0 ~ 1
        # speed = self.max_speed * (1 - steer_ratio)  # 회전 클수록 속도 감소
        # speed = max(speed, self.min_speed)
        speed = 400
        # 5. 결과 저장 혹은 publish
        self.publish_move(speed,steer)
        print(f"[INFO] steer: {steer:.2f} deg, speed: {speed:.2f} km/h")
    def chose_center_left(self):
        stop_line, yellow_left, yellow_right, white_left, white_right = self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane
        left_lane = white_left
        right_lane = white_right
        self.right_lane_delta = 134
        self.left_lane_delta = 163 - 10
        if left_lane:
            self.center_index = (left_lane[0][0] + left_lane[-1][0]) // 2 + self.left_lane_delta
        elif right_lane:
            self.center_index = right_lane[0][0] + self.left_lane_delta
        else:
            self.center_index = 320
        print(f"self.center_index {self.center_index}")
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
        self.publish_move(speed,steer)
        return False
    
    def check_obstacle_rotary(self):
        if self.left_obstacle or self.front_obstacle or self.right_obstacle:
            return True 
        return False
