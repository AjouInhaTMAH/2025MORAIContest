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
from lane.PIDController import PIDController
import numpy as np
from drive_decision.ctrl import ctrl_motor_servo
class DecLaneCurvature:
    def __init__(self,CtrlMotorServo):
        """_summary_
        현재 방법이 2가지로 나누었기에 함수에 이 이름들을 일단 넣도록 한다.
        방법 1, pt01 - 곡률 계산
        방법 2, pt02 - lane으로 일정 거리 계산 
        """
        print(f"DecLaneCurvature create")
        self.init_both_pth()
        self.init_pth01()
        self.init_processing(CtrlMotorServo)

    def init_both_pth(self):
        self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane = [],None,None,None,None
        self.center_index = 0
        self.center_pixel = 320
        self.steer_per_pixel = 2 / 640  # 수정 가능
        self.max_steer = 1 #19.5
        self.min_steer = 0 #-19.5
        self.hold_until_ts = 0
    def init_pth01(self):
        self.stop_flag_num = 0
        self.max_speed = 1200
        self.min_speed = 700
        self.pid = PIDController()
        
        self.sequence_active = False
        self.sequence_start_time = 0
        self.LANE_WIDTH_PIXELS = 260
        
        self.steer_hold_until_ts = 0.0
        
        self.goal_stop_line = 0        
        
    def init_processing(self,CtrlMotorServo:ctrl_motor_servo.CtrlMotorServo):
        self.CtrlMotorServo = CtrlMotorServo
    def set_camera_info(self,stop_line,yellow_left_lane,yellow_right_lane,white_left_lane,white_right_lane):
        self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane = stop_line,yellow_left_lane,yellow_right_lane,white_left_lane,white_right_lane

    def pth01_ctrl_decision(self):
        stop_line, yellow_left, yellow_right, white_left, white_right = self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane

        # 한 쪽 차선만 들어오는 경우를 위한 통합 처리
        left_white_lane = white_left
        left_yellow_lane = yellow_left
        right_lane = white_right
        #print(f"weight_left : {left_white_lane} / weight_right : {right_lane}")
            

        if stop_line != []:
            cross_threshold = 35
            min_y, max_y = stop_line
            cross_diff = (max_y - min_y)
            if cross_threshold < cross_diff:
                self.stop_flag_num += 1
            return self.stop_flag_num, [], []
        
        if left_yellow_lane and stop_line:
            return self.move_forward(left_yellow_lane)

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
        A = 40.0  # 최대 gain
        B = 0.002528 # 0.002528
        return max(1.2, A * np.exp(-B * curvature))
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
        self.center_pixel = 320 
        pixel_error = self.center_index - self.center_pixel
        steer_error = pixel_error*self.steer_per_pixel
        print(f"left_lane:{left_lane} / right_lane:{right_lane}")
        
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

        self.CtrlMotorServo.pub_move_motor_servo(speed, steer)
        
        rospy.loginfo(f"[PID] error: {steer_error:.4f}, output: {pid_output:.4f}")
        rospy.loginfo(f"[LCTRL] steer: {steer:.2f}, speed: {speed:.2f}")
        rospy.loginfo(f"[Curvature] value: {curvature:.2f}")

    def get_steer_gain_goal(self, curvature):
        A = 40.0  # 최대 gain
        B = 0.000298 # 0.002528
        return max(1.2, A * np.exp(-B * curvature))
    
    def chosse_left_lane_mission1(self,left_lane, right_lane):
        left_index = (left_lane[0][0] + left_lane[-1][0]) // 2
        self.center_index = left_index + self.LANE_WIDTH_PIXELS // 2  # 가상의 중심선
        return "first_lane", left_lane, []
    def ctrl_decision_mission1(self):
        stop_line, yellow_left, yellow_right, white_left, white_right = self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane

        # 한 쪽 차선만 들어오는 경우를 위한 통합 처리
        left_white_lane = white_left
        left_yellow_lane = yellow_left
        right_lane = white_right
        #print(f"weight_left : {left_white_lane} / weight_right : {right_lane}")
            

        if stop_line != []:
            cross_threshold = 35
            min_y, max_y = stop_line
            cross_diff = (max_y - min_y)
            if cross_threshold < cross_diff:
                self.stop_flag_num += 1
            return self.stop_flag_num, [], []
        
        if left_yellow_lane and stop_line:
            return self.move_forward(left_yellow_lane)

        elif left_white_lane and right_lane:
            return self.chosse_left_lane_mission1(left_white_lane, right_lane)
        
        elif left_yellow_lane and right_lane:
            return self.chosse_left_lane_mission1(left_yellow_lane, right_lane)
        
        elif left_white_lane and not right_lane:
            return self.follow_left_lane(left_white_lane)

        elif left_yellow_lane and not right_lane:
            return self.follow_left_lane(left_yellow_lane)

        elif right_lane and not left_white_lane:
            return self.follow_right_lane(right_lane)

        else:
            return self.forward()
    def get_steer_gain_mission1(self, curvature):
        A = 40.0  # 최대 gain
        B = 0.000298 # 0.002528
        return max(1.2, A * np.exp(-B * curvature))
    def ctrl_move_mission1(self, mode, left_lane=None, right_lane=None,stop_flag = False):
        self.center_pixel = 320 + 30
        
        pixel_error = self.center_index - self.center_pixel
        steer_error = pixel_error*self.steer_per_pixel
        print(f"left_lane:{left_lane} / right_lane:{right_lane}")
        
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
        steer_gain = self.get_steer_gain_mission1(curvature)
        base_speed = self.pth01_get_base_speed(curvature)

        pid_output = self.pid.compute(steer_error)
        steer = steer_gain*pid_output + 0.5
        steer = max(self.min_steer, min(self.max_steer, steer))  # 스티어링 제한

        deviation = abs(steer-0.5)
        speed = max(self.min_speed, int(base_speed - deviation * (base_speed - self.min_speed)))
        
        #self.last_steer = steer

        self.CtrlMotorServo.pub_move_motor_servo(speed, steer)
        
        rospy.loginfo(f"[PID] error: {steer_error:.4f}, output: {pid_output:.4f}")
        rospy.loginfo(f"[LCTRL] steer: {steer:.2f}, speed: {speed:.2f}")
        rospy.loginfo(f"[Curvature] value: {curvature:.2f}")
    
    def ctrl_move_goal(self, mode, left_lane=None, right_lane=None,stop_flag = False):
        now_ts = time()

        # 2초 홀드 중이면 즉시 고정 출력
        if now_ts < self.steer_hold_until_ts:
            elapsed = self.steer_hold_until_ts - now_ts  # 남은 시간
            hold_duration = 2.3
            passed = hold_duration - elapsed             # 지난 시간

            # (경과 시간, 스티어 값) 리스트
            steer_pattern = [
                (0.0, 0.5),  # 시작~0.45초
                (0.25, 0.6),  # 시작~0.45초
                (0.5, 0.7),  # 시작~0.45초
                (0.7, 0.8),  # 시작~0.45초
                (0.9, 0.9),  # 시작~0.45초
                (1.1, 1.0),  # 0.45~2초
                (2.3, 0.5),  # 0.45~2초
            ]

            steer_val = 0.55  # 기본값
            for t_point, s_val in steer_pattern:
                if passed >= t_point:
                    steer_val = s_val
                else:
                    break

            self.CtrlMotorServo.pub_move_motor_servo(1000, steer_val)
            rospy.loginfo(f"[CURV HOLD pattern] t={passed:.2f}s steer={steer_val:.2f}")
            return
        self.center_pixel = 320 - 20
        pixel_error = self.center_index - self.center_pixel
        steer_error = pixel_error * self.steer_per_pixel
        print(f"left_lane:{left_lane} / right_lane:{right_lane} / stop_line:{stop_flag}")

        x_vals, y_vals = [], []
        # if left_lane:
        #     for pt in left_lane:
        #         x_vals.append(pt[0])
        #         y_vals.append(pt[1])

        # elif right_lane:
        #     for pt in right_lane:
        #         x_vals.append(pt[0])
        #         y_vals.append(pt[1])
        if right_lane:
            for pt in left_lane:
                x_vals.append(pt[0])
                y_vals.append(pt[1])

        elif left_lane:
            for pt in right_lane:
                x_vals.append(pt[0])
                y_vals.append(pt[1])

        curvature = self.pth01_calculate_curvature(x_vals, y_vals)

            # stop_flag True이고 curvature 조건을 만족하면 2.5초 홀드 시작
        if stop_flag and self.goal_stop_line == 0: # and curvature <= 1000
            self.steer_hold_until_ts = now_ts + 2.3
            # 홀드 시작 프레임에서는 즉시 첫 패턴 값 적용
            self.goal_stop_line += 1
            self.CtrlMotorServo.pub_move_motor_servo(1000, 0.5)
            rospy.loginfo("[CURV HOLD start] 3s steer pattern")
            return
        print(f"modemodemode {mode}")
        print(f"modemodemode {mode}")
        
        # 3초 홀드 중이면 무조건 직진 유지
        if now_ts < self.hold_until_ts:
            self.CtrlMotorServo.pub_move_motor_servo(1000, 0.5)
            rospy.loginfo("[HOLD] go straight (%.2fs left)" % (self.hold_until_ts - now_ts))
            return
        
        # go_straight가 트리거된 '그 순간'에만 3.5초 홀드 시작
        if stop_flag and self.goal_stop_line == 1:
            self.hold_until_ts = now_ts + 3.5
            self.CtrlMotorServo.pub_move_motor_servo(1000, 0.5)
            rospy.loginfo("[HOLD start] 3s straight")
            return
        
        steer_gain = self.get_steer_gain_goal(curvature)
        pid_output = self.pid.compute(steer_error)
        steer = steer_gain * pid_output + 0.5
                       
        steer = max(self.min_steer, min(self.max_steer, steer))
        base_speed = self.pth01_get_base_speed(curvature)
        deviation = abs(steer - 0.5)
        speed = max(self.min_speed, int(base_speed - deviation * (base_speed - self.min_speed)))

        self.CtrlMotorServo.pub_move_motor_servo(speed, steer)

        rospy.loginfo(f"[PID] error: {steer_error:.4f}, output: {pid_output:.4f}")
        rospy.loginfo(f"[LCTRL] steer: {steer:.2f}, speed: {speed:.2f}")
        rospy.loginfo(f"[Curvature] value: {curvature:.2f}")
        rospy.loginfo(f"[steer_gain] value: {steer_gain:.2f}")