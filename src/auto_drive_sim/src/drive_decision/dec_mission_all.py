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
MIN_Y = 0
MAX_Y = 1

class DecMissionAll:
    def __init__(self):
        print(f"DecMissionAll start")
        rospy.init_node('dec_mission_all_node')
        self.init_lidar_info()
        self.init_camera_info()
        self.init_pub()
        self.car_mission_status = [0,1,2,3,4,5]
        self.current_car_mission = 4
        self.init_car_lane()
        self.init_mission4()
        self.init_goal()

    def init_pub(self):
        rospy.Subscriber("/perception/camera", String, self.CB_camera_info, queue_size=1)
        rospy.Subscriber("/perception/lidar", String, self.CB_lidar_info, queue_size=1)
        # self.motor_pub = rospy.Publisher('/commands/motor/ctrl', Float64, queue_size=1)
        # self.servo_pub = rospy.Publisher('/commands/servo/ctrl', Float64, queue_size=1)
        self.motor_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
        self.lane_mode = 3  # 0=기본, 1=왼쪽 차선만, 2=오른쪽 차선만 등
        rospy.Subscriber("/lane_mode", Int32, self.CB_car_nav)
        rospy.Subscriber('/dr_info', String, self.callback)
        self.x = 0
        self.y = None
        self.w = 0
        self.vel = 0
        self.zones = {
            1: (0.042443, 8.8616749),   # mission 2 & 3 영역
            2: (4.7866139, 4.34265),    # mission 5 영역
            3: (8.2325, 1.7338),     # mission 5 영역
            401:(3.5279, 7.048),
            402:(4.0596, 7.0403),
            403:(4.6748, 6.52258),
            404:(4.8693, 6.2139),
        }
        self.motor_cmd_msg_pub = Float64()
        self.servo_cmd_msg_pub = Float64()
        self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane = [],None,None,None,None
    
    def init_goal(self):
        self.path1_01flag = False
        self.path1_02flag = False
        
    def init_lidar_info(self):
        self.left_obstacle = False
        self.front_obstacle = False
        self.right_obstacle = False
    def init_camera_info(self):
        self.stop_line = []
        self.yellow_left_lane = None
        self.yellow_right_lane = None
        self.white_left_lane = None
        self.white_right_lane = None 
    def init_car_lane(self):
        self.pid = PIDController()
        self.center_pixel = 320
        self.max_steer = 1 #19.5
        self.min_steer = 0 #-19.5
        self.total_steer = self.max_steer - self.min_steer
        self.steer_per_pixel = 2 / 640  # 수정 가능
        self.max_speed = 1200
        self.min_speed = 400
        self.stop_flag = False
        self.stop_flag_num = 0
        self.center_index = 0
        
    def init_mission4(self):
        self.mi4_stop_flag = False
        self.mi4_in_flag = False
        self.mi4_out_flag = False
    def callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.x = data.get("x")
            self.y = data.get("y")
            self.yaw = data.get("yaw")
            self.vel = data.get("vel")
        except json.JSONDecodeError as e:
            rospy.logwarn(f"JSON Decode Error: {e}")
        except Exception as e:
            rospy.logerr(f"Unexpected error: {e}")
            
    def CB_lidar_info(self,msg):
        try:
            data = json.loads(msg.data)
            self.left_obstacle, self.front_obstacle, self.right_obstacle = data[0],data[1],data[2]
        except Exception as e:
            print("복원 실패:", e)
    def CB_camera_info(self,msg):
        try:
            data = json.loads(msg.data)
            self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane = data[0],data[1],data[2],data[3],data[4]
        except Exception as e:
            print("복원 실패:", e)
    def CB_car_nav(self, msg):
        self.lane_mode = msg.data  # 예: 1, 2, 3 등의 영역 구분  

    def publish(self, speed, steer):
        speed_msg = Float64()
        steer_msg = Float64()
        speed_msg.data = speed
        steer_msg.data = steer
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

    def ctrl_decision(self):
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
    def ctrl_decision_left(self):
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
    def ctrl_decision_right(self):
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
    def ctrl_move(self, mode, left_lane=None, right_lane=None):
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
    def ctrl_move_right(self, mode, left_lane=None, right_lane=None):
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

        curvature = self.calculate_curvature(x_vals, y_vals)
        if curvature < 2000:
            steer = 0.88
            base_speed = self.get_base_speed(2000)
            pid_output = 0
        else:
            steer_gain = self.get_steer_gain_right(curvature)
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

    def action_mission2_3(self):
        mode, left_lane, right_lane = self.ctrl_decision()
        self.ctrl_move(mode, left_lane, right_lane)

        # 부드러운 steer_gain 계산 함수
    def action_mission4(self):
        self.mission4_fast_hardcoding()
    def action_mission5(self):
        mode, left_lane, right_lane = self.ctrl_decision_right()
        self.ctrl_move(mode, left_lane, right_lane)

        
    def get_steer_gain(self, curvature):
        A = 500.0  # 최대 gain
        B = 0.0022
        return max(1.0, A * np.exp(-B * curvature))
    def get_steer_gain_right(self, curvature):
        A = 500.0  # 최대 gain
        B = 0.008
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

    def is_obstacle_rotary(self):
        if self.left_obstacle or self.front_obstacle or self.right_obstacle:
            return True 
        return False
    
    def stop_time(self,time = 2):
        self.motor_pub.publish(0)
        self.servo_pub.publish(0)
        sleep(time)
        
    def mission4_fast_hardcoding(self):
        if self.mi4_out_flag:
            print(f"5")
            mode, left_lane, right_lane = self.ctrl_decision_left()
            self.ctrl_move(mode, left_lane, right_lane)
        elif self.mi4_in_flag:
            self.motor_pub.publish(2400)
            self.servo_pub.publish(-0.7179)
            sleep(0.16)
            self.motor_pub.publish(2100)
            self.servo_pub.publish(1)
            sleep(0.425)
            self.mi4_out_flag = True
            print(f"4")
            self.stop_time()
        elif self.mi4_stop_flag:
            print(f"3")
            if self.is_obstacle_rotary():
                return
            print(f"no obs")
            self.motor_pub.publish(2400)
            self.servo_pub.publish(0.5)
            sleep(0.275)
            self.motor_pub.publish(2400)
            self.servo_pub.publish(1)
            sleep(0.4)
            self.mi4_in_flag = True
        elif self.stop_line != [] and self.stop_line[MAX_Y] > 320:
            print(f"2")
            self.mi4_stop_flag =True
            self.stop_time(2)
        else:
            print(f"1")
            mode, left_lane, right_lane = self.ctrl_decision_left()
            self.ctrl_move(mode, left_lane, right_lane)
    
    def action_go_goal(self):
        self.steer_01 = [0.60,0.8,0.75]
        self.time_01 = [0.5,2.5,0.5]
        if self.path1_02flag:
            print(f"hi2")
            self.publish(0,0)
            # mode, left_lane, right_lane = self.ctrl_decision()
            # self.ctrl_move(mode, left_lane, right_lane)
        elif self.path1_01flag:
            print(f"hi1")
            for steer in self.steer_01:
                self.publish(400,steer)
                sleep(1)
                print(f"steer {steer}")
            self.path1_02flag = True
            
        elif self.stop_line != [] and self.stop_line[MAX_Y] > 400:
            self.path1_01flag = True
            self.path1_02flag = True
        else:
            mode, left_lane, right_lane = self.ctrl_decision()
            self.ctrl_move(mode, left_lane, right_lane)

    def processing(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            start = time()
            # print(f"self.stop_line {self.stop_line}")
            if self.yellow_left_lane is not None or self.white_left_lane is not None or self.white_right_lane is not None:
                if self.lane_mode == 0:
                    print(f"mode {self.lane_mode}")
                    self.action_mission2_3()
                elif self.lane_mode == 1:
                    print(f"mode {self.lane_mode}")
                    self.action_mission4()
                elif self.lane_mode == 2:
                        print(f"mode {self.lane_mode}")
                        self.action_mission5()
                elif self.lane_mode == 3:
                        print(f"mode {self.lane_mode}")
                        self.action_go_goal()
                else:
                    print(f"self.lane_mode {self.lane_mode}")
                    mode, left_lane, right_lane = self.ctrl_decision_right()
                    self.ctrl_move_right(mode, left_lane, right_lane)
            end = time()
            # print(f"time2 {end - start}")
            self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane = [],None,None,None,None
            
            rate.sleep()
            
if __name__ == '__main__':
    node_name = '/throttle_interpolator'

    try:
        subprocess.run(['rosnode', 'kill', node_name], check=True)
        print(f"✅ 노드 {node_name} 종료 성공")
    except subprocess.CalledProcessError:
        print(f"❌ 노드 {node_name} 종료 실패 (이미 종료되었거나 존재하지 않음)")
    node = DecMissionAll()
    node.processing()   # spin 대신 processing 돌기