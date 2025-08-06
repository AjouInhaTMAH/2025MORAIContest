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

    def init_pub(self):
        rospy.Subscriber("/perception/camera", String, self.CB_camera_info, queue_size=1)
        rospy.Subscriber("/perception/lidar", String, self.CB_lidar_info, queue_size=1)
        self.motor_pub = rospy.Publisher('/commands/motor/ctrl', Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('/commands/servo/ctrl', Float64, queue_size=1)
        self.lane_mode = 1  # 0=기본, 1=왼쪽 차선만, 2=오른쪽 차선만 등
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
        self.is_target_car_lane_1 = True
        self.is_target_car_lane_1 = False
        self.car_lane_number = {"null" : -1, "forward" : 0, "left" : 1, "right" : 2}
        self.current_car_lane_number = -1
        self.prev_car_lane_number = -1
        self.car_lane_env = {"null" : -1, "forward" : 0, "left" : 1, "right" : 2}
        self.current_car_lane_env = -1
        self.prev_car_lane_env = -1
        

        self.current_left_lane = None
        self.current_right_lane = None
        
        self.curve_degree = 50
        self.car_center_pixel = 320
        self.car_steer_per_pixel = 1 / 640
        self.pid_steer = PIDController()
        # self.pid_steer = PIDController(Kp=0.1, Ki=0.001, Kd=0.000001)
        
        self.max_speed = 4
        self.min_speed = 2
        self.max_steer = 19.5
        self.min_steer = -19.5
        self.total_steer = self.max_steer - self.min_steer
        self.pre_spped = 1
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
        
    def check_lanes_number(self):
        if self.is_target_car_lane_1:
            if self.yellow_left_lane != [] and self.white_right_lane != []:
                self.current_left_lane = self.yellow_left_lane
                self.current_right_lane = self.white_right_lane
                self.current_car_lane_number = self.car_lane_number["forward"]
            elif self.yellow_right_lane != []:
                self.current_car_lane_number = self.car_lane_number["right"]
            elif self.white_left_lane == [] and self.white_right_lane != []:
                self.current_car_lane_number = self.car_lane_number["right"]
            elif self.white_left_lane != [] and self.white_right_lane == []:
                self.current_car_lane_number = self.car_lane_number["left"]
            elif self.white_left_lane != [] and self.white_right_lane != []:
                if self.prev_car_lane_number == self.car_lane_number["right"]:
                    self.current_car_lane_number = self.car_lane_number["right"]
                else:
                    self.current_car_lane_number = self.car_lane_number["left"]
            else:
                self.current_car_lane_number = self.car_lane_number["null"]
        else:
            if self.yellow_left_lane != [] and self.white_right_lane != []:
                self.current_car_lane_number = self.car_lane_number["right"]
            elif self.yellow_right_lane != []:
                self.current_car_lane_number = self.car_lane_number["right"]
            elif self.white_left_lane == [] and self.white_right_lane != []:
                self.current_car_lane_number = self.car_lane_number["right"]
            elif self.white_left_lane != [] and self.white_right_lane == []:
                self.current_car_lane_number = self.car_lane_number["left"]
            elif self.white_left_lane != [] and self.white_right_lane != []:
                if self.white_left_lane != [] and self.white_right_lane != []:
                    self.current_left_lane = self.white_left_lane
                    self.current_right_lane = self.white_right_lane
                    self.current_car_lane_number = self.car_lane_number["forward"]
                else:
                    self.current_car_lane_number = self.car_lane_number["right"]
            else:
                self.current_car_lane_number = self.car_lane_number["null"]
        self.prev_car_lane_number = self.current_car_lane_number
    def check_lanes_env(self):
        self.prev_car_lane_env = self.current_car_lane_env
        if self.current_car_lane_number == self.car_lane_number["forward"]:
            detect_lane_center = self.detect_center()
            point_direction = self.car_center_pixel - detect_lane_center
            if point_direction > self.curve_degree:
                self.current_car_lane_env = self.car_lane_env["left"]
                pass
            elif point_direction < self.curve_degree:
                self.current_car_lane_env = self.car_lane_env["right"]
                pass
            else:
                self.current_car_lane_env = self.car_lane_env["forward"]
                pass
        else:
            self.current_car_lane_env = self.car_lane_env["null"]
    def check_lanes_status(self):
        self.check_lanes_number()
        self.check_lanes_env()
    
    def move_left(self):
        self.motor_pub.publish(8)
        self.servo_pub.publish(-19.5)
    def move_right(self):
        self.motor_pub.publish(8)
        self.servo_pub.publish(19.5)
    def move_null(self):
        self.motor_pub.publish(8)
        self.servo_pub.publish(-0)
        
    def detect_center(self):
        # lane_centers = []
        # for i in range(3):
        #     left = self.current_left_lane[i] if len(self.current_left_lane) > i else None
        #     right = self.current_right_lane[i] if len(self.current_right_lane) > i else None

        #     if left is not None and right is not None:
        #         center = (left + right) / 2.0
        #     elif left is not None:
        #         center = left
        #     elif right is not None:
        #         center = right
        #     else:
        #         continue  # 둘 다 없으면 스킵

        #     lane_centers.append(center)

        # if lane_centers:
        #     detect_lane_center = sum(lane_centers) / len(lane_centers)
        # else:
        #     detect_lane_center = (self.current_left_lane[0] + self.current_right_lane[0]) / 2.0
        
        alpha = 0.85
        start_center = (self.current_left_lane[0] + self.current_right_lane[0]) / 2.0
        end_center = (self.current_left_lane[-1] + self.current_right_lane[-1]) / 2.0
        detect_lane_center = start_center * alpha + (1 - alpha) * end_center
        
        detect_lane_center = (self.current_left_lane[0] + self.current_right_lane[0]) / 2.0
        return detect_lane_center
    def moveByLine(self):
        if self.current_car_lane_number == self.car_lane_number["null"]:
            self.move_null()
        elif self.current_car_lane_number == self.car_lane_number["left"]:
            self.move_left()
        elif self.current_car_lane_number == self.car_lane_number["right"]:
            self.move_right()
        else:
            # 1. 차선 중심 계산
            detect_lane_center = self.detect_center()
            # detect_lane_center = (self.current_left_lane[-1] + self.current_right_lane[-1]) / 2.0
            # 2. 차선 중심과 차량 중심의 픽셀 오차
            pixel_error = detect_lane_center - self.car_center_pixel  # +면 우측, -면 좌측
            # 3. 조향각 계산 (픽셀 오차 → 각도로 환산)
            if self.pre_spped > 7:
                pixel_error = pixel_error * self.car_steer_per_pixel * self.max_steer
            else:
                # print(f"steersteersteersteersteersteersteersteer {pixel_error}")
                pixel_error = pixel_error * self.car_steer_per_pixel * self.total_steer * 3
                # print(f"steersteersteersteersteersteersteersteer {pixel_error}")
            steer = self.pid_steer.compute(pixel_error)
            # 클리핑 (조향각 범위 제한)
            steer = max(min(steer, self.max_steer), self.min_steer)
            # 4. 속도 계산 (회전이 클수록 느려짐)
            steer_ratio = abs(steer) / self.max_steer  # 0 ~ 1
            speed = self.max_speed * (1 - steer_ratio)  # 회전 클수록 속도 감소
            speed = max(speed, self.min_speed)
            self.pre_spped = speed
            # 5. 결과 저장 혹은 publish
            self.motor_pub.publish(speed)
            self.servo_pub.publish(steer)
            print(f"[INFO] steer: {steer:.2f} deg, speed: {speed:.2f} km/h")
        
    def check_mission_change(self):
        if self.prev_car_lane_env == self.car_lane_env["left"] and self.current_car_lane_env == self.car_lane_env["forward"]:
            self.current_car_mission = 2

    def mission2_ctrl(self):
        self.is_target_car_lane_1 = False
        # self.is_target_car_lane_1 = True
        # self.check_mission_change()
        # ------------------------- 
        # 각자 해야하는 제어가 있을 것이다.
        # 미션 2 레이더 값이 있다. 전방에 뭐 있으면 걸고 거기서 제어를 한다.
        
        self.moveByLine() # 차선 따라가는 함수

    def mission3_ctrl(self):
        self.motor_pub.publish(0)

    def stop_time(self):
        self.motor_pub.publish(0)
        self.servo_pub.publish(0)
        sleep(2)
    def stop(self):
        self.motor_pub.publish(0)
        self.servo_pub.publish(0)

    def is_obstacle_rotary(self):
        if self.left_obstacle or self.front_obstacle or self.right_obstacle:
            return True 
        return False
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
        dx = tx - x
        dy = ty - y
        dist = math.hypot(dx, dy)  # 거리 계산
        target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - w)
        
        weight = max(1.0, 5.0 / (dist + 1e-5))  # 5.0은 조정 가능
        weighted_steer = angle_diff * weight
        print(f"angle_diff {angle_diff}")
        print(f"weighted_steer {weighted_steer}")
        return weighted_steer
    
    def compute_drive_command(self,x, y, w, tx, ty):
        # 거리 계산
        distance = math.hypot(tx - x, ty - y)

        # 도달 조건 (선택적으로 함수 외부에서 사용 가능)
        reached = distance < 0.2

        # 조향 계산
        angle_diff_rad = self.angle_to_target(x, y, w, tx, ty)
        angle_diff_deg = math.degrees(angle_diff_rad)
        print(f"angle_diff_deg {angle_diff_deg}")
        # [-90, 90]로 제한
        angle_diff_deg = max(min(angle_diff_deg, 90), -90)

        # -90~90 → -19.5~19.5로 선형 맵핑
        steer = (angle_diff_deg / 90) * 19.5
        
        speed = 0.5  # 고정 속도
        return speed, -steer, reached
    
    def drvie_amcl(self):
        # 현재 위치
        x = self.x
        y = self.y
        w = self.yaw
        if self.lane_mode == 1:
            tx, ty = self.zones[401]
        elif self.lane_mode == 401:
            tx, ty = self.zones[402]
        elif self.lane_mode == 402:
            tx, ty = self.zones[403]
        elif self.lane_mode == 403:
            tx, ty = self.zones[404]
        elif self.lane_mode == 404:
            self.mi4_out_flag = True
            return
        print(f"self.lane_mode {self.lane_mode}")
        speed, steer, reached = self.compute_drive_command(x, y, w, tx, ty)
        # print(f"reached {reached}")
        print(f"speed, steer {speed} {steer}")
        self.motor_pub.publish(speed)
        self.servo_pub.publish(steer)
        
    def mission4_fast_hardcoding(self):
        if self.mi4_out_flag:
            print(f"5")
            self.moveByLine() # 차선 따라가는 함수
        elif self.mi4_in_flag:
            self.motor_pub.publish(8)
            self.servo_pub.publish(-8.5)
            sleep(0.16)
            self.motor_pub.publish(7)
            self.servo_pub.publish(19.5)
            sleep(0.425)
            self.mi4_out_flag = True
            print(f"4")
            self.stop_time()
        elif self.mi4_stop_flag:
            print(f"3")
            if self.is_obstacle_rotary():
                return
            print(f"no obs")
            self.motor_pub.publish(8)
            self.servo_pub.publish(0)
            sleep(0.275)
            self.motor_pub.publish(8.0)
            self.servo_pub.publish(19.5)
            sleep(0.4)
            self.mi4_in_flag = True
        elif self.stop_line != [] and self.stop_line[MAX_Y] > 440:
            print(f"2")
            self.mi4_stop_flag =True
            # self.stop_time()
            self.stop()
        else:
            print(f"1")
            self.moveByLine() # 차선 따라가는 함수
    def mission4_amcl_following(self):
        if self.mi4_out_flag:
            print(f"5")
            self.moveByLine() # 차선 따라가는 함수
        elif self.mi4_in_flag:
            print(f"4")
            self.drvie_amcl()
            # 들어가서 이동하고 나감 + 앞에 뭐 있으면 속도 낮추거나 해야함
        elif self.mi4_stop_flag:
            print(f"3")
            if self.is_obstacle_rotary():
                return
            print(f"no obs")
            self.mi4_in_flag = True
        elif self.stop_line != [] and self.stop_line[MAX_Y] > 440:
            print(f"2")
            self.mi4_stop_flag =True
            # self.stop_time()
            self.stop()
        else:
            print(f"1")
            self.moveByLine() # 차선 따라가는 함수
            
    def mission4_ctrl(self):
        self.is_target_car_lane_1 = False
        # self.mission4_fast_hardcoding()
        self.mission4_amcl_following()

    def processing(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            start = time()
            # print(f"self.stop_line {self.stop_line}")
            self.check_lanes_status()
            if self.car_mission_status[self.current_car_mission] == 1:
                pass
            elif self.car_mission_status[self.current_car_mission] == 2:
                print(f"mission 2")
                print(f"self.current_car_lane_number {self.current_car_lane_number}")
                print(f"self.current_left_lane, {self.current_left_lane}, self.current_right_lane, {self.current_right_lane}")
                self.mission2_ctrl()
                pass
            elif self.car_mission_status[self.current_car_mission] == 3:
                print(f"mission 3")
                self.mission3_ctrl()
            elif self.car_mission_status[self.current_car_mission] == 4:
                print(f"mission 4")
                self.mission4_ctrl()
            elif self.car_mission_status[self.current_car_mission] == 5:
                pass
            
            end = time()
            print(f"time2 {end - start}")
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