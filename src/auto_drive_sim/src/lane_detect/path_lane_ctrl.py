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
MIN_Y = 0
MAX_Y = 1
class ctrl_oath:
    def __init__(self):
        self.pub_init()
        
        self.car_mission_status = [0,1,2,3,4,5]
        self.current_car_mission = 2
        
        self.car_lane_init()


    def pub_init(self):
        self.motor_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.motor_cmd_msg_pub = Float64()
        self.servo_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
        self.servo_cmd_msg_pub = Float64()
        self.rate_motor = rospy.Rate(20)
        self.rate_motor = rospy.Rate(20)
    def car_lane_init(self):
        self.is_target_car_lane_1 = True
        self.car_lane_number = {"null" : -1, "forward" : 0, "left" : 1, "right" : 2}
        self.current_car_lane_number = -1
        self.prev_car_lane_number = -1
        self.car_lane_env = {"null" : -1, "forward" : 0, "left" : 1, "right" : 2}
        self.current_car_lane_env = -1
        self.prev_car_lane_env = -1
        
        self.stop_line = None
        self.yellow_left_lane = None
        self.yellow_right_lane = None
        self.white_left_lane = None
        self.white_right_lane = None 
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
    
    def motor_pub_func(self,speed):
        self.motor_cmd_msg_pub.data = speed * 300
        self.motor_pub.publish(self.motor_cmd_msg_pub)
        # self.rate_motor.sleep()
    def servo_pub_func(self,servo):
        self.servo_cmd_msg_pub.data = ((servo / 19.5 + 1)) /2
        self.servo_pub.publish(self.servo_cmd_msg_pub)
        # self.rate_motor.sleep()
    
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
        self.motor_pub_func(8)
        self.servo_pub_func(-19.5)
    def move_right(self):
        self.motor_pub_func(8)
        self.servo_pub_func(19.5)
    def move_null(self):
        self.motor_pub_func(8)
        self.servo_pub_func(-0)
        
    def detect_center(self):
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
                print(f"steersteersteersteersteersteersteersteer {pixel_error}")
                print(f"아래 값을 조절하면 됩니다. 곱하는 값이 크면 회전을 잘하는데 직진을 못하고, 작으면 직진은 잘하는데, 회전을 못합니다.")
                pixel_error = pixel_error * self.car_steer_per_pixel * self.total_steer * 3
                print(f"steersteersteersteersteersteersteersteer {pixel_error}")
            steer = self.pid_steer.compute(pixel_error)
            # 클리핑 (조향각 범위 제한)
            steer = max(min(steer, self.max_steer), self.min_steer)
            # 4. 속도 계산 (회전이 클수록 느려짐)
            steer_ratio = abs(steer) / self.max_steer  # 0 ~ 1
            speed = self.max_speed * (1 - steer_ratio)  # 회전 클수록 속도 감소
            speed = max(speed, self.min_speed)
            self.pre_spped = speed
            # 5. 결과 저장 혹은 publish
            self.motor_pub_func(speed)
            self.servo_pub_func(steer)
            print(f"[INFO] steer: {steer:.2f} deg, speed: {speed:.2f} km/h")

    def ctrl_move(self,dataset):
        print(f"ctrl 부분으로 주행을 제어한다.")
        self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane = dataset
        self.check_lanes_status()
        self.is_target_car_lane_1 = True # 1차선 주행
        self.is_target_car_lane_1 = False # 2차선 주행 1차선 주행을 하고 싶은땐 주석 처리하면 됩니다.
        self.moveByLine()
        
class PIDController:
    def __init__(self, Kp=0.8, Ki=0.1, Kd=0.000001):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.prev_error = 0
        self.integral = 0
        self.last_time = time()

        self.limit_steer_degree = 0.2 * 100
        
    def compute(self, error):
        current_time = time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # dt가 너무 작거나 0일 경우 방지
        if dt <= 0.0:
            dt = 1e-6
        elif dt >= 2:
            dt = 2
            
        self.integral += error * dt
        if self.integral <= -self.limit_steer_degree:
            self.integral = -self.limit_steer_degree
        elif self.integral >= self.limit_steer_degree:
            self.integral = self.limit_steer_degree
            
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        print(f"self.integral {self.integral}, ")
        print(f"derivative {derivative}")
        print(f"error {error}")
        print(f"self.Kp * error {self.Kp * error}, self.Ki * self.integral {self.Ki * self.integral}, self.Kd * derivative {self.Kd * derivative}")
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return output
