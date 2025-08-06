        
#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
from time import *

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
        # print(f"self.integral {self.integral}, ")
        # print(f"derivative {derivative}")
        # print(f"error {error}")
        # print(f"self.Kp * error {self.Kp * error}, self.Ki * self.integral {self.Ki * self.integral}, self.Kd * derivative {self.Kd * derivative}")
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return output
