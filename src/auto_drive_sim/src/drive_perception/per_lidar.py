#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import rospy
from std_msgs.msg import Float64
import numpy as np
import json
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from time import *
import cv2
from utills import check_timer
# 0.35M
class PerLidar:
    def __init__(self):
        print(f"PerLidar start")
        rospy.init_node('per_lidar_node')
        self.init_pubSub()
        self.init_msg()
        self.init_ROI()
        self.init_timer()

    def init_pubSub(self):
        rospy.Subscriber("/lidar2D", LaserScan, self.CB_lidar_raw)
        self.pub_lidar = rospy.Publisher('/perception/lidar', String, queue_size=10)
    def init_msg(self):
        self.laser_data = None
    def init_timer(self):
        self.check_timer = check_timer.CheckTimer("PerLidar")
    def init_ROI(self):
        self.length_x_front = -1
        self.length_y_front = 0.175
        
        self.length_x_front_near = -1
        
        self.length_x_left = -1
        self.length_y_left_min = -0.525
        self.length_y_left_max = -0.175
        
        self.length_x_right = -1.5
        self.length_y_right_min = 0.175
        self.length_y_right_max = 0.525
        
    def CB_lidar_raw(self, msg):
        self.laser_data = msg

    def pub_lidar_info(self,obstacles):
        json_str = json.dumps(obstacles)
        self.pub_lidar.publish(json_str)

    def calculate_xy_coordinates(self):
        angle = self.laser_data.angle_max  # 반대로 시작
        points = []

        for r in self.laser_data.ranges:
            if np.isinf(r) or np.isnan(r):
                angle += self.laser_data.angle_increment
                continue

            x = r * np.cos(angle)
            y = r * np.sin(angle)
            points.append((x, y))
            angle += self.laser_data.angle_increment
        return points
    def divide_ROI(self,points):
        points_np = np.array(points)  # (N, 2) 배열
        
        x = points_np[:, 0]
        y = points_np[:, 1]

        # 전방
        mask_front = (self.length_x_front <= x) & (x <= 0) & (np.abs(y) <= self.length_y_front)
        front_pts = points_np[mask_front]
        
        # 전방
        mask_front_near = (self.length_x_front_near <= x) & (x <= 0) & (np.abs(y) <= self.length_y_front)
        front_pts_near = points_np[mask_front_near]

        # 좌측
        mask_left = (self.length_x_left <= x) & (x <= 0) & (self.length_y_left_min <= y) & (y <= self.length_y_left_max)
        left_pts = points_np[mask_left]

        # 우측
        mask_right = (self.length_x_right <= x) & (x <= 0) & (self.length_y_right_min <= y) & (y <= self.length_y_right_max)
        right_pts = points_np[mask_right]

        return front_pts, left_pts, right_pts ,front_pts_near
        # right 구역
    def decide_obstacle(self,points):
        results = []
        for region in points:
            # np.array일 경우 len(region) == 행 개수
            is_obstacle = len(region) >= 5
            results.append(is_obstacle)
        return results

    def view_obstacle_with_ROI(self,points):
        # OpenCV 시각화
        img_size = 500  # 이미지 크기
        scale = 25      # m to pixel (1m = 40px)
        img = np.zeros((img_size, img_size, 3), dtype=np.uint8)
        cx, cy = img_size // 2, img_size // 2  # 중앙

        for x, y in points:
            px = int(cx + y * scale)     # y축 → 이미지 좌표계의 x
            py = int(cy + x * scale)     # x축 → 이미지 좌표계의 y (위가 +)
            # print(f"x, y {x} {y}")
            # print(f"x, y {py} {px}")
            if 0 <= px < img_size and 0 <= py < img_size:
                cv2.circle(img, (px, py), 1, (0, 255, 0), -1)

        # 전방 ROI (앞쪽, y: 0~1.0, x: -0.175~0.175)
        fx1 = int(cx - (self.length_x_front) / 2 * scale)
        fy1 = int(cy - (self.length_y_front) * scale)
        fx2 = int(cx + (self.length_x_front) / 2 * scale)
        fy2 = cy
        cv2.rectangle(img, (fx1, fy1), (fx2, fy2), (255, 255, 0), 1)  # Cyan

        # 좌측 ROI (왼쪽, x: -0.525~-0.175, y: 0~1.0 → y=앞, x=왼쪽)
        lx1 = int(cx + self.length_y_left_min * scale)
        ly1 = int(cy - self.length_x_left * scale)
        lx2 = int(cx + self.length_y_right_min * scale)
        ly2 = cy
        cv2.rectangle(img, (lx1, ly1), (lx2, ly2), (0, 255, 255), 1)  # Yellow

        # 우측 ROI (오른쪽, x: 0.175~0.525, y: 0~1.0 → y=앞, x=오른쪽)
        rx1 = int(cx + self.length_y_right_min * scale)
        ry1 = int(cy - self.length_x_right * scale)
        rx2 = int(cx + self.length_y_right_max * scale)
        ry2 = cy
        cv2.rectangle(img, (rx1, ry1), (rx2, ry2), (255, 0, 255), 1)  # Magenta
    
        # 로봇 위치 표시
        cv2.circle(img, (cx, cy), 3, (0, 0, 255), -1)
        cv2.imshow("Lidar View", img)
        cv2.waitKey(1)

    def processing(self):
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            if self.laser_data is not None:
                # self.check_timer.start()
                # 모든 점을 현재 위치를 기준이 0,0 이라고 할때, x,y값으로 출력하도록 하기, 앞이 x-, 오른쪽이 y + 방향이다.
                
                points = self.calculate_xy_coordinates()
                front_pts, left_pts, right_pts, front_pts_near = self.divide_ROI(points)
                obstacles = self.decide_obstacle((left_pts,front_pts,right_pts,front_pts_near))
                self.pub_lidar_info(obstacles)

                self.laser_data = None
                self.view_obstacle_with_ROI(points)
                # self.check_timer.check()
            rate.sleep()
            
if __name__ == '__main__':
    node = PerLidar()
    node.processing()   # spin 대신 processing 돌기