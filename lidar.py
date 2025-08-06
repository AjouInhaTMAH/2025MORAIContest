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
# 0.35M
class lidar_class:
    def __init__(self):
        print(f"lidar_class start")
        rospy.init_node("lidar_listener_node", anonymous=True)
        self.laser_data = None
        rospy.Subscriber("/lidar2D", LaserScan, self.lidar_callback)
        self.pub_lidar = rospy.Publisher('/perception/lidar', String, queue_size=10)
        self.lane_length = 0.35 # 1개 차선 가로 길이
        self.init_ROI()
        
    def init_ROI(self):
        self.length_x_front = 0.35
        self.length_y_front = 1
        self.length_x_left = 0.35
        self.length_y_left = 1
        self.length_x_right = 0.35
        self.length_y_right = 1.5
        
    def lidar_callback(self, msg):
        self.laser_data = msg
      
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
        fx1 = int(cx - self.length_x_front / 2 * scale)
        fy1 = int(cy - self.length_y_front * scale)
        fx2 = int(cx + self.length_x_front / 2 * scale)
        fy2 = cy
        cv2.rectangle(img, (fx1, fy1), (fx2, fy2), (255, 255, 0), 1)  # Cyan

        # 좌측 ROI (왼쪽, x: -0.525~-0.175, y: 0~1.0 → y=앞, x=왼쪽)
        lx1 = int(cx + (-0.525) * scale)
        ly1 = int(cy - self.length_y_left * scale)
        lx2 = int(cx + (-0.175) * scale)
        ly2 = cy
        cv2.rectangle(img, (lx1, ly1), (lx2, ly2), (0, 255, 255), 1)  # Yellow

        # 우측 ROI (오른쪽, x: 0.175~0.525, y: 0~1.0 → y=앞, x=오른쪽)
        rx1 = int(cx + 0.175 * scale)
        ry1 = int(cy - self.length_y_right * scale)
        rx2 = int(cx + 0.525 * scale)
        ry2 = cy
        cv2.rectangle(img, (rx1, ry1), (rx2, ry2), (255, 0, 255), 1)  # Magenta
    
        # 로봇 위치 표시
        cv2.circle(img, (cx, cy), 3, (0, 0, 255), -1)
        cv2.imshow("Lidar View", img)
        cv2.waitKey(1)

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
        mask_front = (-1.0 <= x) & (x <= 0) & (np.abs(y) <= 0.175)
        front_pts = points_np[mask_front]

        # 좌측
        mask_left = (-1.0 <= x) & (x <= 0) & (-0.525<= y) & (y <= -0.175)
        left_pts = points_np[mask_left]

        # 우측
        mask_right = (-1.5 <= x) & (x <= 0) & (0.175 <= y) & (y <= 0.525)
        right_pts = points_np[mask_right]

        return front_pts, left_pts, right_pts
        # right 구역
    def decide_obstacle(self,points):
        results = []
        for region in points:
            # np.array일 경우 len(region) == 행 개수
            is_obstacle = len(region) >= 5
            results.append(is_obstacle)
        return results
    
    def processing(self):
        if self.laser_data is not None:
            start = time()
            # 모든 점을 현재 위치를 기준이 0,0 이라고 할때, x,y값으로 출력하도록 하기, 앞이 x+, 오른쪽이 y + 방향이다.
            points = self.calculate_xy_coordinates()
            front_pts, left_pts, right_pts = self.divide_ROI(points)
            # print(f"left_pts : {left_pts}")
            # print(f"front_pts : {front_pts}")
            # print(f"right_pts : {right_pts}")
            # print(f"left_pts : {len(left_pts)}")
            # print(f"front_pts : {len(front_pts)}")
            # print(f"right_pts : {len(right_pts)}")
            
            obstacles = self.decide_obstacle((left_pts,front_pts,right_pts))
            print(f"obstacles : {obstacles}")
            json_str = json.dumps(obstacles)
            self.pub_lidar.publish(json_str)
            
            # 출력: 앞이 x+, 오른쪽이 y+
            print(f"end time {time() - start}")
            
            # self.view_obstacle(points)
            self.view_obstacle_with_ROI(points)
            self.laser_data = None
        rospy.sleep(0.1)  # 너무 빠르게 돌지 않게 sleep
        
def main():
    try:
        lidar = lidar_class()
        while not rospy.is_shutdown():
           lidar.processing()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()