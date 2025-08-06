#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy 
from morai_msgs.msg import GetTrafficLightStatus
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np

from obstacle_avoid.msg import PersonBBox  # 커스텀 메시지 임포트

# from time import *
# import time


class Traffic_control:
    def __init__(self):
        rospy.init_node("lane_sub_node")
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        
        rospy.Subscriber("/GetTrafficLightStatus",GetTrafficLightStatus, self.traffic_CB)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)

        # lidar_callback
        rospy.Subscriber("/lidar2D", LaserScan, self.lidar_CB)

        # yolo cb **
        '''
        yolo_detector.py부분에서 /dynamic_obstacle_detected 이름의 불값 토픽을 쏘고 있다. 
        '''
        rospy.Subscriber('/person_bbox', PersonBBox, self.dynamic_obs_CB)

        self.bridge = CvBridge()
        self.steer_msg = Float64()
        self.speed_msg = Float64()

        self.traffic_msg = GetTrafficLightStatus()

        self.traffic_flag = 0
        self.prev_signal = 0
        self.cross_flag = 0
        self.center_index = 0

        self.center_index=0
        self.degree_per_pixel=0
        self.standard_line=0

        self.signal = 0
        self.img = []

        self.x = 0
        self.y = 0
        self.warped_img = None  # 🔧 초기화 추가

        # LIDAR
        self.lidar_flag = False
        self.lidar_steer = 0.5
        self.lidar_speed = 800  

        self.current_lane = "right"         # 현재 내 차선 (기본은 2차선 도로 기준 오른쪽)
        self.avoid_side = None              # 회피 방향 ("left" or "right")
        self.in_avoid_mode = False          # 회피 중인지 여부


        # CAMERA
        self.waypoint_idx = -1
        self.last_time = rospy.get_time()  # ⬅️ 타이머용 시간 초기화
        self.sleep_duration = 0.0          # ⬅️ 각 단계의 대기시간 초기화


        self.obs_flag = False
        self.last_detected_time = rospy.get_time()

        self.dynamic_obs_flag = False       #는 "지금 사람이 보이나요?" <- traffic_CB에 의해 업데이트 




    def traffic_CB(self,msg):
        self.traffic_msg = msg

        if self.traffic_msg.trafficLightIndex == "SN000002":  # 🔧 오타 수정
            self.signal = self.traffic_msg.trafficLightStatus
            if self.prev_signal != self.signal:
                self.prev_signal = self.signal
                
            self.traffic_think()

    def dynamic_obs_CB(self, msg):
        # 바운딩 박스 중심 계산
        center_x = (msg.xmin + msg.xmax) / 2.0
        box_height = msg.ymax - msg.ymin

        # 카메라 프레임 기준 설정
        frame_width = 640   # 해상도 예시
        frame_height = 480  # 세로 해상도 예시
        margin = frame_width * 0.2 / 2.0
        center_min = frame_width / 2.0 - margin
        center_max = frame_width / 2.0 + margin

        # 🔴 회피 조건:
        # (1) 바운딩 박스가 중앙 20% 범위 안에 있거나
        # (2) 바운딩 박스 높이가 일정 이상 (가까이 있음)
        HEIGHT_THRESHOLD = 0.4 * frame_height  # 🔧 기준값: 전체 프레임의 40%

        if (center_min <= center_x <= center_max) or (box_height > HEIGHT_THRESHOLD):
            self.dynamic_obs_flag = True
            self.last_detected_time = rospy.get_time()
        else:
            self.dynamic_obs_flag = False

    def traffic_think(self):
        if self.signal == 1:
            #print("red")
            pass
        elif self.signal == 4:
            #print("yellow")
            pass
        elif self.signal == 16:
            #print("green")
            pass
        elif self.signal == 3:
            #print("left")
            pass
        else:
            pass

    def cam_CB(self,msg):
        # if self.lidar_flag:
        #     return  # 🔐 라이다 회피 중이면 이미지 무시
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.warped_img, self.center_index, self.standard_line, self.degree_per_pixel = self.cam_lane_detection(msg)

    def cam_lane_detection(self,msg):
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.y, self.x = self.img.shape[0:2]
        img_hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        h,s,v = cv2.split(img_hsv)

        yellow_lower = np.array([15,128,0])
        yellow_upper = np.array([40,255,255])
        yellow_range = cv2.inRange(img_hsv, yellow_lower, yellow_upper)
        white_lower = np.array([0,0,192])
        white_upper = np.array([179,64,255])
        white_range = cv2.inRange(img_hsv, white_lower, white_upper)
        combined_range = cv2.bitwise_or(yellow_range, white_range)
        filltered_img = cv2.bitwise_and(self.img, self.img, mask=combined_range)

        src_points = np.float32([
            [0,420],
            [275,260],
            [self.x-275,260],
            [self.x,420]
        ])

        dst_points = np.float32([
            [self.x//8,480],
            [self.x//8,0],
            [self.x//8*7,0],
            [self.x//8*7,480]
        ])

        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_img = cv2.warpPerspective(filltered_img, matrix, [self.x, self.y])
        grayed_img = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY) # filererd _. warped_img -> gray -> 50 이상,, 
        bin_img = np.zeros_like(grayed_img)
        bin_img[grayed_img > 50] = 1

        # 각 x열에 대한 흰색 픽셀 수
        # 3/4~끝 세로 영역으로 roi변경
        # * 추가
        bottom_bin = bin_img[self.y*3//4 : , :]  # y축 하단 1/4만 슬라이싱
        histogram_x = np.sum(bottom_bin, axis=0)  # 그 부분만 가지고 x축 히스토그램 계산
        histogram_y = np.sum(bottom_bin, axis=1)
        # histogram_x = np.sum(bin_img, axis=0)
        # histogram_y = np.sum(bin_img, axis=1)
        left_hist = histogram_x[0:self.x//2]
        right_hist = histogram_x[self.x//2:]
        #print(f"LEFT HIST {left_hist}")
        #print(f"RIGHT HIST{right_hist}")
        down_hist = histogram_y[self.y//4*3:]

        indices = np.where(histogram_x > 20)[0]
        left_indices = np.where(left_hist > 20)[0]
        right_indices = np.where(right_hist > 20)[0] + self.x//2
        cross_indices = np.where(down_hist > 450)[0] + self.y//4*3

        center_index = self.x // 2  # 🔧 기본값 설정

        LEFT_MIN_PIXELS = 10   # 적절히 조정
        RIGHT_MIN_PIXELS = 10


        try:
            cross_threshold = 25
            cross_diff = cross_indices[-1] - cross_indices[0]
            if cross_threshold < cross_diff:
                self.cross_flag = True
                cv2.rectangle(warped_img, [0, cross_indices[0]], [self.x, cross_indices[-1]], [0,255,0], 3)
            else:
                self.cross_flag = False
        except:
            self.cross_flag = False

        left_count = len(left_indices)
        right_count = len(right_indices)
        #print(f"left_indice {left_indices}")
        #print(f"right_indice {right_indices}")

        try:
            if left_count > LEFT_MIN_PIXELS and right_count > RIGHT_MIN_PIXELS:
                #print(f"L1{left_indices[0]} L2{left_indices[1]} R1{right_indices[0]} R2{right_indices[-1]}")
                center_index = (left_indices[0] + right_indices[-1]) // 2
                #print("both_line")

            elif right_count > left_count:
                center_index = ((right_indices[0] + right_indices[-1]) // 2) - 160
                #print(f"{right_indices[0]} {right_indices[-1]}")
                #print(f"right_line_only {time()}")

            elif left_count > right_count:
                center_index = ((left_indices[0] + left_indices[-1]) // 2) + 160
                #print("left_line_only")

            else:
                center_index = self.standard_line
                #print("no_line_detected")

        except:
            center_index = self.standard_line
            print("line_detection_exception")

        standard_line = self.x // 2
        degree_per_pixel = 1 / self.x

        return warped_img, center_index, standard_line, degree_per_pixel



    def action(self):
        current_time = rospy.get_time()

        # <0> yolo로 사람이 인식되면 그에 맞는 회피를 함. - 동적 장애물 
        DYNAMIC_OBS_TIMEOUT = 2.0  
        # 사람이 사라진 후 기다리는 시간 (초)
        # 최근에 사람을 인식 (사람이 사라진 직후에도) ->  일정 시간 동안은 여전히 회피 상태를 유지" 하도록 하기 위함./YOLO는 프레임마다 추론하므로, 가끔 프레임 드랍을 고려,,
        #  그래서 or이랑, 조건문 초기에 위치시킴, 
        if self.dynamic_obs_flag or (current_time - self.last_detected_time < DYNAMIC_OBS_TIMEOUT):
            print("🚷 동적 장애물 감지 → 정지 대기 중")
            steer = 0.5
            speed = 0
            self.lidar_flag = False  # 정적 장애물 회피 일시 중지
            self.obs_flag = True     # 동적 장애물 대기 상태 진입
            self.waypoint_idx = -1   # 회피 루틴 초기화

            # 동적 장애물 회피 시나리오 
            # 1. yolo로 사람이 잡힘 
            # -> 사람의 바운딩 박스의 중심이, 내 카메라 프레임 중심 기준 20퍼센트 안에 들어오면 정지, 이외에는 기존대로 주행임
            # -> 내 프레임 중심 기준 20퍼센트 이내 , 바운딩 박스가 벗어나면, 대기중이던 차량은 장애물이 없음을 인식하고 주행.  / DYNAMIC OBS CB에 박혀있음!


        # 1️⃣-2 LIDAR 회피 모드 - 정적 장애물
        elif self.lidar_flag or self.in_avoid_mode:
            print('⚠️ LIDAR 회피 중...')

            # 회피 시작 시점
            if not self.in_avoid_mode:
                print("회피 방향 결정 중...")
                if self.current_lane == "right":
                    self.avoid_side = "left"
                else:
                    self.avoid_side = "right"

                self.in_avoid_mode = True
                self.obs_flag = True
                self.waypoint_idx = 0
                self.last_time = current_time
                self.sleep_duration = 1.0

            # 다음 단계로 넘어갈 조건
            if current_time - self.last_time > self.sleep_duration:
                self.waypoint_idx += 1
                print(f"➡️ 회피 단계 {self.waypoint_idx}")
                self.last_time = current_time

            # 회피 동작 단계
            if self.waypoint_idx == 0:
                print(f"[0단계] {self.avoid_side} 방향으로 조향")
                steer = 0.1 if self.avoid_side == "left" else 0.9
                speed = 0
                self.sleep_duration = 1.0

            elif self.waypoint_idx == 1:
                print(f"[1단계] {self.avoid_side} 방향 직진")
                steer = 0.1 if self.avoid_side == "left" else 0.9
                speed = 800
                self.sleep_duration = 1.2

            elif self.waypoint_idx == 2:
                print("[2단계] 정지 후 반대 방향으로 조향")
                steer = 0.9 if self.avoid_side == "left" else 0.1
                speed = 600
                self.sleep_duration = 1.0

            elif self.waypoint_idx == 3:
                print("[3단계] 차선 따라 직진 복귀")
                steer = 0.5
                speed = 800
                self.sleep_duration = 1.5

            elif self.waypoint_idx == 4:
                # 상태 초기화
                self.in_avoid_mode = False
                self.lidar_flag = False
                self.obs_flag = False
                self.waypoint_idx = -1
                self.sleep_duration = 0.0
                # 차선 상태 업데이트
                self.current_lane = self.avoid_side
                print(f"✅ 회피 완료! 차선변경 완료 {self.current_lane}")
                self.avoid_side = None
                steer = 0.5
                speed = 800

            else:
                # 안전 장치 (정의되지 않은 단계)
                print("🚨 회피 단계 초과 → 리셋")
                self.in_avoid_mode = False
                self.lidar_flag = False
                self.obs_flag = False
                self.waypoint_idx = -1
                steer = 0.5
                speed = 0

        
        
        # 2️⃣ 일반 주행 (lane follow) - 차선 인식 주행
        elif isinstance(self.img, np.ndarray) and isinstance(self.warped_img, np.ndarray):
            error = self.center_index - self.standard_line
            steer = 0.5 + 3 * error * self.degree_per_pixel # 3은 가중치 
            steer = max(0.0, min(1.0, steer))
            speed = 800



        # 3️⃣ 예외 처리
        else:
            print("🚫 이미지 없음 또는 조건 불충족 → 정지")
            steer = 0.5
            speed = 0

        # ===============================>

        # PUB
        self.steer_msg.data = steer
        self.speed_msg.data = speed
        self.speed_pub.publish(self.speed_msg)
        self.steer_pub.publish(self.steer_msg)

        # 시각화
        if isinstance(self.img, np.ndarray):
            cv2.imshow("img", self.img)
        if isinstance(self.warped_img, np.ndarray):
            cv2.imshow("warped_img", self.warped_img)
        cv2.waitKey(1)


    def lidar_CB(self, msg):
            ranges = msg.ranges
            center_idx = len(ranges) // 2
            forward_ranges = ranges[center_idx - 15 : center_idx + 15]
            filtered = [r for r in forward_ranges if 0.0 < r <= 1.5]

            if len(filtered) > 0:
                min_dist = np.mean(filtered)
                print(f"전방 {min_dist:.2f}m에 장애물 감지!")

                if min_dist < 0.7:
                    self.lidar_flag = True
                    self.lidar_steer = 0.1
                    self.lidar_speed = 0
                elif min_dist < 2.0:
                    self.lidar_flag = True
                    self.lidar_steer = 0.1
                    self.lidar_speed = 100
                else:
                    self.lidar_flag = False
            else:
                print("✅ 장애물 없음 → 정상 주행 가능")
                self.lidar_flag = False


def main():
    try:
        traffic_control = Traffic_control()
        while not rospy.is_shutdown():
            traffic_control.action()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
