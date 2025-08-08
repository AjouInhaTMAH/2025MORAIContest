#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
카메라 기반 차선 추종 + LIDAR 정적장애물 회피 + YOLO 동적장애물 정지 + 
'두꺼운 가로선' 인식 시 일정 시간 직진(steer=0.5) 유지.

⚠️ 로직은 일절 건드리지 않고, 주석만 정리/보강했습니다.
"""

import rospy 
from morai_msgs.msg import GetTrafficLightStatus
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
from std_msgs.msg import Int32

from obstacle_avoid.msg import PersonBBox  # YOLO에서 검출된 사람 bbox 커스텀 메시지

import subprocess


class Traffic_control:
    def __init__(self):
        rospy.init_node("lane_sub_node")

        # =============================
        # Publishers
        # =============================
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed",   Float64, queue_size=1)
        
        # =============================
        # Subscribers
        # =============================
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.traffic_CB)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage,       self.cam_CB)
        rospy.Subscriber("/lane_mode",             Int32,                 self.car_nav_CB)
        rospy.Subscriber("/lidar2D",               LaserScan,             self.lidar_CB)
        rospy.Subscriber("/person_bbox",           PersonBBox,            self.dynamic_obs_CB)

        # =============================
        # 상태 / 모드 변수
        # =============================
        self.lane_mode      = 0     # 0: 기본, 1/2/3/4: 특정 ROI/직진 강제 등 (외부 네비 노드가 결정)
        self.lidar_flag     = False # 라이다로 정적 장애물 감지 시 True
        self.current_lane   = "right" # 현재 차선(우측 차선 기준), 회피 방향 결정용
        self.avoid_side     = None
        self.in_avoid_mode  = False  # 회피 단계 진행 중 여부
        self.dynamic_obs_flag = False # YOLO로 사람 검출(회피/정지 트리거)
        self.obs_flag       = False

        # 회피 단계 진행용 타이밍 변수
        self.waypoint_idx   = -1
        self.last_time      = rospy.get_time()
        self.sleep_duration = 0.0
        self.last_detected_time = rospy.get_time()

        # =============================
        # 카메라/기하 정보
        # =============================
        self.center_index     = 0
        self.degree_per_pixel = 0
        self.standard_line    = 0

        self.bridge    = CvBridge()
        self.steer_msg = Float64()
        self.speed_msg = Float64()

        # =============================
        # 신호등 상태
        # =============================
        self.traffic_msg  = GetTrafficLightStatus()
        self.traffic_flag = 0
        self.prev_signal  = 0
        self.signal       = 0
        self.cross_flag   = 0

        # =============================
        # 이미지 버퍼
        # =============================
        self.img        = []
        self.x          = 0
        self.y          = 0
        self.warped_img = None

        # =============================
        # 두꺼운 가로선 감지/유지 설정
        #  - 하단 ROI에서 가로로 넓은 흰색 픽셀이 연속해서 나타나면 "두꺼운 선"으로 판단
        #  - 감지 시 일정 시간 steer=0.5 유지하고, 재트리거를 막기 위한 cooldown 적용
        # =============================
        self.thick_hold_active     = False
        self.thick_hold_until      = 0.0
        self.thick_hold_duration   = rospy.get_param("~thick_hold_duration", 3.0)  # 유지 시간(s)
        self.thick_cooldownspy.init_node("lane_sub_node")

        # =============================
        # Publishers
        # =============================
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed",   Float64, queue_size=1)
        
        # =============================
        # Subscribers
        # =============================
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.traffic_CB)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage,       self.cam_CB)
        rospy.Subscriber("/lane_mode",             Int32,                 self.car_nav_CB)
        rospy.Subscriber("/lidar2D",               LaserScan,             self.lidar_CB)
        rospy.Subscriber("/person_bbox",           PersonBBox,            self.dynamic_obs_CB)

        # =============================
        # 상태 / 모드 변수
        # =============================
        self.lane_mode      = 0     # 0: 기본, 1/2/3/4: 특정 ROI/직진 강제 등 (외부 네비 노드가 결정)
        self.lidar_flag     = False # 라_until  = 0.0
        self.thick_cooldown        = rospy.get_param("~thick_cooldown", 2.0)       # 재트리거 방지(s)
        self.thick_count           = 0  # 감지 횟수 카운트(선택적 로깅/미션 조건 등에 사용 가능)

    # =============================
    # 콜백: 외부 네비게이션이 주는 차선 모드
    # =============================
    def car_nav_CB(self, msg):
        self.lane_mode = msg.data
    
    # =============================
    # 콜백: 신호등
    # =============================
    def traffic_CB(self, msg):
        self.traffic_msg = msg

        # 특정 신호등 ID만 처리
        if self.traffic_msg.trafficLightIndex == "SN000002":
            self.signal = self.traffic_msg.trafficLightStatus
            if self.prev_signal != self.signal:
                self.prev_signal = self.signal
            self.traffic_think()

    def traffic_think(self):
        # 실제 신호별 동작은 추후 필요 시 구현
        if   self.signal == 1:  # red
            pass
        elif self.signal == 4:  # yellow
            pass
        elif self.signal == 16: # green
            pass
        elif self.signal == 3:  # left
            pass
        else:
            pass

    # =============================
    # 콜백: YOLO 사람 bbox (동적 장애물)
    #  - 카메라 중심부(가로폭 20%)에 들어오고 bbox 높이가 일정 이상이면 "가까운 전방"으로 판단해 정지
    # =============================
    def dynamic_obs_CB(self, msg):
        center_x   = (msg.xmin + msg.xmax) / 2.0
        box_height = msg.ymax - msg.ymin

        frame_width  = 640
        frame_height = 480
        margin       = frame_width * 0.2 / 2.0
        
        center_min = frame_width / 2.0 - margin
        center_max = frame_width / 2.0 + margin
        HEIGHT_THRESHOLD = 0.25 * frame_height  # "가까움" 기준

        if (center_min <= center_x <= center_max) and (box_height > HEIGHT_THRESHOLD):
            self.dynamic_obs_flag  = True
            self.last_detected_time = rospy.get_time()
        else:
            self.dynamic_obs_flag = False

    # =============================
    # 콜백: 카메라 프레임 수신
    #  - 이미지를 버드뷰로 워핑 → 바이너리화 → 히스토그램 기반으로 center_index 계산
    #  - 하단 ROI에서 '두꺼운 가로선' 감지도 함께 수행
    # =============================
    def cam_CB(self, msg):
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.warped_img, self.center_index, self.standard_line, self.degree_per_pixel = self.cam_lane_detection(msg)

    def cam_lane_detection(self, msg):
        # 원본 이미지/HSV
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.y, self.x = self.img.shape[0:2]
        img_hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

        # 노란선/흰선 마스크
        yellow_lower = np.array([15,128,0])
        yellow_upper = np.array([40,255,255])
        yellow_range = cv2.inRange(img_hsv, yellow_lower, yellow_upper)

        white_lower = np.array([0,0,192])
        white_upper = np.array([179,64,255])
        white_range = cv2.inRange(img_hsv, white_lower, white_upper)

        combined_range = cv2.bitwise_or(yellow_range, white_range)
        filltered_img  = cv2.bitwise_and(self.img, self.img, mask=combined_range)

        # 버드뷰 변환(워핑)
        src_points = np.float32([
            [0,420],
            [275,260],
            [self.x-275,260],
            [self.x,420]
        ])
        dst_points = np.float32([
            [self.x//8,   480],
            [self.x//8,     0],
            [self.x//8*7,   0],
            [self.x//8*7, 480]
        ])

        matrix     = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_img = cv2.warpPerspective(filltered_img, matrix, [self.x, self.y])

        # 흑백/바이너리
        grayed_img = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
        bin_img    = np.zeros_like(grayed_img)
        bin_img[grayed_img > 50] = 1

        # 하단 1/4 ROI만 사용해 히스토그램 계산(노이즈 내성 ↑)
        bottom_bin   = bin_img[self.y*3//4:, :]
        histogram_x  = np.sum(bottom_bin, axis=0)
        histogram_y  = np.sum(bottom_bin, axis=1)

        left_hist    = histogram_x[0:self.x//2]
        right_hist   = histogram_x[self.x//2:]
        down_hist    = histogram_y

        # threshold 넘는 인덱스들
        indices       = np.where(histogram_x > 20)[0]
        left_indices  = np.where(left_hist  > 20)[0]
        right_indices = np.where(right_hist > 20)[0] + self.x//2
        cross_indices = np.where(down_hist  > 450)[0] + self.y//4*3  # 하단 ROI 내부 기준

        center_index = self.x // 2  # 기본값

        LEFT_MIN_PIXELS  = 10
        RIGHT_MIN_PIXELS = 10

        # 교차로(가로선 다수) 대충 감지: 하단 ROI에서 굵은 범위가 넓게 나오는지
        try:
            cross_threshold = 25
            cross_diff = cross_indices[-1] - cross_indices[0]
            if cross_threshold < cross_diff:
                self.cross_flag = True
                cv2.rectangle(
                    warped_img,
                    [0, cross_indices[0]],
                    [self.x, cross_indices[-1]],
                    [0,255,0],
                    3
                )
            else:
                self.cross_flag = False
        except:
            self.cross_flag = False

        left_count  = len(left_indices)
        right_count = len(right_indices)

        # lane_mode 에 따른 center_index 결정
        try:
            if self.lane_mode == 1:
                # 왼쪽 차선만 추종
                if left_count > LEFT_MIN_PIXELS:
                    center_index = ((left_indices[0] + left_indices[-1]) // 2) + 160
                    cv2.line(warped_img, (left_indices[0], self.y-1), (left_indices[0], self.y//2), (255, 0, 0), 2)
                    cv2.line(warped_img, (left_indices[-1], self.y-1), (left_indices[-1], self.y//2), (255, 0, 0), 2)
                    cv2.circle(warped_img, (center_index, self.y - 60), 10, (255, 0, 0), -1)

            elif self.lane_mode == 2:
                # (설정상 1과 동일 동작)
                if left_count > LEFT_MIN_PIXELS:
                    center_index = ((left_indices[0] + left_indices[-1]) // 2) + 160
                    cv2.line(warped_img, (left_indices[0], self.y-1), (left_indices[0], self.y//2), (255, 0, 0), 2)
                    cv2.line(warped_img, (left_indices[-1], self.y-1), (left_indices[-1], self.y//2), (255, 0, 0), 2)
                    cv2.circle(warped_img, (center_index, self.y - 60), 10, (255, 0, 0), -1)

            elif self.lane_mode == 3:
                # 오른쪽 차선만 추종
                if right_count > RIGHT_MIN_PIXELS:
                    center_index = ((right_indices[0] + right_indices[-1]) // 2) - 160
                    cv2.line(warped_img, (right_indices[0], self.y-1), (right_indices[0], self.y//2), (0, 0, 255), 2)
                    cv2.line(warped_img, (right_indices[-1], self.y-1), (right_indices[-1], self.y//2), (0, 0, 255), 2)
                    cv2.circle(warped_img, (center_index, self.y - 60), 10, (0, 0, 255), -1)

            elif self.lane_mode == 4:
                # 차선 무시, 강제 직진
                center_index = self.x // 2

            else:
                # 기본: 양쪽 차선 존재 시 중앙, 한쪽만 있으면 그쪽으로 보정
                if   left_count  > LEFT_MIN_PIXELS and right_count > RIGHT_MIN_PIXELS:
                    center_index = (left_indices[0] + right_indices[-1]) // 2
                elif right_count > left_count:
                    center_index = ((right_indices[0] + right_indices[-1]) // 2) - 160
                elif left_count  > right_count:
                    center_index = ((left_indices[0] + left_indices[-1]) // 2) + 160
                else:
                    center_index = self.standard_line
        except:
            center_index = self.standard_line
            print("❌ 차선 추종 예외 발생")

        standard_line    = self.x // 2
        degree_per_pixel = 1 / self.x

        # ---------------------------------------------------------
        # 두꺼운 가로선 감지:
        #  - 하단 1/4 ROI에서 각 '행'의 흰 픽셀 수(row_sum)를 보고
        #  - 폭의 55% 이상 흰 픽셀을 가진 행이 6줄 이상 연속하면 '두꺼운 선'으로 인식
        #  - 감지되면 일정 시간(hold_duration) steer=0.5로 고정
        # ---------------------------------------------------------
        rows_roi     = bin_img[self.y*3//4:, :]
        row_sum      = np.sum(rows_roi, axis=1)
        ROW_PIX_MIN  = int(0.55 * self.x)  # 한 행이 화면폭의 55% 이상 흰색
        MIN_RUN_ROWS = 6                   # 연속 6줄 이상이면 두꺼운 선

        run = 0
        max_run = 0
        for val in row_sum:
            if val >= ROW_PIX_MIN:
                run += 1
            else:
                if run > max_run:
                    max_run = run
                run = 0
        if run > max_run:
            max_run = run

        now = rospy.get_time()
        if (max_run >= MIN_RUN_ROWS) and (now > self.thick_cooldown_until):
            # hold + cooldown 설정
            self.thick_hold_active   = True
            self.thick_hold_until    = now + self.thick_hold_duration
            self.thick_cooldown_until= now + self.thick_cooldown
            self.thick_count        += 1

            rospy.logwarn(f"[THICK] detected: run={max_run}, hold={self.thick_hold_duration}s (count={self.thick_count})")
            cv2.putText(warped_img, "THICK-LINE DETECTED -> HOLD",
                        (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

        return warped_img, center_index, standard_line, degree_per_pixel
    
    # =============================
    # 현재 차선(좌/우) 추정: 워프 이미지에서 색 마스크 카운트 비교
    # =============================
    def estimate_current_lane(self, warped_img):
        img_hsv = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)
        
        yellow_lower = np.array([15,128,0])
        yellow_upper = np.array([40,255,255])
        yellow_mask  = cv2.inRange(img_hsv, yellow_lower, yellow_upper)

        white_lower = np.array([0,0,192])
        white_upper = np.array([179,64,255])
        white_mask  = cv2.inRange(img_hsv, white_lower, white_upper)

        height, width = yellow_mask.shape
        left_roi  = yellow_mask[:, :width//2]
        right_roi = white_mask[:,  width//2:]

        left_yellow_count  = cv2.countNonZero(left_roi)
        right_white_count  = cv2.countNonZero(right_roi)

        print(f"🟨 left yellow: {left_yellow_count}, ⬜ right white: {right_white_count}")

        threshold = 300
        if left_yellow_count > threshold and left_yellow_count > right_white_count:
            return "left"
        elif right_white_count > threshold:
            return "right"
        else:
            return self.current_lane  # 변화 없으면 유지

    # =============================
    # 메인 루프: 상태 우선순위에 따라 조향/속도 결정
    #  1) 동적 장애물(YOLO) → 정지
    #  2) LIDAR 회피 시나리오 → 단계적 조향/주행
    #  3) 두꺼운 가로선 hold → steer=0.5 유지
    #  4) 일반 차선 추종
    #  5) 예외 → 정지
    # =============================
    def action(self):
        current_time = rospy.get_time()

        # 1) 동적 장애물: 정지 유지
        if self.dynamic_obs_flag:
            print("🚷 동적 장애물 감지 → 정지 대기 중")
            steer = 0.5
            speed = 0
            self.lidar_flag = False
            self.obs_flag   = True
            self.waypoint_idx = -1

        # 2) 정적 장애물 회피(LIDAR)
        elif self.lidar_flag or self.in_avoid_mode:
            self.current_lane = self.estimate_current_lane(self.warped_img)
            print(f"+++++++++++++++++{self.current_lane}++++++++++++++++++++++++++++")

            # 회피 시작 시 초기화
            if not self.in_avoid_mode:
                print("회피 방향 결정 중...")
                self.avoid_side   = "left" if self.current_lane == "right" else "right"
                self.in_avoid_mode = True
                self.obs_flag      = True
                self.waypoint_idx  = 0
                self.last_time     = current_time
                self.sleep_duration= 1.0

            # 단계 진행 타이밍
            if current_time - self.last_time > self.sleep_duration:
                self.waypoint_idx += 1
                self.last_time = current_time
            
            # 단계별 동작 (로직 그대로)
            if   self.waypoint_idx == 0:
                steer = 0.1 if self.avoid_side == "left" else 0.9
                speed = 0
                self.sleep_duration = 1.5

            elif self.waypoint_idx == 1:
                steer = 0.1 if self.avoid_side == "left" else 0.9
                speed = 1000
                self.sleep_duration = 0.5

            elif self.waypoint_idx == 2:
                steer = 0.8 if self.avoid_side == "left" else 0.2
                speed = 800
                self.sleep_duration = 0.8

            elif self.waypoint_idx == 3:
                steer = 0.5
                speed = 800
                self.sleep_duration = 0.5

            elif self.waypoint_idx == 4:
                self.in_avoid_mode = False
                self.lidar_flag    = False
                self.obs_flag      = False
                self.waypoint_idx  = -1
                self.sleep_duration= 0.0
                self.avoid_side    = None
                steer = 0.5
                speed = 800

            else:
                # 안전장치: 비정상 단계면 리셋
                print("🚨 회피 단계 초과 → 리셋")
                self.in_avoid_mode = False
                self.lidar_flag    = False
                self.obs_flag      = False
                self.waypoint_idx  = -1
                steer = 0.5
                speed = 0

        # 3) 두꺼운 가로선 HOLD 구간: 직진 유지
        elif self.thick_hold_active and rospy.get_time() < self.thick_hold_until:
            steer = 0.5
            speed = 1000
            rospy.loginfo_throttle(0.5, "[THICK] holding straight (steer=0.5)")

        # HOLD 종료 시점: 플래그 해제
        elif self.thick_hold_active and rospy.get_time() >= self.thick_hold_until:
            self.thick_hold_active = False
            steer = 0.5
            speed = 800
            rospy.loginfo("[THICK] hold released")
        
        # 4) 일반 차선 추종
        elif isinstance(self.img, np.ndarray) and isinstance(self.warped_img, np.ndarray):
            error = self.center_index - self.standard_line
            steer = 0.5 + 3.1 * error * self.degree_per_pixel  # 단순 비례 제어(원 코드 유지)
            steer = max(0.0, min(1.0, steer))
            speed = 1000

        # 5) 예외: 이미지/상태 부족 → 정지
        else:
            print("🚫 이미지 없음 또는 조건 불충족 → 정지")
            steer = 0.5
            speed = 0

        # 명령 퍼블리시
        self.steer_msg.data = steer
        self.speed_msg.data = speed
        self.speed_pub.publish(self.speed_msg)
        self.steer_pub.publish(self.steer_msg)

        # 디버그 시각화
        if isinstance(self.img, np.ndarray):
            cv2.imshow("img", self.img)
        if isinstance(self.warped_img, np.ndarray):
            cv2.imshow("warped_img", self.warped_img)
        cv2.waitKey(1)

    # =============================
    # 콜백: LIDAR
    #  - 정면 ±5빔 범위를 1.5m 이내 평균값으로 필터링
    #  - 가까우면 회피 플래그 on
    # =============================
    def lidar_CB(self, msg):
        if self.in_avoid_mode:
            # 회피 중에는 라이다 입력 무시
            return

        ranges = msg.ranges
        center_idx = len(ranges) // 2
        forward_ranges = ranges[center_idx - 5 : center_idx + 5]
        filtered = [r for r in forward_ranges if 0.0 < r <= 1.5]

        if len(filtered) > 0:
            min_dist = np.mean(filtered)
            print(f"📡 전방 {min_dist:.2f}m에 장애물 감지!")
            if min_dist < 1.5:
                self.lidar_flag = True
            else:
                self.lidar_flag = False
        else:
            self.lidar_flag = False


def main():
    node_name = '/throttle_interpolator'

    # (선택) 기존 노드 종료 시도 - 실패해도 무시
    try:
        subprocess.run(['rosnode', 'kill', node_name], check=True)
        print(f"✅ 노드 {node_name} 종료 성공")
    except subprocess.CalledProcessError:
        print(f"❌ 노드 {node_name} 종료 실패 (이미 종료되었거나 존재하지 않음)")

    # 메인 루프
    try:
        traffic_control = Traffic_control()
        while not rospy.is_shutdown():
            traffic_control.action()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()