#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
카메라 기반 차선 추종 + LIDAR 정적장애물 회피 + YOLO 동적장애물 정지 +
두꺼운 가로선(THICK) 카운트별 액션 스케줄.
"""

import rospy
from morai_msgs.msg import GetTrafficLightStatus
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Float64, Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
from obstacle_avoid.msg import PersonBBox
import subprocess


class Traffic_control:
    def __init__(self):
        rospy.init_node("lane_sub_node")

        # ------------------ PUB ------------------
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed",   Float64, queue_size=1)

        # ------------------ SUB ------------------
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.traffic_CB)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage,       self.cam_CB)
        rospy.Subscriber("/lane_mode",             Int32,                 self.car_nav_CB)
        rospy.Subscriber("/lidar2D",               LaserScan,             self.lidar_CB)
        rospy.Subscriber("/person_bbox",           PersonBBox,            self.dynamic_obs_CB)

        # for line 5 
        

        # ---------------- 상태/모드 ----------------
        self.lane_mode        = 0
        self.lidar_flag       = False
        self.current_lane     = "right"
        self.avoid_side       = None
        self.in_avoid_mode    = False
        self.dynamic_obs_flag = False
        self.obs_flag         = False

        # 회피 단계 타이밍
        self.waypoint_idx   = -1
        self.last_time      = rospy.get_time()
        self.sleep_duration = 0.0
        self.last_detected_time = rospy.get_time()

        # --------------- 카메라/기하 ---------------
        self.center_index     = 0
        self.degree_per_pixel = 0
        self.standard_line    = 0

        self.bridge    = CvBridge()
        self.steer_msg = Float64()
        self.speed_msg = Float64()

        # --------------- 신호등 상태 ---------------
        self.traffic_msg  = GetTrafficLightStatus()
        self.traffic_flag = 0
        self.prev_signal  = 0
        self.signal       = 0
        self.cross_flag   = 0

        # --------------- 이미지 버퍼 ---------------
        self.img        = []
        self.x          = 0
        self.y          = 0
        self.warped_img = None

        # --------- 두꺼운선(THICK) 스케줄 ----------
        # 카운트별 고정 액션 (원하는 대로 수정)
        # 1,2: steer=0.5, speed=800 유지(1.5s)
        # 4  : steer=0.5, speed=800 유지(1.5s)
        # 5  : steer=0.2, speed=800 유지(3.0s)
        # 6  : steer=0.8, speed=800 유지(2.0s)
        # 7  : steer=0.2, speed=600 유지(2.0s)
        # 8  : steer=0.2, speed=800 유지(3.0s)
        # 9  : steer=0.5, speed=800 유지(3.0s)
        self.thick_plan = {
            1: {"type":"steer_fixed", "steer":0.5, "speed":800, "duration":1.5},
            # 2: {"type":"steer_fixed", "steer":0.5, "speed":800, "duration":1.5},
            # 4: {"type":"steer_fixed", "steer":0.5, "speed":800, "duration":1.5},
            # 5: {"type":"steer_fixed", "steer":0.2, "speed":800, "duration":3.0}, # 왼쪽 
            # # 문제가 영역 5에는 두꺼운 선이 없다. -> /amcl_pose로 일정 영역에 왔을때 이 데이터를 쓰도록 수정해야 함.. 
            # # #5만 “두꺼운선 감지” 대신 “AMCL 위치 만족”으로 발동해 thick_count를 5로 올리고 plan[5]를 실행.
            # 6: {"type":"steer_fixed", "steer":0.8, "speed":800, "duration":2.0}, # 오른쪽
            # 7: {"type":"steer_fixed", "steer":0.2, "speed":600, "duration":2.0}, # 왼쪽
            # 8: {"type":"steer_fixed", "steer":0.2, "speed":800, "duration":3.0}, # 왼쪽
            # 9: {"type":"steer_fixed", "steer":0.5, "speed":800, "duration":3.0},
        }
        self.thick_count           = 0
        self.thick_cooldown        = rospy.get_param("~thick_cooldown", 2.0)  # 재트리거 방지
        self.thick_cooldown_until  = 0.0
        self.thick_action          = None   # {'type','steer','speed','until'}


         # --------------- 동적 장애물 상태 ---------------
         # __init__ 안 어디 적당히:
        self.frame_w = rospy.get_param("~frame_width", 640)
        self.frame_h = rospy.get_param("~frame_height", 480)
        self.center_margin_ratio = rospy.get_param("~center_margin_ratio", 0.20)  # 중앙폭 20%
        self.warn_height_ratio = rospy.get_param("~warn_height_ratio", 0.15)      # 15% 이상 감속
        self.stop_height_ratio = rospy.get_param("~stop_height_ratio", 0.20)      # 20% 이상 정지
        self.dynamic_obs_timeout = rospy.get_param("~dynamic_obs_timeout", 0.5)   # s
        self.dynamic_obs_flag = "none"  # 'none' | 'slow_flag' | 'stop_flag'
        self.dynamic_obs_timeout = rospy.get_param("~dynamic_obs_timeout", 0.5)  # [ADD] YOLO 타임아웃
        self.stop_hold_until = 0.0  # [ADD] 정지 유지(홀드) 타임스탬프



    # ---------- 콜백들 ----------
    def car_nav_CB(self, msg):
        self.lane_mode = msg.data

    def traffic_CB(self, msg):
        self.traffic_msg = msg
        if self.traffic_msg.trafficLightIndex == "SN000002":
            self.signal = self.traffic_msg.trafficLightStatus
            if self.prev_signal != self.signal:
                self.prev_signal = self.signal
            self.traffic_think()

    def traffic_think(self):
        # 필요 시 구현
        pass

    # def dynamic_obs_CB(self, msg):
    #     center_x   = (msg.xmin + msg.xmax) / 2.0
    #     box_height = msg.ymax - msg.ymin
    #     frame_width, frame_height = 640, 480
    #     margin = frame_width * 0.2 / 2.0
    #     center_min = frame_width/2.0 - margin
    #     center_max = frame_width/2.0 + margin
    #     HEIGHT_THRESHOLD = 0.25 * frame_height
    #     if (center_min <= center_x <= center_max) and (box_height > HEIGHT_THRESHOLD):
    #         self.dynamic_obs_flag   = True
    #         self.last_detected_time = rospy.get_time()
    #     else:
    #         self.dynamic_obs_flag = False
    def dynamic_obs_CB(self, msg):
        center_x   = (msg.xmin + msg.xmax) / 2.0
        box_height = (msg.ymax - msg.ymin)

        # 중앙 영역 유지 (비율 기반)
        margin_half = self.frame_w * self.center_margin_ratio * 0.5
        center_min  = self.frame_w/2.0 - margin_half
        center_max  = self.frame_w/2.0 + margin_half
        in_center   = (center_min <= center_x <= center_max)

        h_ratio = box_height / float(self.frame_h)

        # ★ 순서: stop 먼저 → slow
        if in_center and h_ratio >= self.stop_height_ratio:
            self.dynamic_obs_flag = "stop_flag"
            self.last_detected_time = rospy.get_time()
        elif in_center and h_ratio >= self.warn_height_ratio:
            self.dynamic_obs_flag = "slow_flag"
            self.last_detected_time = rospy.get_time()
        else:
            self.dynamic_obs_flag = "none"
        print(self.dynamic_obs_flag)


    def cam_CB(self, msg):
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.warped_img, self.center_index, self.standard_line, self.degree_per_pixel = self.cam_lane_detection(msg)

    def cam_lane_detection(self, msg):
        # 원본 -> HSV
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.y, self.x = self.img.shape[0:2]
        img_hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

        # 노란/흰 차선 마스크
        yellow_range = cv2.inRange(img_hsv, np.array([15,128,0]),  np.array([40,255,255]))
        white_range  = cv2.inRange(img_hsv, np.array([0,0,192]),   np.array([179,64,255]))
        combined_range = cv2.bitwise_or(yellow_range, white_range)
        filltered_img  = cv2.bitwise_and(self.img, self.img, mask=combined_range)

        # 버드뷰 워핑
        src_points = np.float32([[0,420],[275,260],[self.x-275,260],[self.x,420]])
        dst_points = np.float32([[self.x//8,480],[self.x//8,0],[self.x//8*7,0],[self.x//8*7,480]])
        matrix     = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_img = cv2.warpPerspective(filltered_img, matrix, [self.x, self.y])

        # 바이너리
        grayed_img = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
        bin_img    = np.zeros_like(grayed_img)
        bin_img[grayed_img > 50] = 1

        # 하단 1/4 ROI 히스토그램
        bottom_bin  = bin_img[self.y*3//4:, :]
        histogram_x = np.sum(bottom_bin, axis=0)
        histogram_y = np.sum(bottom_bin, axis=1)

        left_hist    = histogram_x[0:self.x//2]
        right_hist   = histogram_x[self.x//2:]
        down_hist    = histogram_y

        left_indices  = np.where(left_hist  > 20)[0]
        right_indices = np.where(right_hist > 20)[0] + self.x//2
        # (참고) down_hist는 self.y*3//4 기준이므로 +오프셋 하고 싶으면 추가

        center_index = self.x // 2
        LEFT_MIN_PIXELS, RIGHT_MIN_PIXELS = 10, 10

        # lane_mode에 따른 center 결정
        try:
            if self.lane_mode == 1:
                if len(left_indices) > LEFT_MIN_PIXELS:
                    center_index = ((left_indices[0] + left_indices[-1]) // 2) + 160
            elif self.lane_mode == 2:
                if len(left_indices) > LEFT_MIN_PIXELS:
                    center_index = ((left_indices[0] + left_indices[-1]) // 2) + 160
            elif self.lane_mode == 3:
                if len(right_indices) > RIGHT_MIN_PIXELS:
                    center_index = ((right_indices[0] + right_indices[-1]) // 2) - 160
            elif self.lane_mode == 4:
                center_index = self.x // 2
            else:
                if len(left_indices) > LEFT_MIN_PIXELS and len(right_indices) > RIGHT_MIN_PIXELS:
                    center_index = (left_indices[0] + right_indices[-1]) // 2
                elif len(right_indices) > len(left_indices):
                    center_index = ((right_indices[0] + right_indices[-1]) // 2) - 160
                elif len(left_indices)  > len(right_indices):
                    center_index = ((left_indices[0] + left_indices[-1]) // 2) + 160
                else:
                    center_index = self.standard_line
        except Exception as e:
            center_index = self.standard_line
            rospy.logwarn(f"lane calc exception: {e}")

        standard_line    = self.x // 2
        degree_per_pixel = 1 / self.x

        # -------- 두꺼운선 감지 (하단 1/4 ROI 행 기준) --------
        rows_roi = bin_img[self.y*3//4:, :]
        row_sum  = np.sum(rows_roi, axis=1)
        ROW_PIX_MIN  = int(0.55 * self.x)  # 한 행이 화면폭의 55% 이상 흰색
        MIN_RUN_ROWS = 6                   # 연속 6줄 이상이면 두꺼운선
        run, max_run = 0, 0
        for val in row_sum:
            if val >= ROW_PIX_MIN:
                run += 1
            else:
                if run > max_run: max_run = run
                run = 0
        if run > max_run: max_run = run
        
        # 두꺼운 선에 따른 동작
        now = rospy.get_time()
        if (max_run >= MIN_RUN_ROWS) and (now > self.thick_cooldown_until):
            # self.thick_count += 1                       # 리셋 없음 (요청사항)
            self.thick_count = 1
            self.thick_cooldown_until = now + self.thick_cooldown
            plan = self.thick_plan.get(self.thick_count) # ** 
            if plan:
                self.thick_action = {
                    "type":  plan["type"],
                    "steer": plan["steer"],
                    "speed": plan["speed"],
                    "until": now + plan["duration"]
                }
            rospy.logwarn(f"[THICK] detected run={max_run} -> count={self.thick_count} plan={self.thick_action}")
            cv2.putText(warped_img, f"THICK COUNT={self.thick_count}",
                        (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

        return warped_img, center_index, standard_line, degree_per_pixel

    # def estimate_current_lane(self, warped_img):
    #     # 안전가드: 이미지 없으면 기존값 유지
    #     if not isinstance(warped_img, np.ndarray) or warped_img.size == 0:
    #         return self.current_lane

    #     img_hsv = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)
    #     yellow_mask = cv2.inRange(img_hsv, np.array([15,128,0]),  np.array([40,255,255]))
    #     white_mask  = cv2.inRange(img_hsv, np.array([0,0,192]),   np.array([179,64,255]))

    #     h, w = yellow_mask.shape
    #     left_yellow_count = cv2.countNonZero(yellow_mask[:, :w//2])
    #     right_white_count = cv2.countNonZero(white_mask[:,  w//2:])

    #     threshold = 300
    #     if left_yellow_count > threshold and left_yellow_count > right_white_count:
    #         return "left"
    #     elif right_white_count > threshold:
    #         return "right"
    #     else:
    #         return self.current_lane
        
        #     # 오른쪽 차선인데, 왼쪽으로 뽑히는 문제있음 ** 
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

            #print(f"🟨 left yellow: {left_yellow_count}, ⬜ right white: {right_white_count}")

            threshold = 300
            if left_yellow_count > threshold and left_yellow_count > right_white_count:
                return "left"
            elif right_white_count > threshold:
                return "right"
            else:
                return self.current_lane  # 변화 없으면 유지

    # ---------------- 메인 루프 ----------------
    def action(self):
        now = rospy.get_time()

        # [ADD] YOLO 감지 타임아웃으로 플래그 자동 해제
        if hasattr(self, "dynamic_obs_timeout"):
            if self.dynamic_obs_flag in ("slow_flag", "stop_flag") and (now - self.last_detected_time > self.dynamic_obs_timeout):
                self.dynamic_obs_flag = "none" if isinstance(self.dynamic_obs_flag, str) else False

        # [ADD] 정지 홀드(예: stop_flag 후 1초 유지). 최우선 처리 & 즉시 반환
        if now < getattr(self, "stop_hold_until", 0.0):
            steer, speed = 0.5, 0
            self.steer_msg.data = steer
            self.speed_msg.data = speed
            self.speed_pub.publish(self.speed_msg)
            self.steer_pub.publish(self.steer_msg)
            return

        # <-----------장애물 회피-------------->
        # 1) 동적 장애물 → 감속/정지 (최우선, 바로 publish & return)
        if self.dynamic_obs_flag == "slow_flag":
            steer, speed = 0.5, 300
            self.lidar_flag   = False
            self.obs_flag     = True
            self.waypoint_idx = -1
            # publish & return
            self.steer_msg.data = steer
            self.speed_msg.data = speed
            self.speed_pub.publish(self.speed_msg)   # [FIX] 실제 퍼블리시
            self.steer_pub.publish(self.steer_msg)
            return

        elif self.dynamic_obs_flag == "stop_flag":
            steer, speed = 0.5, 0
            self.lidar_flag   = False
            self.obs_flag     = True
            self.waypoint_idx = -1
            # [ADD] 1초 정지 유지 예약
            self.stop_hold_until = max(getattr(self, "stop_hold_until", 0.0), now + 1.0)
            # publish & return
            self.steer_msg.data = steer
            self.speed_msg.data = speed
            self.speed_pub.publish(self.speed_msg)   # [FIX] 실제 퍼블리시
            self.steer_pub.publish(self.steer_msg)
            return

        # 2) LIDAR 회피 (타이머 상태머신)  ← [MOVE] 타이머 증분은 이 분기 내부에서만!
        elif self.lidar_flag or self.in_avoid_mode:
            self.current_lane = self.estimate_current_lane(self.warped_img)
            if not self.in_avoid_mode:
                self.avoid_side    = "left" if self.current_lane == "right" else "right"
                self.in_avoid_mode = True
                self.obs_flag      = True
                self.waypoint_idx  = 0
                self.last_time     = now
                self.sleep_duration= 1.0

            # stop_line이 하드 코딩, slepp이 된 순간에 들어오면 
            # steer, speed부분이 적용이 안되는 문제 ,,? 
            # 스레드 개념인가 

            # [MOVE] 단계 전진 로직: LIDAR 분기 내부로 이동
            if now - self.last_time > self.sleep_duration:
                self.waypoint_idx += 1
                self.last_time = now

            if   self.waypoint_idx == 0:
                steer, speed = (0.1 if self.avoid_side == "left" else 0.9), 0
                self.sleep_duration = 1.0

            elif self.waypoint_idx == 1:
                steer, speed = (0.2 if self.avoid_side == "left" else 0.8), 1000
                self.sleep_duration = 0.9

            elif self.waypoint_idx == 2:
                steer, speed = (0.9 if self.avoid_side == "left" else 0.1), 800
                self.sleep_duration = 0.7

            # elif self.waypoint_idx == 3:
            #     steer, speed = (0.3 if self.avoid_side == "left" else 0.7), 800
            #     self.sleep_duration = 0.3

            elif self.waypoint_idx == 3:
                steer, speed = 0.5, 600
                self.sleep_duration = 0.5

            elif self.waypoint_idx == 4:
                self.in_avoid_mode = False
                self.lidar_flag = False
                self.obs_flag   = False
                self.waypoint_idx = -1
                self.sleep_duration= 0.0
                self.avoid_side   = None
                steer, speed = 0.5, 800

            else:
                self.in_avoid_mode = False
                self.lidar_flag = False
                self.obs_flag   = False
                self.waypoint_idx = -1
                steer, speed = 0.5, 0

        # <--------------차선 추종-------------->
        elif self.thick_action and now < self.thick_action["until"]:
            steer = float(self.thick_action["steer"])
            speed = float(self.thick_action["speed"])
            rospy.loginfo_throttle(0.5, f"[THICK] action steer={steer:.2f} speed={speed:.0f} left={self.thick_action['until']-now:.1f}s")

        elif self.thick_action and now >= self.thick_action["until"]:
            self.thick_action = None
            steer, speed = 0.5, 800
            rospy.loginfo("[THICK] action finished")

        elif isinstance(self.img, np.ndarray) and isinstance(self.warped_img, np.ndarray):
            error = self.center_index - self.standard_line
            steer = 0.5 + 3.1 * error * self.degree_per_pixel
            steer = max(0.0, min(1.0, steer))
            speed = 1000

        else:
            steer, speed = 0.5, 0

        # Publish (위에서 return하지 않은 경우만 공용 퍼블리시)
        self.steer_msg.data = steer
        self.speed_msg.data = speed
        self.speed_pub.publish(self.speed_msg)
        self.steer_pub.publish(self.steer_msg)

        # Debug view
        if isinstance(self.img, np.ndarray):
            cv2.imshow("img", self.img)
        if isinstance(self.warped_img, np.ndarray):
            cv2.imshow("warped_img", self.warped_img)
        cv2.waitKey(1)

    def lidar_CB(self, msg):
        if self.in_avoid_mode:
            return
        ranges = msg.ranges
        c = len(ranges) // 2
        forward = ranges[c-5:c+5]
        filt = [r for r in forward if 0.0 < r <= 1.5]
        if len(filt) > 0:
            dist = np.mean(filt)
            # if dist < 1.5:
            if dist < 1.0:
                self.lidar_flag = True
            else:
                self.lidar_flag = False
        else:
            self.lidar_flag = False


def main():
    node_name = '/throttle_interpolator'
    try:
        subprocess.run(['rosnode', 'kill', node_name], check=True)
        print(f"✅ 노드 {node_name} 종료 성공")
    except subprocess.CalledProcessError:
        print(f"❌ 노드 {node_name} 종료 실패 (이미 종료되었거나 존재하지 않음)")

    tc = Traffic_control()
    try:
        while not rospy.is_shutdown():
            tc.action()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()