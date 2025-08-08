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

        # ---------------- 상태/모드 ----------------
        self.lane_mode        = 0
        self.lidar_flag       = False
        self.current_lane     = None
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

            #
            1: {"type":"steer_fixed", "steer":0.8, "speed":800, "duration":2.0}, # 오른쪽 c
            
            # 
            # 1: {"type":"steer_fixed", "steer":0.5, "speed":800, "duration":1.5},
            # 2: {"type":"steer_fixed", "steer":0.5, "speed":800, "duration":1.5},
            # 4: {"type":"steer_fixed", "steer":0.5, "speed":800, "duration":1.5},
            # 5: {"type":"steer_fixed", "steer":0.2, "speed":800, "duration":3.0}, # 왼쪽
            # 6: {"type":"steer_fixed", "steer":0.8, "speed":800, "duration":2.0}, # 오른쪽 c
            # 7: {"type":"steer_fixed", "steer":0.2, "speed":600, "duration":2.0}, # 왼쪽
            # 8: {"type":"steer_fixed", "steer":0.2, "speed":800, "duration":3.0}, # right
            # 9: {"type":"steer_fixed", "steer":0.5, "speed":800, "duration":3.0},
        }
        self.thick_count           = 0
        self.thick_cooldown        = rospy.get_param("~thick_cooldown", 2.0)  # 재트리거 방지
        self.thick_cooldown_until  = 0.0
        self.thick_action          = None   # {'type','steer','speed','until'}

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

    def dynamic_obs_CB(self, msg):
        center_x   = (msg.xmin + msg.xmax) / 2.0
        box_height = msg.ymax - msg.ymin
        frame_width, frame_height = 640, 480
        margin = frame_width * 0.2 / 2.0
        center_min = frame_width/2.0 - margin
        center_max = frame_width/2.0 + margin
        HEIGHT_THRESHOLD = 0.25 * frame_height
        if (center_min <= center_x <= center_max) and (box_height > HEIGHT_THRESHOLD):
            self.dynamic_obs_flag   = True
            self.last_detected_time = rospy.get_time()
        else:
            self.dynamic_obs_flag = False
#######################
    def cam_CB(self, msg):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg)
        except Exception as e:
            rospy.logwarn(f"[cam_CB] CvBridge error: {e}")
            self.img = None
            self.warped_img = None
            return

        if img is None:
            rospy.logwarn("[cam_CB] decoded img is None")
            self.img = None
            self.warped_img = None
            return

        self.img = img

        res = self.cam_lane_detection(img)   # ← msg 말고 img!
        if not res or len(res) != 4:
            rospy.logerr("[cam_CB] cam_lane_detection returned None/invalid")
            h, w = img.shape[:2]
            self.warped_img = None
            self.standard_line = w // 2
            self.center_index  = self.standard_line
            self.degree_per_pixel = 1.0 / max(w, 1)
            return

        self.warped_img, self.center_index, self.standard_line, self.degree_per_pixel = res

    def cam_lane_detection(self, img):
        try:
            if img is None:
                raise ValueError("img is None")

            # 🔁 여기서부터는 img만 사용 (self.img 금지)
            h, w = img.shape[:2]

            # --- 원본 -> HSV ---
            img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # --- 노란/흰 차선 마스크 ---
            yellow_range = cv2.inRange(img_hsv, np.array([15,128,0]),  np.array([40,255,255]))
            white_range  = cv2.inRange(img_hsv, np.array([0,0,192]),   np.array([179,64,255]))
            combined_range = cv2.bitwise_or(yellow_range, white_range)
            filtered       = cv2.bitwise_and(img, img, mask=combined_range)

            # --- 버드뷰 워핑 ---
            src_points = np.float32([[0,420],[275,260],[w-275,260],[w,420]])
            dst_points = np.float32([[w//8,480],[w//8,0],[w//8*7,0],[w//8*7,480]])
            M = cv2.getPerspectiveTransform(src_points, dst_points)
            warped = cv2.warpPerspective(filtered, M, (w, h))

            # --- 바이너리 ---
            gray    = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
            bin_img = np.zeros_like(gray, dtype=np.uint8)
            bin_img[gray > 50] = 1

            # --- 하단 1/4 ROI 히스토그램 ---
            bottom_bin  = bin_img[h*3//4:, :]
            histogram_x = np.sum(bottom_bin, axis=0)
            histogram_y = np.sum(bottom_bin, axis=1)

            left_hist    = histogram_x[:w//2]
            right_hist   = histogram_x[w//2:]

            left_indices  = np.where(left_hist  > 20)[0]
            right_indices = np.where(right_hist > 20)[0] + w//2

            # --- center 계산 ---
            standard_line = w // 2
            center_index  = standard_line
            LEFT_MIN_PIXELS, RIGHT_MIN_PIXELS = 10, 10

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
                    center_index = standard_line
                else:
                    if len(left_indices) > LEFT_MIN_PIXELS and len(right_indices) > RIGHT_MIN_PIXELS:
                        center_index = (left_indices[0] + right_indices[-1]) // 2
                    elif len(right_indices) > len(left_indices):
                        center_index = ((right_indices[0] + right_indices[-1]) // 2) - 160
                    elif len(left_indices)  > len(right_indices):
                        center_index = ((left_indices[0] + left_indices[-1]) // 2) + 160
                    else:
                        center_index = standard_line
            except Exception as e:
                rospy.logwarn(f"lane calc exception: {e}")
                center_index = standard_line

            degree_per_pixel = 1.0 / max(w, 1)

            # --- 두꺼운선(THICK) 감지 ---
            rows_roi = bin_img[h*3//4:, :]
            row_sum  = np.sum(rows_roi, axis=1)
            ROW_PIX_MIN  = int(0.55 * w)
            MIN_RUN_ROWS = 6
            run = max_run = 0
            for val in row_sum:
                if val >= ROW_PIX_MIN:
                    run += 1
                else:
                    if run > max_run: max_run = run
                    run = 0
            if run > max_run: max_run = run

            now = rospy.get_time()
            if (max_run >= MIN_RUN_ROWS) and (now > self.thick_cooldown_until):
                self.thick_count += 1
                self.thick_cooldown_until = now + self.thick_cooldown
                plan = self.thick_plan.get(self.thick_count)
                if plan:
                    self.thick_action = {
                        "type":  plan["type"],
                        "steer": plan["steer"],
                        "speed": plan["speed"],
                        "until": now + plan["duration"]
                    }
                rospy.logwarn(f"[THICK] detected run={max_run} -> count={self.thick_count} plan={self.thick_action}")
                cv2.putText(warped, f"THICK COUNT={self.thick_count}",
                            (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

            # ✅ 항상 4-튜플 반환
            return warped, int(center_index), int(standard_line), float(degree_per_pixel)

        except Exception as e:
            rospy.logerr(f"[cam_lane_detection] error: {e}")
            # 실패해도 4-튜플 반환
            if isinstance(img, np.ndarray):
                h, w = img.shape[:2]
            else:
                h, w = 480, 640
            standard_line = w // 2
            center_index  = standard_line
            degree_per_pixel = 1.0 / max(w, 1)
            return None, center_index, standard_line, degree_per_pixel


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
        
        # <-----------장애물 회피-------------->
        # 1) 동적 장애물 → 정지
        if self.dynamic_obs_flag:
            steer, speed = 0.5, 0
            self.lidar_flag = False
            self.obs_flag   = True
            self.waypoint_idx = -1

        # 2) LIDAR 회피
        elif self.lidar_flag or self.in_avoid_mode:
            self.current_lane = self.estimate_current_lane(self.warped_img)
            print(f"current lane {self.current_lane}")
            if not self.in_avoid_mode:
                self.avoid_side    = "left" if self.current_lane == "right" else "right"
                print(f"avoid lane {self.avoid_side}")
                self.in_avoid_mode = True
                self.obs_flag      = True
                self.waypoint_idx  = 0
                self.last_time     = now
                self.sleep_duration= 1.0

            if now - self.last_time > self.sleep_duration:
                self.waypoint_idx += 1
                self.last_time = now

            if   self.waypoint_idx == 0:
                steer, speed = (0.4 if self.avoid_side == "left" else 0.6), 0
                self.sleep_duration = 1.0
            elif self.waypoint_idx == 1: ##
                steer, speed = (0.2 if self.avoid_side == "left" else 0.8), 900
                self.sleep_duration = 1.0
            elif self.waypoint_idx == 2:
                steer, speed = (0.9 if self.avoid_side == "left" else 0.1), 700
                self.sleep_duration = 0.7
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
        # 3) 두꺼운선 스케줄 액션
        # **
        elif self.thick_action and now < self.thick_action["until"]:
            steer = float(self.thick_action["steer"])
            speed = float(self.thick_action["speed"])
            rospy.loginfo_throttle(0.5, f"[THICK] action steer={steer:.2f} speed={speed:.0f} left={self.thick_action['until']-now:.1f}s") # 시간쪽,, 

        # 3') 액션 종료
        elif self.thick_action and now >= self.thick_action["until"]:
            self.thick_action = None
            steer, speed = 0.5, 800
            rospy.loginfo("[THICK] action finished")

       
        # 4) 일반 차선 추종
        elif isinstance(self.img, np.ndarray) and isinstance(self.warped_img, np.ndarray):
            error = self.center_index - self.standard_line
            steer = 0.5 + 3.1 * error * self.degree_per_pixel
            steer = max(0.0, min(1.0, steer))
            speed = 1000
        
        # <--------------예외 -------------->
        # 5) 예외
        else:
            # rospy.logwarn("🚫 이미지 없음 또는 조건 불충족 → 정지")
            steer, speed = 0.5, 0

        # Publish
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