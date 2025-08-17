#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import cv2 
import numpy as np

class LaneFeatureExtractor:
    def __init__(self):
        self.init_current_lane()

    def init_current_lane(self):
        self.current_lane     = "right"  

    def estimate_stop_line(self, white_bin_img, img_y , threshold=240):
        # y축 방향으로 sum → 수평선은 y축 기준으로 sum값이 커짐
        self.img_y = img_y
        horizontal_sum = np.sum(white_bin_img, axis=1) // 255  # shape: (height,)
        stop_line_indices = np.where(horizontal_sum > threshold)[0]
        # threshold 이상인 위치 찾기
        if len(stop_line_indices) > 0:
            min_y = stop_line_indices[0]
            max_y = stop_line_indices[-1]
            
            # 디텍팅된 구간을 검은색(0)으로 만듦
            white_bin_img[min_y:max_y+1, :] = 0

            return [int(min_y), int(max_y)], white_bin_img
        else:
            return [], white_bin_img

    def estimate_lane_mode(self, warped_img):
        img_hsv = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)

        yellow_lower = np.array([18, 140, 120]) 
        yellow_upper = np.array([34,255,255])
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
        # cv2.imshow("Yellow Mask", yellow_mask)
        # cv2.imshow("White Mask", white_mask)
        # cv2.waitKey(1)

        threshold = 9000
        # print(f"self.current_lane {self.current_lane}")
        # print(f"left_yellow_count {left_yellow_count}  right_white_count {right_white_count} ")
        if left_yellow_count > threshold:
            self.current_lane = "left"
            return "left"
        elif right_white_count > threshold:
            self.current_lane = "right"
            return "right"
        else:
            return self.current_lane  # 변화 없으면 유지
        
    def detect_out_rotary(self,yellow_bin_img):
        img = yellow_bin_img
        h, w = img.shape[:2]
        # 로타리 전용 ROI (필요 시 비율 조정)
        x1, x2 = int(0.68 * w), w
        y1, y2 = 0, int(0.42* h )
        roi = img[y1:y2, x1:x2]
        yellow_count = int(cv2.countNonZero(roi))
        return yellow_count > 0