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
class SlidingWindow:
    def __init__(self):
        self.init_sliding()
    def init_sliding(self):
        self.midpoint = None
        self.is_left_lane = True
        self.img_y = 0
        
        self.window_height = 0
        self.nwindows = 10
        self.nonzero = None
        self.nonzeroy = None
        self.nonzerox = None
        self.left_blocked_flag = 0
        self.right_blocked_flag = 0
        self.alpha = 1
        
        self.left_lane_start = None
        self.left_lane_end = None
        self.right_lane_start = None
        self.right_lane_end = None
        
        self.left_lanes =[]
        self.right_lanes =[]

        self.stop_flag = False
        self.min_sep = 60

    def set_img_y(self,y):
        self.img_y = y

    def sliding_window_lane_calculation(self, x_current, y_current, prev_margin, x_prev,
                                        blocked_flag, binary_img, is_left_lane,
                                        other_x = None, min_sep = 0,color=(0, 255, 0)):

        flag = blocked_flag

        win_y_low = y_current - self.window_height // 2
        win_y_high = y_current + self.window_height // 2
        win_x_low = x_current - self.margin - prev_margin
        win_x_high = x_current + self.margin + prev_margin

        cv2.rectangle(binary_img, (win_x_low, win_y_low), (win_x_high, win_y_high), color, 2)

        good_inds = ((self.nonzeroy >= win_y_low) & (self.nonzeroy < win_y_high) &
                     (self.nonzerox >= win_x_low) & (self.nonzerox < win_x_high)).nonzero()[0]

        if len(good_inds) > self.minpix:
            min_x = np.min(self.nonzerox[good_inds])
            max_x = np.max(self.nonzerox[good_inds])

            if (max_x - min_x) > 100:
                y_current -= self.window_height
                flag += 1
                return x_current, y_current, flag, self.margin, x_prev

            center_y = y_current
            mean_x = np.mean(self.nonzerox[good_inds])
            delta = mean_x + (mean_x - x_prev)
            x_current = int(self.alpha * mean_x + (1 - self.alpha) * delta)
            y_current = center_y - self.window_height
            prev_margin = min((max_x - min_x) // 4, 20)
            x_prev = x_current

            # 반대 트랙의 현재 x와 너무 가까우면 이 윈도우는 스킵하여 소유권 충돌 방지
            if other_x is not None and abs(x_current - other_x) < min_sep:
                flag += 1
                return x_current, y_current, flag, prev_margin, x_prev

            cv2.circle(binary_img, (x_current, y_current), 5, (0, 0, 255), -1)
            flag = 0

            # 생성 라벨 고정: 왼쪽으로 시작했으면 끝까지 왼쪽, 오른쪽도 동일
            if is_left_lane:
                self.left_lanes.append((x_current, y_current))
            else:
                self.right_lanes.append((x_current, y_current))

        else:
            y_current -= self.window_height
            flag += 1

        return x_current, y_current, flag, prev_margin, x_prev

    def sliding_window_adaptive(self,binary_img, nwindows=15, margin=80, minpix=100):
        # margin=80
        #margin= 75

        binary_img_color = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
        #histogram = np.sum(binary_img[binary_img.shape[0]//2:, :], axis=0)
        self.left_blocked_flag = 0
        self.right_blocked_flag = 0
        self.left_lanes =[]
        self.right_lanes =[]
        
        self.margin = margin
        self.minpix = minpix
        #self.midpoint = histogram.shape[0] // nwindows
        self.left_lane_start = 160
        self.right_lane_start = 480 #480


        self.window_height = binary_img.shape[0] // nwindows
        self.nonzero = binary_img.nonzero()
        self.nonzeroy = np.array(self.nonzero[0])
        self.nonzerox = np.array(self.nonzero[1])
        
        prev_left_margin = margin
        prev_right_margin = margin

        leftx_current = self.left_lane_start
        leftx_prev = self.left_lane_start
        lefty_current = binary_img.shape[0] - self.window_height // 2

        rightx_current = self.right_lane_start
        rightx_prev = self.right_lane_start
        righty_current = binary_img.shape[0] - self.window_height // 2

        for window in range(nwindows):
            # 윈도우 범위 (적응형 이동)
            # print(f"rightx_current {rightx_current}")
            if self.left_blocked_flag < 8:
                leftx_current, lefty_current, self.left_blocked_flag, prev_left_margin, leftx_prev = \
                    self.sliding_window_lane_calculation(
                        leftx_current, lefty_current, prev_left_margin, leftx_prev,
                        self.left_blocked_flag, binary_img_color, True,
                        other_x=rightx_current, min_sep=self.min_sep, color=(0, 255, 0))
                                                                                                            
            if self.right_blocked_flag < 8:
                rightx_current, righty_current, self.right_blocked_flag, prev_right_margin, rightx_prev  = \
                    self.sliding_window_lane_calculation(
                        rightx_current, righty_current, prev_right_margin, rightx_prev,
                        self.right_blocked_flag, binary_img_color, False,
                        other_x=leftx_current, min_sep=self.min_sep, color=(255, 0, 0))

        # print(self.left_lane_start)
        # print(self.right_lane_start)
        
        #print(self.left_lanes)
        #print(self.right_lanes)

        # print(self.left_lane_start < self.left_lane_end)
        # print(self.right_lane_start < self.right_lane_end)
        return binary_img_color, self.left_lanes, self.right_laness