#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))  # dec_mission_all.py가 있는 폴더
# src/drive_decision 까지 상대 경로로 올라갔다가 내려가기 (예: 현재 파일 위치 기준)
drive_decision_path = os.path.abspath(os.path.join(current_dir, '..', '..', 'src', 'drive_decision'))

if drive_decision_path not in sys.path:
    sys.path.insert(0, drive_decision_path)
    
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from time import *
from drive_decision.lane import dec_lane_curvature

class DecLaneMode_003:
    def __init__(self):
        self.init_processing()
        
    def init_processing(self):
        pass
    def handle_zone_goal_01(self):
        self.DecLaneDistance.chose_center_right_yellow_lane()
        self.DecLaneDistance.ctrl_moveByLine()
        
        # mode, left_lane, right_lane = self.lane_detect.pth01_ctrl_decision_right()
        # self.lane_detect.pth01_ctrl_move_right(mode, left_lane, right_lane)
        