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
from drive_decision.lane import dec_lane_amcl, dec_lane_curvature, dec_lane_distance

MAX_Y = 1
class DecLaneMode_004:
    def __init__(self,DecLaneAmcl, DecLaneDistance):
        self.init_goal()
        self.init_processing(DecLaneAmcl, DecLaneDistance)
    def init_goal(self):
        self.go_goal_stop_flag = False
        self.go_goal_stop_end_flag = False
    def init_processing(self, DecLaneAmcl:dec_lane_amcl.DecLaneAmcl , DecLaneDistance:dec_lane_distance.DecLaneDistance):
        self.DecLaneAmcl = DecLaneAmcl
        self.DecLaneDistance = DecLaneDistance
    def handle_zone_goal_02(self,stop_line):
        if self.go_goal_stop_end_flag:
            print(f"4!")
            self.DecLaneDistance.chose_center_right_white_lane()
            self.DecLaneDistance.ctrl_moveByLine()
        elif self.go_goal_stop_flag:
            self.go_goal_stop_end_flag = self.DecLaneAmcl.drvie_amcl()
        elif stop_line != [] and stop_line[MAX_Y] > 320:
            print(f"stop!")
            self.go_goal_stop_flag = True
        else:
            print(f"1!")
            self.DecLaneDistance.chose_center_right_white_lane()
            self.DecLaneDistance.ctrl_moveByLine()
