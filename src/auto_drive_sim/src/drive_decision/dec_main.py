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
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Bool
from utills import check_timer
import subprocess
import json
import numpy as np
from morai_msgs.msg import GetTrafficLightStatus
from auto_drive_sim.msg import PersonBBox  # ← 커스텀 메시지에 confidence 필드 포함
from drive_decision.ctrl import ctrl_motor_servo
from drive_decision.lane import dec_lane_curvature
from drive_decision.zone import dec_mission_mode_000, dec_mission_mode_001, dec_mission_mode_002, dec_mission_mode_003
import cv2
import threading

MIN_Y = 0
MAX_Y = 1

class DecMain:
    def __init__(self):
        print(f"DecMain start")

        rospy.init_node('dec_main_node')
        self.init_processing()
        self.init_pubSub()
        self.init_msg()
        self.init_mission_mode()
        self.init_lidar_info()
        self.init_camera_info()
        self.init_timer()
        
    def kill_slim_mover(self):
        node_name = '/throttle_interpolator'
        try:
            subprocess.run(['rosnode', 'kill', node_name], check=True)
            print(f"✅ 노드 {node_name} 종료 성공")
        except subprocess.CalledProcessError:
            print(f"❌ 노드 {node_name} 종료 실패 (이미 종료되었거나 존재하지 않음)")
    def init_pubSub(self):
        rospy.Subscriber("/perception/camera", String, self.CB_camera_info, queue_size=1)
        rospy.Subscriber("/perception/camera/rotary", String, self.CB_camera_rotary_info, queue_size=1)
        rospy.Subscriber("/perception/lidar", String, self.CB_lidar_info, queue_size=1)
        rospy.Subscriber('/is_to_go_traffic', Bool, self.CB_check_to_go_traffic_info, queue_size=1)
        rospy.Subscriber("/person_bbox", String, self.CB_dynamic_obs, queue_size=1)
        rospy.Subscriber("/mission_mode", Int32, self.CB_car_nav_info, queue_size=1)
        rospy.Subscriber('/start/mission_zero', Bool, self.CB_mission_0, queue_size=1)

    def init_mission_mode(self):
        self.mission_mode = 3  # 0=기본, 1=왼쪽 차선만, 2=오른쪽 차선만 등
        self.mission_mode = 2  # 0=기본, 1=왼쪽 차선만, 2=오른쪽 차선만 등
        self.mission_mode = 1  # 0=기본, 1=왼쪽 차선만, 2=오른쪽 차선만 등
        self.mission_mode = 0  # 0=기본, 1=왼쪽 차선만, 2=오른쪽 차선만 등
        self.start_flag = False
    def init_msg(self):
        self.is_to_go_traffic = False
        self.traffic_msg = GetTrafficLightStatus()
        self.x = 0
        self.y = None
        self.w = 0
        self.vel = 0
        self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane = [],None,None,None,None
    def init_processing(self):
        self.CtrlMotorServo = ctrl_motor_servo.CtrlMotorServo()
        
        self.DecLaneCurvature = dec_lane_curvature.DecLaneCurvature(self.CtrlMotorServo)
        self.DecLaneMode_000 = dec_mission_mode_000.DecLaneMode_000(self.CtrlMotorServo,self.DecLaneCurvature)
        self.DecLaneMode_001 = dec_mission_mode_001.DecLaneMode_001(self.CtrlMotorServo,self.DecLaneCurvature,)
        self.DecLaneMode_002 = dec_mission_mode_002.DecLaneMode_002(self.CtrlMotorServo,self.DecLaneCurvature)
        self.DecLaneMode_003 = dec_mission_mode_003.DecLaneMode_003(self.CtrlMotorServo,self.DecLaneCurvature)
    def init_lidar_info(self):
        self.left_obstacle = False
        self.front_obstacle = False
        self.right_obstacle = False
        self.front_near_obstacle = None
        self.front_obstacle_length = None
    def init_camera_info(self):
        self.stop_line = []
        self.yellow_left_lane = None
        self.yellow_right_lane = None
        self.white_left_lane = None
        self.white_right_lane = None 
    def init_timer(self):
        self.check_timer = check_timer.CheckTimer("per_camera_node")

    def CB_check_to_go_traffic_info(self, msg):
        self.is_to_go_traffic = msg.data
        self.DecLaneMode_002.set_is_to_go_traffic(self.is_to_go_traffic)
    def CB_lidar_info(self,msg):
        try:
            data = json.loads(msg.data)
            self.left_obstacle, self.front_obstacle, self.right_obstacle = data[0],data[1],data[2]
            self.front_near_obstacle = data[3]
            self.rotary_obstacle_length = data[4]
            self.DecLaneMode_001.set_lidar_info(self.left_obstacle, self.front_obstacle, self.right_obstacle,self.front_obstacle_length)
            self.DecLaneMode_000.set_front_near_obstacle(self.front_near_obstacle)
            self.DecLaneMode_001.set_rotary_obstacle_length(self.rotary_obstacle_length)
            # print(f"self.front_obstacle_length {self.front_obstacle_length}")
        except Exception as e:  
            print("복원 실패:", e)
    def CB_camera_info(self,msg):
        try:
            data = json.loads(msg.data)
            self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane = data[0],data[1],data[2],data[3],data[4]
            self.lane_mode = data[5]
            self.DecLaneCurvature.set_camera_info(self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane)
            self.DecLaneMode_000.set_lane_mode(self.lane_mode)
            self.DecLaneMode_001.set_lane_mode(self.lane_mode)
            self.DecLaneMode_003.set_camera_info(self.stop_line)
        except Exception as e:
            print("복원 실패:", e)
    def CB_camera_rotary_info(self,msg):
        try:
            data = json.loads(msg.data)
            self.is_out_rotary = data[0]
            self.DecLaneMode_001.set_is_out_rotary(self.is_out_rotary)
            # print(f"self.is_out_rotary {self.is_out_rotary}")
        except Exception as e:
            print("복원 실패:", e)
    def CB_dynamic_obs(self, msg):
        try:
            data = json.loads(msg.data)
            self.dynamic_obs_flag  = data
            self.DecLaneMode_000.set_dynamic_obs_flag(self.dynamic_obs_flag)
        except Exception as e:
            print("복원 실패:", e)
    def CB_car_nav_info(self, msg):
        self.mission_mode = msg.data  # 예: 1, 2, 3 등의 영역 구분  
    def CB_mission_0(self,msg):
        self.start_flag = msg.data  # 예: 1, 2, 3 등의 영역 구분  
        self.kill_slim_mover()  
    def CB_spin(self):
        rospy.spin()

    def processing(self):
        """_summary_
        현재 구역을 나누었다.
        미션 2 & 3 구역
        미션 4 구역
        미션 5 구역
        골인 지점 경로 1
        골인 지점 경로 2
        이는 유동적으로 변할 수 있으므로 유의해야 한다.
        해당 좌표는 amcl_pose를 좌표계로 imu + wheel 값을 이용해서 좌표계를 추정하는 좌표계의 위치로 구분한다.
        """
        rate = rospy.Rate(90)
        # slam억제부분,,
        # self.mission_mode = 0  # 0=기본, 1=왼쪽 차선만, 2=오른쪽 차선만 등
        # self.kill_slim_mover()
        # self.start_flag = True

        while not rospy.is_shutdown():
            if not self.start_flag:
                rate.sleep()
                continue
            try:
                if self.mission_mode == 0:
                    # print(f"mode {self.mission_mode}")
                    result = self.DecLaneMode_000.handle_zone_mission2_3(self.stop_line)
                    if result:
                        self.mission_mode = 1
                elif self.mission_mode == 1:
                    # print(f"mode {self.mission_mode}")
                    result = self.DecLaneMode_001.handle_zone_mission4(self.stop_line)
                    if result:
                        self.mission_mode = 2
                elif self.mission_mode == 2:
                    print(f"mode {self.mission_mode}")
                    result = self.DecLaneMode_002.handle_zone_mission5(self.stop_line)
                    if result:
                        self.mission_mode = 3
                elif self.mission_mode == 3:
                    print(f"mode {self.mission_mode}")
                    self.DecLaneMode_003.handle_zone_goal(self.stop_line)
                else:
                    print(f"Wrong mission_mode {self.mission_mode}")
                    self.DecLaneCurvature.decision()
            except Exception as e:
                rospy.logerr("어떤 값이 존재하지 않습니다.: %s", e)
                

            self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane = [],None,None,None,None
            rate.sleep()
            
            
if __name__ == '__main__':

    node = DecMain()
    cb_thread = threading.Thread(target=node.CB_spin)
    cb_thread.start()
    node.processing()   # spin 대신 processing 돌기

