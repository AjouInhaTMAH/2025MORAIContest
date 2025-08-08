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
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Bool
from utils import check_timer
import subprocess
import json
import numpy as np
from morai_msgs.msg import GetTrafficLightStatus
from dec_lane_detect import lane_detect
from auto_drive_sim.msg import PersonBBox  # ← 커스텀 메시지에 confidence 필드 포함
import cv2

MIN_Y = 0
MAX_Y = 1

class DecMissionAll:
    def __init__(self):
        print(f"DecMissionAll start")

        rospy.init_node('dec_mission_all_node')
        self.kill_slim_mover()
        
        self.init_lidar_info()
        self.init_camera_info()
        
        self.init_pubSub()
        
        self.init_mission2_3()
        self.init_mission4()
        self.init_mission5()
        self.init_goal()
        self.lane_detect = lane_detect()
        self.lane_mode = 4  # 0=기본, 1=왼쪽 차선만, 2=오른쪽 차선만 등
        self.lane_mode = 3  # 0=기본, 1=왼쪽 차선만, 2=오른쪽 차선만 등
        # self.lane_mode = 2  # 0=기본, 1=왼쪽 차선만, 2=오른쪽 차선만 등
        # self.lane_mode = 1  # 0=기본, 1=왼쪽 차선만, 2=오른쪽 차선만 등
        self.lane_mode = 0  # 0=기본, 1=왼쪽 차선만, 2=오른쪽 차선만 등
    def kill_slim_mover(self):
        node_name = '/throttle_interpolator'
        try:
            subprocess.run(['rosnode', 'kill', node_name], check=True)
            print(f"✅ 노드 {node_name} 종료 성공")
        except subprocess.CalledProcessError:
            print(f"❌ 노드 {node_name} 종료 실패 (이미 종료되었거나 존재하지 않음)")
    def init_pubSub(self):
        rospy.Subscriber("/perception/camera", String, self.CB_camera_info, queue_size=1)
        rospy.Subscriber("/perception/lidar", String, self.CB_lidar_info, queue_size=1)
        rospy.Subscriber('/is_to_go_traffic', Bool, self.CB_check_to_go_traffic_info)
        rospy.Subscriber("/person_bbox", PersonBBox, self.CB_dynamic_obs)

        rospy.Subscriber("/lane_mode", Int32, self.CB_car_nav_info)
        rospy.Subscriber('/cur_pose', String, self.CB_currentPose_info)
        self.is_to_go_traffic = False
        self.traffic_msg = GetTrafficLightStatus()
        self.x = 0
        self.y = None
        self.w = 0
        self.vel = 0
        self.motor_cmd_msg_pub = Float64()
        self.servo_cmd_msg_pub = Float64()
        self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane = [],None,None,None,None

    def init_lidar_info(self):
        self.left_obstacle = False
        self.front_obstacle = False
        self.right_obstacle = False
    def init_camera_info(self):
        self.stop_line = []

        self.yellow_left_lane = None
        self.yellow_right_lane = None
        self.white_left_lane = None
        self.white_right_lane = None 

    def init_mission2_3(self):
        self.lidar_flag       = False
        self.current_lane     = "right"
        self.avoid_side       = None
        self.in_avoid_mode    = False
        self.dynamic_obs_flag = False
        self.obs_flag         = False
        self.count_stopsline = 0
        self.thick_plan = {
            1: {"type":"steer_fixed", "steer":0.5, "speed":800, "duration":1.5},
        }
        
    def init_mission4(self):
        self.mi4_stop_flag = False
        self.mi4_in_flag = False
        self.mi4_out_flag = False
    def init_mission5(self):
        self.stop_mission5_flag = False
        self.pass_mission5_flag = False
    def init_goal(self):
        self.go_goal_stop_flag = False
        self.go_goal_stop_end_flag = False
        
    def CB_currentPose_info(self, msg):
        try:
            data = json.loads(msg.data)
            self.x = data.get("x")
            self.y = data.get("y")
            self.yaw = data.get("yaw")
            self.vel = data.get("vel")
            self.lane_detect.set_currentPose_info(self.x, self.y, self.yaw)
        except json.JSONDecodeError as e:
            rospy.logwarn(f"JSON Decode Error: {e}")
        except Exception as e:
            rospy.logerr(f"Unexpected error: {e}")
    def CB_check_to_go_traffic_info(self, msg):
        self.is_to_go_traffic = msg.data
    def CB_lidar_info(self,msg):
        try:
            data = json.loads(msg.data)
            self.left_obstacle, self.front_obstacle, self.right_obstacle = data[0],data[1],data[2]
            self.lidar_flag = data[3]
            self.lane_detect.set_lidar_info(self.left_obstacle, self.front_obstacle, self.right_obstacle)
        except Exception as e:
            print("복원 실패:", e)
    def CB_camera_info(self,msg):
        try:
            data = json.loads(msg.data)
            self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane, self.current_lane = data[0],data[1],data[2],data[3],data[4], data[5]
            self.lane_detect.set_camera_info(self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane)
        except Exception as e:
            print("복원 실패:", e)
    def CB_dynamic_obs(self, msg):
        center_x   = (msg.xmin + msg.xmax) / 2.0
        box_height = msg.ymax - msg.ymin
        frame_width, frame_height = 640, 480
        margin = frame_width * 0.15 / 2.0
        center_min = frame_width/2.0 - margin
        center_max = frame_width/2.0 + margin
        HEIGHT_THRESHOLD = 0.2 * frame_height
        if (center_min <= center_x <= center_max) and (box_height > HEIGHT_THRESHOLD):
            self.dynamic_obs_flag   = True
        else:
            self.dynamic_obs_flag = False

    def CB_car_nav_info(self, msg):
        self.lane_mode = msg.data  # 예: 1, 2, 3 등의 영역 구분  
        
    def handle_zone_mission2_3(self):
        now = rospy.get_time()
        
        if self.dynamic_obs_flag:
            print(f"동적 장애물 발견")
            steer, speed = 0.5, 0
            self.lidar_flag = False
            self.obs_flag   = True
            self.waypoint_idx = -1
            self.lane_detect.publish_move(speed,steer)
            
        elif self.lidar_flag or self.in_avoid_mode:
            print(f"정적 장애물 발견")
            if not self.in_avoid_mode:
                self.avoid_side    = "left" if self.current_lane == "right" else "right"
                self.in_avoid_mode = True
                self.obs_flag      = True
                self.waypoint_idx  = 0
                self.last_time     = now
                self.sleep_duration= 1.0

            if now - self.last_time > self.sleep_duration:
                self.waypoint_idx += 1
                self.last_time = now

            if   self.waypoint_idx == 0:
                steer, speed = (0.1 if self.avoid_side == "left" else 0.9), 0
                self.sleep_duration = 1.5
            elif self.waypoint_idx == 1:
                steer, speed = (0.1 if self.avoid_side == "left" else 0.9), 1000
                self.sleep_duration = 0.5
            elif self.waypoint_idx == 2:
                steer, speed = (0.8 if self.avoid_side == "left" else 0.2), 800
                self.sleep_duration = 0.8
            elif self.waypoint_idx == 3:
                steer, speed = 0.5, 800
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
            self.lane_detect.publish_move(speed,steer)
        elif self.stop_line != [] and self.stop_line[MAX_Y] > 440:
        # {"type":"steer_fixed", "steer":0.5, "speed":800, "duration":1.5},
            steer = float(self.thick_plan[1]["steer"])
            speed = float(self.thick_plan[1]["speed"])
            duration = float(self.thick_plan[1]["duration"])
            self.lane_detect.publish_move(speed,steer)
            self.count_stopsline += 1
            print(f"self.count_stopsline {self.count_stopsline}")
            print(f"self.count_stopsline {self.count_stopsline}")
            print(f"self.count_stopsline {self.count_stopsline}")
            sleep(duration)
        else:
            mode, left_lane, right_lane = self.lane_detect.pth01_ctrl_decision()
            self.lane_detect.pth01_ctrl_move(mode, left_lane, right_lane)
            
        # 부드러운 steer_gain 계산 함수
    def handle_zone_mission4(self):
        if self.mi4_out_flag:
            print(f"5")
            mode, left_lane, right_lane = self.lane_detect.pth01_ctrl_decision_left()
            self.lane_detect.pth01_ctrl_move(mode, left_lane, right_lane)
        elif self.mi4_in_flag:
            self.lane_detect.publish_move(2400,-0.7179)
            sleep(0.16)
            self.lane_detect.publish_move(2100,1)
            sleep(0.425)
            self.mi4_out_flag = True
            print(f"4")
            self.lane_detect.stop_time()
        elif self.mi4_stop_flag:
            print(f"3")
            if self.lane_detect.check_obstacle_rotary():
                return
            print(f"no obs")
            self.lane_detect.publish_move(2400,0.5)
            # sleep(0.275)
            sleep(0.55)
            self.lane_detect.publish_move(2400,1)
            sleep(0.4)
            self.mi4_in_flag = True
        elif self.stop_line != [] and self.stop_line[MAX_Y] > 240:
            print(f"2")
            self.mi4_stop_flag =True
            self.lane_detect.stop_time(2)
        else:
            print(f"1")
            self.lane_detect.chose_center_left()
            self.lane_detect.ctrl_moveByLine_right()
            # mode, left_lane, right_lane = self.ctrl_decision_left()
            # self.ctrl_move(mode, left_lane, right_lane)
    def handle_zone_mission5(self):
        if self.pass_mission5_flag:
            self.lane_detect.chose_center_right()
            self.lane_detect.ctrl_moveByLine_right()
        elif self.stop_mission5_flag and self.is_to_go_traffic:
            print(f"movemove")
            steer = 0.5
            speed = 800
            self.lane_detect.publish(speed,steer)
            sleep(0.5)
            steer = 0.3
            speed = 800
            self.lane_detect.publish(speed,steer)
            sleep(2)
            steer = 0.225
            speed = 800
            self.lane_detect.publish(speed,steer)
            sleep(2)
            self.pass_mission5_flag = True
            # self.stop_time(5)
        elif self.stop_mission5_flag:
            print(f"stopstop")
            self.lane_detect.publish(0,0.5)
        elif self.stop_line != [] and self.stop_line[MAX_Y] > 400:
            print(f"2")
            self.stop_mission5_flag =True
            self.lane_detect.stop_time(0)
        else:
            self.lane_detect.chose_center_right()
            self.lane_detect.ctrl_moveByLine_right()
    def handle_zone_goal_01(self):
        self.lane_detect.chose_center_right()
        self.lane_detect.ctrl_moveByLine_right()
        # mode, left_lane, right_lane = self.lane_detect.pth01_ctrl_decision_right()
        # self.lane_detect.pth01_ctrl_move_right(mode, left_lane, right_lane)
    def handle_zone_goal_02(self):
        if self.go_goal_stop_end_flag:
            print(f"4!")
            self.lane_detect.chose_center_right()
            self.lane_detect.ctrl_moveByLine_right()
        elif self.go_goal_stop_flag:
            self.go_goal_stop_end_flag = self.lane_detect.drvie_amcl()
        elif self.stop_line != [] and self.stop_line[MAX_Y] > 320:
            print(f"stop!")
            self.go_goal_stop_flag = True
        else:
            print(f"1!")
            self.lane_detect.chose_center_right()
            self.lane_detect.ctrl_moveByLine_right()


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
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            start = time()
            # print(f"self.stop_line {self.stop_line}")
            if self.yellow_left_lane is not None or self.white_left_lane is not None or self.white_right_lane is not None:
                if self.lane_mode == 0:
                    print(f"mode {self.lane_mode}")
                    self.handle_zone_mission2_3()
                elif self.lane_mode == 1:
                    print(f"mode {self.lane_mode}")
                    self.handle_zone_mission4()
                elif self.lane_mode == 2:
                        print(f"mode {self.lane_mode}")
                        self.handle_zone_mission5()
                elif self.lane_mode == 3:
                        print(f"mode {self.lane_mode}")
                        self.handle_zone_goal_01()
                elif self.lane_mode == 4:
                        print(f"mode {self.lane_mode}")
                        self.handle_zone_goal_02()
                else:
                    print(f"self.lane_mode {self.lane_mode}")
                    mode, left_lane, right_lane = self.lane_detect.pth01_ctrl_decision_right()
                    self.lane_detect.pth01_ctrl_move_right(mode, left_lane, right_lane)
            end = time()
            print(f"time2 {end - start}")
            self.stop_line, self.yellow_left_lane, self.yellow_right_lane, self.white_left_lane, self.white_right_lane = [],None,None,None,None
            
            rate.sleep()
            
if __name__ == '__main__':

    node = DecMissionAll()
    node.processing()   # spin 대신 processing 돌기

