#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy 
from morai_msgs.msg import GetTrafficLightStatus
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np
from time import *

class Traffic_control:
    def __init__(self):
        rospy.init_node("lane_sub_node")
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        rospy.Subscriber ("/GetTrafficLightStatus", GetTrafficLightStatus, self.traffic_CB)
        self.steer_msg = Float64()
        self.speed_msg = Float64()
        self.bridge = CvBridge()
        self.traffic_msg = GetTrafficLightStatus()
        self.traffic_flag = 0

        self.prev_signal = 0
        self.signal = 0
        
        self.prev_cross_flag = False
        self.cross_flag = False

        self.img = []
        self.center_index = 0 
        self.standard_line = 0 
        self.degree_per_pixel = 0
        self.yellow_detected = False
        self.blue_detected = True
        self.yellow_center_index = 0
        self.cross_flag_num = 0

        self.cross3_start_time = 0
        self.cross3_done = False

    def traffic_CB(self,msg):
        self.traffic_msg = msg
        if self.traffic_msg.trafficLightIndex == "SN000005":
            self.signal = self.traffic_msg.trafficLightStatus
            if self.prev_signal != self.signal:
                self.prev_signal = self.signal
                self.traffic_think()
    
    def traffic_think(self):    
    # 1 = red / left = 33 / yellow = 4 / green = 16
        if self.signal == 1: # stop
            #print("red")
            pass
        elif self.signal == 4:
            #print("yellow")
            pass
        elif self.signal == 16:
            #print("green")
            pass
        elif self.signal == 33:
            pass
            #print("left")
        else:
            pass
        
    def cam_CB(self,msg):
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg)
        (self.yellow_center_index, 
         self.yellow_detected,
         self.blue_detected, 
         self.warped_img, 
         self.center_index, 
         self.standard_line, 
         self.degree_per_pixel) = self.cam_lane_detection()


        # ✅ 정지선 인식 시 False → True로 바뀌는 순간만 카운트 증가
        if not self.prev_cross_flag and self.cross_flag:
            self.cross_flag_num += 1
            print(f"[정지선 인식] cross_flag_num: {self.cross_flag_num}")
        
        self.prev_cross_flag = self.cross_flag


        
    def cam_lane_detection(self):
        y,x = self.img.shape[0:2]
        img_hsv = cv2.cvtColor(self.img,cv2.COLOR_BGR2HSV)
        yellow_lower = np.array([25,150,0])
        yellow_upper = np.array([35,255,255])
        yellow_range = cv2.inRange(img_hsv,yellow_lower,yellow_upper)
        white_lower = np.array([0,0,192])
        white_upper = np.array([179,64,255])
        white_range = cv2.inRange(img_hsv,white_lower,white_upper)
        combined_range = cv2.bitwise_or(yellow_range,white_range)
        filltered_img = cv2.bitwise_and(self.img,self.img,mask=combined_range)
        
        src_point1 = [40,400]
        src_point2 = [180,300]
        src_point3 = [x-180,300]
        src_point4 = [x-40,400]
        src_points = np.float32([src_point1,src_point2,src_point3,src_point4])
        
        dst_point1=[x//8,480] # desination point
        dst_point2=[x//8,0]
        dst_point3=[x//8*7,0]
        dst_point4=[x//8*7,480]
        dst_points = np.float32([dst_point1,dst_point2,dst_point3,dst_point4])
     
        matrix = cv2.getPerspectiveTransform(src_points,dst_points)
        warped_img = cv2.warpPerspective(filltered_img,matrix,[x,y])
        grayed_img = cv2.cvtColor(warped_img,cv2.COLOR_BGR2GRAY)
        bin_img = np.zeros_like(grayed_img)
        bin_img[grayed_img>50] = 1
        histogram_x = np.sum(bin_img,axis=0)
        histogram_y = np.sum(bin_img,axis=1)
        
        indices = np.where(histogram_x>20)[0] 
        left_hist = histogram_x[0:x//2]
        right_hist = histogram_x[x//2:]
        up_hist = histogram_y[0:y//4*1]
        down_hist = histogram_y[y//4*1:]
        left_indices = np.where(left_hist>20)[0]
        right_indices = np.where(right_hist>20)[0] + 320
        
        cross_indices = np.where(down_hist>300)[0]+y//4*1

        try:
            cross_threshold = 35
            cross_diff = cross_indices[-1] - cross_indices[0]
            if cross_threshold < cross_diff:
                self.cross_flag = True
                cv2.rectangle(warped_img,[0,cross_indices[0]],[x,cross_indices[-1]],[0,255,0],3)
            else:
                self.cross_flag = False
        except:
            self.cross_flag = False
        

        warp_yellow = cv2.warpPerspective(yellow_range, matrix, (x, y))
        left_yellow = warp_yellow[:, :x//2]
        left_yellow_hist = np.sum(left_yellow, axis=0)
        left_yellow_indices = np.where(left_yellow_hist > 40)[0]  # 노란선 존재 시

        if len(left_yellow_indices) != 0 and len(right_indices) != 0:
            yellow_center = (left_yellow_indices[0] + left_yellow_indices[-1]) // 2
            center_index = (left_yellow_indices[-1]+ right_indices[-1]) // 2
            yellow_detected = True
            blue_detected = True
            yellow_center_index = center_index
            cv2.rectangle(warped_img, (yellow_center-10,0), (yellow_center+10,y), (0,255,255), 3)
            cv2.rectangle(warped_img,[right_indices[0],0],[right_indices[-1],y],[255,0,0],3) 
            print("just forward")

        elif len(left_yellow_indices) != 0:
            # 왼쪽 노란선 기준 오른쪽 흰선 추출
            yellow_center = (left_yellow_indices[0] + left_yellow_indices[-1]) // 2
            yellow_detected = True
            blue_detected = False
            yellow_center_index = yellow_center
            center_index = x // 2
            cv2.rectangle(warped_img, (yellow_center-10,0), (yellow_center+10,y), (0,255,255), 3)
            print("yellow line first")

        elif len(left_indices) != 0 and len(right_indices) != 0:
            center_index = (indices[0]+indices[-1])//2
            print("both_line")
            cv2.rectangle(warped_img,[indices[0],0],[indices[-1],y],[255,0,0],3)
            
            blue_detected = True
            yellow_detected = False
            yellow_center_index = x // 2
            
        elif len(left_indices) != 0 and len(right_indices) == 0:
            center_index = (left_indices[0]+left_indices[-1]) // 2 
            print("left_line")
            cv2.rectangle(warped_img,[left_indices[0],0],[left_indices[-1],y],[255,0,0],3)

            blue_detected = True
            yellow_detected = False
            yellow_center_index = x // 2
            
        elif len(left_indices) == 0 and len(right_indices) != 0:
            center_index = (right_indices[0]+right_indices[-1]) // 2
            cv2.rectangle(warped_img,[right_indices[0],0],[right_indices[-1],y],[255,0,0],3) 
            print("right_line")
            
            blue_detected = True
            yellow_detected = False
            yellow_center_index = x // 2
            
        
        else:

            blue_detected = True
            yellow_detected = False
            yellow_center_index = 0
            center_index = x // 2
            print("no_line")

        #first_time = time()
        #second_time = time()
        #print(f"different:{second_time-first_time}")        
        
        standard_line = x // 2
        degree_per_pixel = 3 / x

        return yellow_center_index, yellow_detected, blue_detected, warped_img, center_index, standard_line, degree_per_pixel
    
    def action(self):
        if len(self.img) != 0:
            print(f"self.cross_flag_num:{self.cross_flag_num} / cross_flag : {self.cross_flag} / signal : {self.signal}")
             
            
            if self.cross_flag_num == 2 and self.cross_flag == True and self.signal == 33:
                steer = 0.85
                speed = 1000
                print("turn right")

            elif self.cross_flag_num == 3 and not self.cross3_done:
                current_time = time()

                if self.cross3_start_time == 0:
                    self.cross3_start_time = current_time
                    print("[cross 3] 시작")

                if current_time - self.cross3_start_time < 3:
                    steer = 0.5
                    speed = 1000
                    print("[cross 3] 5초 동안 직진")

                    # 직진 명령 후 return: 아래 코드 실행 방지
                    self.speed_msg.data = speed
                    self.steer_msg.data = steer
                    self.speed_pub.publish(self.speed_msg)
                    self.steer_pub.publish(self.steer_msg)
                    cv2.imshow("img", self.img)
                    cv2.imshow("warped_img", self.warped_img)
                    cv2.waitKey(1)
                    return
                else:
                    self.cross3_done = True
                    steer = 0.5
                    speed = 800
                    print("[cross 3] 완료")
                
            elif self.cross_flag_num == 4 and self.cross_flag == True:
                steer = 0.5
                speed = 0
            
            elif self.cross_flag == True and self.signal == 33:
                #steer = (self.center_index - self.standard_line)*self.degree_per_pixel 
                steer = 0.1
                speed = 800

            elif self.cross_flag == True and self.signal != 33 and self.signal !=0:
                speed = 0
                steer = 0.5 
                #print(f"cross_flag: {self.cross_flag}, signal: {self.signal}")

            elif self.yellow_detected == True and self.blue_detected == False:
                yellow_target = self.standard_line - 40 
                yellow_error = self.yellow_center_index - yellow_target
                steer = -yellow_error * self.degree_per_pixel + 0.5
                print(f"[YELLOW] steer: {steer:.3f} (error: {yellow_error})")

                max_speed = 800
                min_speed = 400
                steer_deviation = abs(steer - 0.5)
                speed = max_speed-steer_deviation*(max_speed-min_speed)
                speed = int(speed)

            else:
                steer = (self.center_index - self.standard_line)*self.degree_per_pixel 
                steer += 0.5
                
                max_speed = 800
                min_speed = 400
                steer_deviation = abs(steer - 0.5)
                speed = max_speed-steer_deviation*(max_speed-min_speed)
                speed = int(speed)

            #print(f"steer:{steer}")
            self.speed_msg.data = speed
            self.steer_msg.data = steer
            self.speed_pub.publish(self.speed_msg)
            self.steer_pub.publish(self.steer_msg)
        
            cv2.imshow("img", self.img)
            cv2.imshow("warped_img", self.warped_img)
            cv2.waitKey(1)

def main():
    try:
        traffic_control = Traffic_control()
        while not rospy.is_shutdown():
            traffic_control.action()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()