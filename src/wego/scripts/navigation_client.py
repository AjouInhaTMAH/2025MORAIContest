#!/usr/bin/env python
import rospy
import actionlib
import math
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from morai_msgs.msg import ObjectStatusList
from std_msgs.msg import Bool

class NavigationClient():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base")

        self.goal_list = []
        self.goal_index = 0
        self.goal_sent = False
        self.distance_threshold = 1.0
        self.robot_x = None
        self.robot_y = None

        self.delivery_objects = [None, None]
        self.delivery_goal = [None]
        self.route_planned = False
        # self.final_destination = (10.918598175048828, -3.1679697036743164, 0.7024387324996206, 0.7117442146475983)
        self.final_destination = (10.818598175048828, -3.1679697036743164, 0.7024387324996206, 0.7117442146475983)

        # 4x4 transform matrix: ObjectInfo → map
        self.T_map_from_objectinfo = np.array([[ 6.12323400e-17 ,-1.00000000e+00 ,-0.00000000e+00 , 5.39406395e+00],
        [ 1.00000000e+00, 6.12323400e-17 , 0.00000000e+00, -4.38229799e+00],
        [ 0.00000000e+00,  0.00000000e+00,  1.00000000e+00, -3.10335420e-02],
        [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]
        )

        rospy.wait_for_service('/move_base/make_plan')
        self.make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber("/delivery_object", ObjectStatusList, self.object_callback)
        rospy.Timer(rospy.Duration(0.1), self.check_distance)
        self.pub_start_mission2And3 = rospy.Publisher('/start/mission_zero', Bool, queue_size=10)
    def transform_to_map(self, x, y, z=0.0):
        pt = np.array([x, y, z, 1.0])  # Homogeneous coordinates
        pt_map = self.T_map_from_objectinfo @ pt
        print("pt: ", pt)
        print("pt_map: ", pt_map)
        return pt_map[0], pt_map[1], pt_map[2]

    def create_goal(self, x, y, w=1.0, z=0.0):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = w
        goal.target_pose.pose.orientation.z = z
        return goal

    def to_pose_stamped(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    def compute_path_length(self, start, goal):
        try:
            resp = self.make_plan(start=start, goal=goal, tolerance=0.0)
            path = resp.plan.poses
            length = 0.0
            for i in range(len(path) - 1):
                dx = path[i+1].pose.position.x - path[i].pose.position.x
                dy = path[i+1].pose.position.y - path[i].pose.position.y
                length += math.sqrt(dx**2 + dy**2)
            return length
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call make_plan: %s" % e)
            return float('inf')

    def pose_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def object_callback(self, msg):
        if self.route_planned:
            return

        rospy.loginfo("Received delivery object info")

        objects = []
        closest_pedestrian = None
        min_ped_dist = float('inf')

        # Transform and collect objects
        for obj in msg.obstacle_list:
            if obj.type == 2:
                x, y, _ = self.transform_to_map(obj.position.x, obj.position.y)
                obj.position.x = x
                obj.position.y = y
                objects.append(obj)
            if len(objects) == 2:
                break

        # Transform and find closest pedestrian
        for ped in msg.pedestrian_list:
            if ped.type == 0 and self.robot_x is not None:
                x, y, _ = self.transform_to_map(ped.position.x, ped.position.y)
                dx = x - self.robot_x
                dy = y - self.robot_y
                dist = math.sqrt(dx**2 + dy**2)
                if dist < min_ped_dist:
                    min_ped_dist = dist
                    ped.position.x = x
                    ped.position.y = y
                    closest_pedestrian = ped

        if len(objects) < 2 or closest_pedestrian is None:
            rospy.logwarn("delivery_objects 다 안들어옴 or delivery_goal 안들어옴.")
            return

        self.delivery_objects[0] = objects[0]
        self.delivery_objects[1] = objects[1]
        self.delivery_goal[0] = closest_pedestrian

        self.route_planned = True
        self.plan_optimal_route()

    def plan_optimal_route(self):
        obj1, obj2 = self.delivery_objects
        ped = self.delivery_goal[0]

        robot_pose = self.to_pose_stamped(self.robot_x, self.robot_y)
        obj1_pose = self.to_pose_stamped(obj1.position.x, obj1.position.y)
        obj2_pose = self.to_pose_stamped(obj2.position.x, obj2.position.y)
        ped_pose = self.to_pose_stamped(ped.position.x, ped.position.y)

        # Path A: robot → obj1 → obj2 → ped
        d1 = self.compute_path_length(robot_pose, obj1_pose) + \
             self.compute_path_length(obj1_pose, obj2_pose) + \
             self.compute_path_length(obj2_pose, ped_pose)

        # Path B: robot → obj2 → obj1 → ped
        d2 = self.compute_path_length(robot_pose, obj2_pose) + \
             self.compute_path_length(obj2_pose, obj1_pose) + \
             self.compute_path_length(obj1_pose, ped_pose)

        if d1 <= d2:
            ordered = [obj1, obj2, ped]
            rospy.loginfo("🚗 경로 선택: obj1 → obj2 → ped (길이 %.2f m)" % d1)
        else:
            ordered = [obj2, obj1, ped]
            rospy.loginfo("🚗 경로 선택: obj2 → obj1 → ped (길이 %.2f m)" % d2)

        self.goal_list = [
            self.create_goal(o.position.x, o.position.y)
            for o in ordered
        ]

        # 마지막 고정 도착지점 추가
        final_x, final_y, final_w,final_z = self.final_destination
        self.goal_list.append(self.create_goal(final_x, final_y,final_w,final_z))
        rospy.loginfo("📍 마지막 도착지점 추가됨: (%.2f, %.2f, %.2f, %.2f)" % (final_x, final_y, final_w,final_z))

        self.send_goal()

    def send_goal(self):
        if self.goal_index < len(self.goal_list):
            goal = self.goal_list[self.goal_index]
            print("goal: ", goal)
            goal.target_pose.header.stamp = rospy.Time.now()
            self.client.send_goal(goal)
            self.goal_sent = True
            rospy.loginfo(f"Sent goal {self.goal_index + 1}/{len(self.goal_list)}")
        else:
            self.pub_start_mission2And3.publish(True)
            rospy.loginfo("🎉 All goals completed.")
            rospy.signal_shutdown("Navigation complete.")

    def check_distance(self, event):
        if self.robot_x is None or not self.goal_sent:
            return

        target = self.goal_list[self.goal_index].target_pose.pose.position
        dx = target.x - self.robot_x
        dy = target.y - self.robot_y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < self.distance_threshold:
            rospy.loginfo(f"✅ Reached waypoint {self.goal_index + 1} (distance: {distance:.2f} m)")
            self.goal_index += 1
            self.goal_sent = False
            self.send_goal()

if __name__ == '__main__':
    rospy.init_node('navigation_client')
    NavigationClient()
    rospy.spin()
