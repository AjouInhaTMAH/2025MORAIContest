import rospy
import actionlib
import math
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

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

        self.received_waypoints = [None, None, None]  # wp1, wp2, wp3

        # Subscribe to robot pose
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        # Subscribe to each waypoint
        rospy.Subscriber("/wp1", Pose, self.wp1_callback)
        rospy.Subscriber("/wp2", Pose, self.wp2_callback)
        rospy.Subscriber("/wp3", Pose, self.wp3_callback)

        rospy.Timer(rospy.Duration(0.1), self.check_distance)

        self.final_goal = self.create_goal(10.531286239624023, -1.1053885221481323, 0.7098, 0.7043)

    def create_goal(self, x, y, w, z):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = w
        goal.target_pose.pose.orientation.z = z
        return goal

    def pose_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def wp1_callback(self, msg):
        rospy.loginfo("Received wp1")
        self.received_waypoints[0] = msg
        self.check_all_waypoints_received()

    def wp2_callback(self, msg):
        rospy.loginfo("Received wp2")
        self.received_waypoints[1] = msg
        self.check_all_waypoints_received()

    def wp3_callback(self, msg):
        rospy.loginfo("Received wp3")
        self.received_waypoints[2] = msg
        self.check_all_waypoints_received()

    def check_all_waypoints_received(self):
        if all(wp is not None for wp in self.received_waypoints):
            self.select_and_sort_goals()

    def select_and_sort_goals(self):
        waypoints_with_dist = []
        for wp in self.received_waypoints:
            dx = wp.position.x - self.robot_x
            dy = wp.position.y - self.robot_y
            dist = math.sqrt(dx**2 + dy**2)
            waypoints_with_dist.append((dist, wp))

        sorted_wps = sorted(waypoints_with_dist, key=lambda x: x[0])
        self.goal_list = [
            self.create_goal(wp.position.x, wp.position.y, wp.orientation.w, wp.orientation.z)
            for _, wp in sorted_wps
        ]
        self.goal_list.append(self.final_goal)

        rospy.loginfo("All waypoints received and sorted. Sending first goal.")
        self.send_goal()

    def send_goal(self):
        if self.goal_index < len(self.goal_list):
            goal = self.goal_list[self.goal_index]
            goal.target_pose.header.stamp = rospy.Time.now()
            self.client.send_goal(goal)
            self.goal_sent = True
            rospy.loginfo(f"Sent goal {self.goal_index + 1}")
        else:
            rospy.loginfo("All goals completed.")
            rospy.signal_shutdown("Navigation complete.")

    def check_distance(self, event):
        if self.robot_x is None or not self.goal_sent:
            return

        target = self.goal_list[self.goal_index].target_pose.pose.position
        dx = target.x - self.robot_x
        dy = target.y - self.robot_y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < self.distance_threshold:
            rospy.loginfo(f"Reached waypoint {self.goal_index + 1} (distance: {distance:.2f} m)")
            self.goal_index += 1
            self.goal_sent = False
            self.send_goal()

def main():
    rospy.init_node("dynamic_waypoint_navigation")
    nav = NavigationClient()
    rospy.spin()

if __name__ == "__main__":
    main()
