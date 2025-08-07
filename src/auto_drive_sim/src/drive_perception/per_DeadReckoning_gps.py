import rospy
import math
import json
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from vesc_msgs.msg import VescStateStamped
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

class DeadReckoningNode:
    def __init__(self):
        print(f"DeadReckoningNode start")
        rospy.init_node('dead_reckoning_node')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.imu_yaw = 0.0
        self.vel = 0.0
        self.last_time = rospy.Time.now()
        self.initialized = False

        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.CB_amcl_raw)
        rospy.Subscriber('/imu', Imu, self.CB_imu_raw)
        rospy.Subscriber('/sensors/core', VescStateStamped, self.CB_vel_raw)

        self.cur_pose_pub = rospy.Publisher('/cur_pose', String, queue_size=1)
        self.rate = rospy.Rate(50)

    def CB_amcl_raw(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw_rad = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw = math.degrees(yaw_rad)
        self.last_time = rospy.Time.now()
        self.initialized = True
        # rospy.loginfo(f"[AMCL] 위치 초기화: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.1f}")

    def CB_imu_raw(self, msg):
        q = msg.orientation
        _, _, yaw_rad = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.imu_yaw = math.degrees(yaw_rad)

    def CB_vel_raw(self, msg):
        if not self.initialized:
            return

        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now

        rpm = msg.state.speed
        self.vel = rpm * 0.0013577451514397 * 1000 / (2 * math.pi * 60)

        yaw_rad = math.radians(self.imu_yaw)
        vx = self.vel * math.cos(yaw_rad)
        vy = self.vel * math.sin(yaw_rad)

        dx = dt * vx / 3.6
        dy = dt * vy / 3.6

        self.x += dx
        self.y += dy
        self.yaw = self.imu_yaw

        # rospy.loginfo(f"[DR] x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.1f} vel={self.vel:.2f}")

        # JSON 메시지 생성 및 publish
        msg_dict = {
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "yaw": round(self.yaw, 2),
            "vel": round(self.vel, 2)
        }
        msg_str = json.dumps(msg_dict)
        self.cur_pose_pub.publish(msg_str)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            
if __name__ == '__main__':
    node = DeadReckoningNode()
    node.run()