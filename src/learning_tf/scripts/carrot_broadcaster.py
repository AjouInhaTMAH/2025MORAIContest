#! /usr/bin/env python3

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped



class CarrotBroadcaster:
    def __init__(self):
        self.br = tf2_ros.StaticTransformBroadcaster()
        self.transforStamped = TransformStamped()
        self.transforStamped.header.frame_id = "turtle1"
        self.transforStamped.child_frame_id = "carrot"

    def broadcast(self):
        # 브로드 캐스팅을 진행할 거임
        self.transforStamped.header.stamp = rospy.Time().now()
        self.transforStamped.transform.translation.y = 1.0
        self.transforStamped.transform.rotation.w = 1.0
        self.br.sendTransform(self.transforStamped)



def main():
    rospy.init_node('CarrotBroadcaster')
    cb = CarrotBroadcaster()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
        cb.broadcast()

if __name__ == "__main__":
    main()