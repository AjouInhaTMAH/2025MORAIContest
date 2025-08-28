#! /usr/bin/env python3

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped

from turtlesim.msg import Pose
import tf_conversions

from turtlesim.srv import Spawn, SpawnRequest

class TurtleTwoBroadcaster:
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.transforStamped = TransformStamped()
        self.transforStamped.header.frame_id = "world"
        self.transforStamped.child_frame_id = "turtle2"

        service = rospy.ServiceProxy('spawn', Spawn)
        service.wait_for_service()
        srv = SpawnRequest()
        srv.name = 'turtle2'
        service(srv)
        
        rospy.Subscriber('/turtle2/pose', Pose, self.callback)

    def callback(self, data):
        self.transforStamped.transform.translation.x = data.x
        self.transforStamped.transform.translation.y = data.y
        
        q = tf_conversions.transformations.quaternion_from_euler(0,0, data.theta)
        
        
        self.transforStamped.transform.rotation.x = q[0]
        self.transforStamped.transform.rotation.y = q[1]
        self.transforStamped.transform.rotation.z = q[2]
        self.transforStamped.transform.rotation.w = q[3]



    def broadcast(self):
        # 브로드 캐스팅을 진행할 거임
        self.transforStamped.header.stamp = rospy.Time().now()
        self.br.sendTransform(self.transforStamped)



def main():
    rospy.init_node('turtle_two_broadcaster')
    ttb = TurtleTwoBroadcaster()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
        ttb.broadcast()

if __name__ == "__main__":
    main()