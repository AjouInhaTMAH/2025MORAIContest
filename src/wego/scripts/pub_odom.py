#! /usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


class PubTF:
    def __init__(self):
        rospy.init_node('odom_tf_broadcast', anonymous=True)
        self.br = tf2_ros.TransformBroadcaster()
        self.transforStamped = TransformStamped()
        self.transforStamped.header.frame_id = "odom"
        rospy.Subscriber('/odom', Odometry, self.callback)


    def callback(self, msg):
        self.transforStamped.header.stamp = rospy.Time.now()
        self.transforStamped.child_frame_id = msg.child_frame_id
        self.transforStamped.transform.translation.x = msg.pose.pose.position.x
        self.transforStamped.transform.translation.y = msg.pose.pose.position.y
        self.transforStamped.transform.translation.z = msg.pose.pose.position.z
        self.transforStamped.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(self.transforStamped)

def main():
    try:  
        pub_tf= PubTF()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    


if __name__ == "__main__":
    main()