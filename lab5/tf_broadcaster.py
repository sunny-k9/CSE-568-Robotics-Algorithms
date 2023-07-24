#!/usr/bin/env python
# Code referenced from http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
import rospy
import tf
from nav_msgs.msg import Odometry
import tf_conversions
import tf2_ros
import geometry_msgs.msg

def handle_odometry(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    twist = msg.twist.twist
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "robot_0"
    t.transform.translation.x = twist.linear.x
    t.transform.translation.y = twist.linear.y
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, twist.angular.z)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    rospy.Subscriber('robot_0/odom',
                     Odometry,
                     handle_odometry)


    rospy.spin()
