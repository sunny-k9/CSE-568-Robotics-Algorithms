#!/usr/bin/env python
import rospy
import random
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

range_data = []
thresh = 1.0


def move_bot(data):
    range_data = np.array(data.ranges)
    twist = Twist()
    pub = rospy.Publisher("robot_0/cmd_vel", Twist, queue_size=1)

    rate = rospy.Rate(10)
    bool = 0
    if np.shape(range_data)[0] > 0:
        # print(np.min(range_data))

        if np.min(range_data[120:240]) > 2:

            twist.linear.x = 2.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
        else:

            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0


            twist.angular.z = random.uniform(0, 3.14)



    pub.publish(twist)
    # time.sleep(2)



if __name__ == "__main__":
    rospy.init_node("evader", anonymous=True)
    rospy.Subscriber("robot_0/base_scan", LaserScan, move_bot)
    rospy.spin()
