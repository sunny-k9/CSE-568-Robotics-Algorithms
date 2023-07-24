#!/usr/bin/env python
import rospy
import random
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import math

range_data = []
states = ["goal_seek", "wall_follow"]
current_state = states[0]
orien = 0
pos = 0
initial_pos = Point()
initial_pos.x = -8
initial_pos.y = -2
moved_from_line = False


def callback(data):
    global orien
    global pos
    pos = data.pose.pose.position
    orien = data.pose.pose.orientation


def move_bot(data):
    # print("aaa")
    global current_state, states, orien, pos, moved_from_line
    range_data = np.array(data.ranges)
    twist = Twist()
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/odom", Odometry, callback)

    goal = Point()
    goal.x = rospy.get_param("goal_pos_x")
    goal.y = rospy.get_param("goal_pos_y")
    rate = rospy.Rate(10)
    bool = 0

    if np.shape(range_data)[0] > 0 and pos != 0:
        goal_yaw = math.atan2(goal.y - pos.y, goal.x - pos.x)

        d = abs(((goal.x - initial_pos.x) * (initial_pos.y - pos.y)) - (
                (initial_pos.x - pos.x) * (goal.y - initial_pos.y))) / math.sqrt(
            math.pow(goal.x - initial_pos.x, 2) + math.pow(goal.y - initial_pos.y, 2))
        # print(d)
        # print(current_state)
        distance_from_goal = math.sqrt(math.pow(goal.x - pos.x, 2) + math.pow(goal.y - pos.y, 2))
        if distance_from_goal > 1:
            if current_state == "goal_seek":
                if abs(goal_yaw - orien.z) > 0.3:
                    print("follow line")
                    print(goal_yaw - orien.z)
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.linear.z = 0.0
                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = 0.3

                elif np.min(range_data[120:240]) > 1.2:

                    print("follow line")
                    print(goal_yaw - orien.z)
                    twist.linear.x = 2.0
                    twist.linear.y = 0.0
                    twist.linear.z = 0.0
                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = 0.0

                else:

                    current_state = "wall_follow"
                    print(current_state)
            else:  # wall follow

                if np.min(range_data[160:200]) < 1:  # turn right if wall encountered
                    print("front wall")
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.linear.z = 0.0
                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = -0.5
                else:
                    if (min(range_data[200:360]) < 0.7):
                        print("wall follow straight")
                        twist.linear.x = 1.0
                        twist.linear.y = 0.0
                        twist.linear.z = 0.0
                        twist.angular.x = 0.0
                        twist.angular.y = 0.0
                        twist.angular.z = -0.5
                    elif min(range_data[200:330]) > 1.2:  # turn left if wall lost to continue following wall
                        print("left wall")
                        twist.linear.x = 0.0
                        twist.linear.y = 0.0
                        twist.linear.z = 0.0
                        twist.angular.x = 0.0
                        twist.angular.y = 0.0
                        twist.angular.z = 0.5
                    else:  # keep going straight and follow wall
                        twist.linear.x = 1.0
                        twist.linear.y = 0.0
                        twist.linear.z = 0.0
                        twist.angular.x = 0.0
                        twist.angular.y = 0.0
                        twist.angular.z = 0.0
                    if d > 0.3:
                        moved_from_line = True

                    if d < 0.3 and moved_from_line == True:
                        current_state = "goal_seek"
                        moved_from_line = False

        pub.publish(twist)
        # time.sleep(2)


if __name__ == "__main__":
    try:
        rospy.init_node("bug2", anonymous=True)
        rospy.Subscriber("base_scan", LaserScan, move_bot)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
