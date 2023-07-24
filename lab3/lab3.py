#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    
    pub = rospy.Publisher('lab3_topic', String, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg_str = String()
    while not rospy.is_shutdown():
        inp_str = input("Enter a message: ")
        
        msg_str.data = inp_str

        rospy.loginfo(msg_str)
        pub.publish(msg_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
