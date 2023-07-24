#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

ub_id = 50428194
def callback(data):
    if(data.data == ub_id):
        print("true")
    else:
        print("false")
        
    
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("lab4_topic", Int32, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
