#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import math
 
def talker():
    pub = rospy.Publisher('/kuka_arm/joint1_position_controller/command', Float64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        position = math.pi/2
        #rospy.loginfo(position)
        pub.publish(position)
        rate.sleep()
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass