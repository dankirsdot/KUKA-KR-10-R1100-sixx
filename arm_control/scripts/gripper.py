#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Float64
import numpy as np


def gripper_on():
    # Wait till the srv is available
    rospy.wait_for_service('/kuka_arm/vacuum_gripper/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/kuka_arm/vacuum_gripper/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper_off():
    rospy.wait_for_service('/kuka_arm/vacuum_gripper/off')
    try:
        turn_off = rospy.ServiceProxy('/kuka_arm/vacuum_gripper/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def move_joint(pub, position):
    pub.publish(position)

def move_robot(pub, position, joints_number, time):
    for t in range(time):
        for i in range(joints_number):
            try:
                move_joint(pub[i], position[i])
            except rospy.ROSInterruptException:
                pass
        rate.sleep()

def joint_name(number):
    joint_name = '/kuka_arm/joint' + \
        str(number) + '_position_controller/command'
    return joint_name


if __name__ == '__main__':

    joints = []
    pub = []
    joints_number = 6
    for i in range(joints_number):
        joints.append(joint_name(i+1))
        pub.append(rospy.Publisher(joints[i], Float64, queue_size=10))

    rospy.init_node('joints_talker', anonymous=True)
    rate_value = 50  # 50hz
    rate = rospy.Rate(rate_value)

    time = 100

    gripper_off()

    position = np.array([0, 0, 0, 0, 0, 0])
    move_robot(pub, position, joints_number, time)

    position = np.array([0, -1.85*np.pi/12, 5*np.pi/8, 0, 0, 0])
    move_robot(pub, position, joints_number, time)

    rospy.sleep(0.5)
    gripper_on()
    rospy.sleep(0.5)

    position = np.array([0, -np.pi/2, 5*np.pi/8, 0, 0, 0])
    move_robot(pub, position, joints_number, time)

    position = np.array([0, -np.pi/4, 5*np.pi/16, 0, 0, 0])
    move_robot(pub, position, joints_number, time)

    position = np.array([0, 0, 0, 0, 0, 0])
    move_robot(pub, position, joints_number, time)
    
    gripper_off()
