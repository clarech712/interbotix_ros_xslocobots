#!/usr/bin/env python

# Below are all the topics with "command" in them.
#
# /locobot/arm_controller/command
# /locobot/cmd_vel : geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0},
# angular: {x: 0.0, y: 0.0, z: 0.0}}'
# /locobot/gripper_controller/command
# /locobot/pan_controller/command : std_msgs/Float64 "data: 1.0"
# /locobot/tilt_controller/command : std_msgs/Float64 "data: 1.0"
#
# My aim is to send random messages to all of them.

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import numpy as np

# function for generating custom random so that I only need to toggle once
def custom_random():
    return np.random.uniform(0, 2) - 1

# motion of the base of the robot around the maze
def cmd_vel(pub):
    msg = Twist()

    msg.linear.x = custom_random()
    msg.linear.y = custom_random()
    msg.linear.z = custom_random()

    msg.angular.x = custom_random()
    msg.angular.y = custom_random()
    msg.angular.z = custom_random()

    rospy.loginfo(msg)
    pub.publish(msg)

# tilt and pan of the camera
# probably not very useful
def pan_controller(pub):
    msg = Float64()

    msg.data = custom_random()

    rospy.loginfo(msg)
    pub.publish(msg)

def tilt_controller(pub):
    msg = Float64()

    msg.data = custom_random()

    rospy.loginfo(msg)
    pub.publish(msg)

def talker():
    # initialize individual publishers
    cmd_vel_pub = rospy.Publisher('/locobot/cmd_vel', Twist, queue_size=1)
    pan_controller_pub = rospy.Publisher('/locobot/pan_controller/command', Float64, queue_size=1)
    tilt_controller_pub = rospy.Publisher('/locobot/tilt_controller/command', Float64, queue_size=1)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        # call individual publishing functions
        cmd_vel(cmd_vel_pub)
        pan_controller(pan_controller_pub)
        tilt_controller(tilt_controller_pub)

        rospy.loginfo('Msg published.')
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass