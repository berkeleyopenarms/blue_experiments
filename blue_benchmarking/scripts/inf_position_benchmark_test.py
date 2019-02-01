#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Float64MultiArray
import numpy as np
import scipy
import time

amplitude = 0.2
half_rate = 1

def p():
        rospy.init_node('p')
        pub = rospy.Publisher("/blue_controllers/joint_position_controller/command", Float64MultiArray, queue_size=1)
        rate = rospy.Rate(1/half_rate)
        count = 0
        while not rospy.is_shutdown():
                cmd = Float64MultiArray()
                cmd.data = [-amplitude, 0]
                pub.publish(cmd)
                rate.sleep()

                cmd = Float64MultiArray()
                cmd.data = [amplitude, 0]
                pub.publish(cmd)
                rate.sleep()
                count += 1
                rospy.logerr("count = {}".format(count))

if __name__ == '__main__':
        try:
                p()
        except rospy.ROSInteruptException:
                pass
