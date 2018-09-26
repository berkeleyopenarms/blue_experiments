#!/usr/bin/env python
import time
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from blue_hardware_drivers.msg import MotorState

def callback(msg):
    print msg

class Base:
    def __init__(self):

        rospy.init_node('Base_Calibration', anonymous=True)
        rospy.Subscriber("hi", JointState, callback)
        while(True):
            rospy.sleep(1)

if __name__ == '__main__':
    link = Base()
