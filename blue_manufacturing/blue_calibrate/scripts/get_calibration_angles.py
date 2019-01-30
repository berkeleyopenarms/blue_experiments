#!/usr/bin/env python
import time
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from blue_msgs.msg import MotorState

class Arm:
    def __init__(self, side):
        rospy.init_node('Calibration', anonymous=True)
        rate = rospy.Rate(200)
        self.motor_pos = [0.0 for i in range(8)]
        self.motor_names = rospy.get_param(side + "_arm/blue_hardware/motor_names")
        rospy.Subscriber(side + "_arm/blue_hardware/motor_states", MotorState, self.update_motors)

    def update_motors(self, msg):
        for i, n in enumerate(msg.name):
            for j, j_name in enumerate(self.motor_names):
                if j_name == n:
                    self.motor_pos[j] = msg.position[i]

    def get_motor_state(self):
        print(self.motor_names)
        print(self.motor_pos)
        return self.motor_pos

gr1 = 7.1875
gr2 = 8.2444852941

rot1 = 2.189
rot2 = np.pi/2
if __name__ == '__main__':
    arm = Arm("left")

    raw_input("Press enter to save base state ")
    ms = arm.get_motor_state()
    a0 = ms[0]

    raw_input("Press enter to save first link state")
    ms = arm.get_motor_state()
    a1 = ms[1] +  rot1 * gr1 + rot2 * gr2
    a2 = ms[2] + -rot1 * gr1 + rot2 * gr2

    raw_input("Press enter to save second link state")
    ms = arm.get_motor_state()
    a3 = ms[3] +  rot1 * gr1 + rot2 * gr2
    a4 = ms[4] + -rot1 * gr1 + rot2 * gr2

    raw_input("Press enter to save thrid link state")
    ms = arm.get_motor_state()
    a5 = ms[5] +  rot1 * gr1 + rot2 * gr2
    a6 = ms[6] + -rot1 * gr1 + rot2 * gr2

    actuators = [a0, a1, a2, a3, a4, a5, a6]
    print("raw")
    print(actuators)

    # actuators = [ ( (a+np.pi) % 2*np.pi) - np.pi for a in actuators]
    actuators_wrapped = []
    for a in actuators:
        while a > np.pi:
            a -= 2 * np.pi
        while a < -np.pi:
            a += 2 * np.pi
        actuators_wrapped.append(a)

    print("Save the following to the yaml configuration file")
    print(actuators_wrapped)

