#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

import numpy as np
import math as mt

class Mobile_robot():
    def __init__(self):
        # Ros node
        rospy.init_node("rover_simulation")

        # Torques de cada uno de los motores
        self.Tr = 0.0
        self.Tl = 0.0
        self.r = 0.0 # Radius of the robot
        self.theta = 0.0
        self.w = 0.0
        self.v = 0.0

    def model_rover(self, m): # Pasar masa como argumento
        # Calculate forces 
        Fr = self.Tr / self.r
        Fl = self.Tl / self.r

        # Generate robot forces
        F = Fr + Fl

        # Calculate speed, aceleration and position in x
        x = self.v * mt.sin(self.theta)
        vx = self.v * mt.cos(self.theta) 
        ax = (F / m) * mt.cos(self.theta) - vx * mt.sin(self.theta) * self.w

        # Calculate speed, aceleration and position for y
        y = self.v * mt.cos(self.theta)
        vy = self.v * mt.sin(self.theta)
        ay = (F / m) * mt.sin(self.theta) + vy * mt.cos(self.theta) * self.w

        # Publish v, a and x


if __name__=='__main__':
    mobile = Mobile_robot()
