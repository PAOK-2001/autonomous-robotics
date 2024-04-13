#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

import numpy as np
import math as mt

class Mobile_robot():
    def __init__(self):
        # Ros node
        rospy.init_node("rover_model")

        # Mobile robot variables
        self.x = 0.0
        self.y = 0.0
        self.r = 0.0
        self.theta = 0.0
        self.r = 0.0
        self.v = 0.0
        self.a = 0.0
        self.m = 0.0

        # Fuerzas longitudinales
        self.Fl = 0.0
        self.Fr = 0.0

    def motion_robot(self):
        # Calculate robot force
        F = self.Fl + self.Fr

        # Generar fuerza del robot y movimiento del robot
        self.x = self.v * mt.cos(self.theta)
        self.a = self.v * mt.cos(self.theta) - self.v * mt.sin(self.theta) * self.w

        # Para la posicion de y
        self.y = (F / self.m) * mt.sin(self.theta) + self.v * mt.cos(self.theta) * self.w


if __name__=='__main__':
    mobile = Mobile_robot()
