#!/usr/bin/env python

import math
import time
import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float32

class DoublePendulum():
    def __init__(self):

        rospy.init_node("pendulum_model")
        self.rate = rospy.Rate(100)
        self.theta1_pub = rospy.Publisher('/theta1', Float32, queue_size = 10)
        self.theta2_pub = rospy.Publisher('/theta2', Float32, queue_size = 10)
        self.x1_pub = rospy.Publisher('/x1', Float32, queue_size = 10)
        self.y1_pub = rospy.Publisher('/y1', Float32, queue_size = 10)
        self.x2_pub = rospy.Publisher('/x2', Float32, queue_size = 10)
        self.y2_pub = rospy.Publisher('/y2', Float32, queue_size = 10)

        # Constant
        self.MASS_1 = 1.0 # kilograms
        self.MASS_2 = 1.0
        self.M = self.MASS_1/self.MASS_2
        self.LENGTH_1 = 1.0 # meters
        self.LENGTH_2 = 1.0
        self.GRAVITY = 9.81
        self.SIM_TIME = 1500

        self.theta_1 = 0.523599
        self.theta_2 = - 0.523599

        self.angle_1 = []
        self.angle_2 = []

        self.pos_x1 = []
        self.pos_x2 = []

        self.pos_y1 = []
        self.pos_y2 = []
        
        self.angular_speed_1 = []
        self.angular_speed_2 = []


        _, self.sim_plot = plt.subplots(1, figsize = (5,5))
        # _, self.speed_plot = plt.subplots(1, figsize = (5,5))

    def nonlinear_sim(self):
        X3 = 0
        X4 = 0
        prev_time = time.time()
        while (True):
            dt =time.time() - prev_time
            X3_dot = self.compute_dot_X3(self.theta_1, self.theta_2, X3, X4)
            X4_dot = self.compute_dot_X4(self.theta_1, self.theta_2, X3, X4)
            X4 += X4_dot * dt
            X3 += X3_dot * dt
            self.theta_2 += X4 * dt
            self.theta_1 += X3 * dt

            self.angle_1.append(self.theta_1)
            self.angle_2.append(self.theta_2)
            
            self.angular_speed_1.append(X3)
            self.angular_speed_2.append(X4)

            prev_time = time.time()

            # Publish messages
            theta1_msg = Float32()
            theta1_msg.data = self.theta_1

            theta2_msg = Float32()
            theta2_msg.data = self.theta_2 
            
            x1_msg, y1_msg = Float32(), Float32()
            x2_msg, y2_msg = Float32(), Float32()

            x1 = self.LENGTH_1 * np.sin(self.theta_1) 
            y1 = -1 * self.LENGTH_1 * np.cos(self.theta_1) 

            x2 = x1 + self.LENGTH_2*np.sin(self.theta_2)
            y2 = y1 - self.LENGTH_2*np.cos(self.theta_2)
            
            x1_msg.data = x1
            y1_msg.data = y1

            x2_msg.data = x2
            y2_msg.data = y2

            self.x1_pub.publish(x1_msg)
            self.y1_pub.publish(y1_msg)
            
            self.x2_pub.publish(x2_msg)
            self.y2_pub.publish(y2_msg)

            self.theta1_pub.publish(theta1_msg)
            self.theta2_pub.publish(theta2_msg)
            self.rate.sleep()   
        
    def linear_sim(self):
        coeff_mat = np.mat([[0,0,1,0],
                            [0,0,0,1],
                            [(-self.GRAVITY * (self.MASS_1+self.MASS_2))/(self.LENGTH_1 * self.MASS_1),(-self.GRAVITY * self.MASS_2)/(self.LENGTH_1 * self.MASS_1),0,0],
                            [(self.GRAVITY * (self.MASS_1+self.MASS_2))/(self.LENGTH_1 * self.MASS_1),(-self.GRAVITY * ((self.MASS_1+self.MASS_2)))/(self.LENGTH_1 * self.MASS_1),0,0]])
    
    
        states = np.array([self.theta_1, self.theta_2, 0, 0]).T
        prev_time = time.time()
        while(True):
            dt = time.time() - prev_time
            dot_states = states*coeff_mat
            states = states + dot_states * dt

            self.theta_1 = states[0,0]
            self.theta_2 = states[0,1]

            self.angle_1.append(self.theta_1)
            self.angle_2.append(self.theta_2)
            prev_time = time.time()

            # Publish messages
            theta1_msg = Float32
            theta1_msg.data = self.theta_1

            theta2_msg = Float32
            theta2_msg.data = self.theta_2
            
            x1_msg = Float32()
            y1_msg = Float32()

            x2_msg = Float32()
            y2_msg = Float32()

            x1 = self.LENGTH_1 * np.sin(self.theta_1) 
            y1 = -1 * self.LENGTH_1 * np.cos(self.theta_1) 

            x2 = x1 + self.LENGTH_2*np.sin(self.theta_2)
            y2 = y1 - self.LENGTH_2*np.cos(self.theta_2)

            x1_msg.data = x1
            y1_msg.data = y1

            x2_msg.data = x2
            y2_msg.data = y2

            self.x1_pub.publish(x1_msg)
            self.y1_pub.publish(y1_msg)
            
            self.x2_pub.publish(x2_msg)
            self.y2_pub.publish(y2_msg)
            
            self.theta1_pub.publish(theta1_msg)
            self.theta2_pub.publish(theta2_msg)    

    def linear_sim_RK(self):
        # Define coefficients
        coeff_mat = np.mat([[0, 0, 1, 0],
                            [0, 0, 0, 1],
                            [(-self.GRAVITY*self.MASS_2 - self.GRAVITY*(2*self.MASS_1 + self.MASS_2))/(2*self.LENGTH_1*self.MASS_1),
                             self.GRAVITY*self.MASS_2/(self.LENGTH_1*self.MASS_1)/(self.LENGTH_1 * self.MASS_1), 0, 0],
                            [self.GRAVITY*(self.MASS_1 + self.MASS_2)/(self.LENGTH_2*self.MASS_1),
                             -self.GRAVITY*(self.MASS_1 + self.MASS_2)/(self.LENGTH_2*self.MASS_1), 0, 0]])

        # Initialize states
        states = np.array([[self.theta_1, self.theta_2, 0, 0]])

        prev_time = time.time()
        while True:
            dt = time.time() - prev_time

            # Runge-Kutta integration
            k1 = np.dot(states, coeff_mat)
            k2 = np.dot(states + 0.5*dt*k1, coeff_mat)
            k3 = np.dot(states + 0.5*dt*k2, coeff_mat)
            k4 = np.dot(states + dt*k3, coeff_mat)

            states += (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

            self.theta_1 = states[0, 0]
            self.theta_2 = states[0, 1]

            self.angle_1.append(self.theta_1)
            self.angle_2.append(self.theta_2)

            prev_time = time.time()
            
            # Publish messages
            theta1_msg = Float32
            theta1_msg.data = self.theta_1

            theta2_msg = Float32
            theta2_msg.data = self.theta_2
            
            x1_msg = Float32()
            y1_msg = Float32()

            x2_msg = Float32()
            y2_msg = Float32()

            x1 = self.LENGTH_1 * np.sin(self.theta_1) 
            y1 = -1 * self.LENGTH_1 * np.cos(self.theta_1) 

            x2 = x1 + self.LENGTH_2*np.sin(self.theta_2)
            y2 = y1 - self.LENGTH_2*np.cos(self.theta_2)

            x1_msg.data = x1
            y1_msg.data = y1

            x2_msg.data = x2
            y2_msg.data = y2

            self.x1_pub.publish(x1_msg)
            self.y1_pub.publish(y1_msg)
            
            self.x2_pub.publish(x2_msg)
            self.y2_pub.publish(y2_msg)
            
            self.theta1_pub.publish(theta1_msg)
            self.theta2_pub.publish(theta2_msg)   
    
    def compute_dot_X3(self, X1, X2, X3, X4):
        numerator = math.cos(X1 - X2) * ((self.GRAVITY / self.LENGTH_1) * math.sin(X2) - X3**2 * math.sin(X1 - X2)) - (self.LENGTH_2 / self.LENGTH_1) * ((self.M + 1) * (self.GRAVITY / self.LENGTH_2) * math.sin(X1) + X4**2 * math.sin(X1 - X2))
        denominator = self.M + math.sin(X1 - X2)**2
        dot_X3 = numerator / denominator
        return dot_X3

    def compute_dot_X4(self, X1, X2, X3, X4):
        numerator = math.cos(X1 - X2) * ((self.M + 1) * (self.GRAVITY / self.LENGTH_2) * math.sin(X1) + X4**2 * math.sin(X1 - X2)) - (self.M + 1) * (self.LENGTH_1 / self.LENGTH_2) * ((self.GRAVITY / self.LENGTH_1) * math.sin(X2) - X3**2 * math.sin(X1 - X2))
        denominator = self.M + math.sin(X1 - X2)**2
        dot_X4 = numerator / denominator
        return dot_X4
        
if __name__ == "__main__":
    lineal = False
    pendulum = DoublePendulum()
    if (lineal):
        print("Linear simulation started")
        pendulum.linear_sim()
    else:
        print("Non linear simulation started")
        pendulum.nonlinear_sim()        
        