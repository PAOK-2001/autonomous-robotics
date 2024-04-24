#!/usr/bin/env python

import time
import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float32

class RoverControl():
    def __init__(self):
        # Node Params
        rospy.init_node('robot_control_node')

        self.rate = rospy.Rate(100)
        self.x_pub = rospy.Publisher('/x', Float32, queue_size=10)
        self.y_pub = rospy.Publisher('/y', Float32, queue_size=10)
        self.theta_pub = rospy.Publisher('/theta', Float32, queue_size=10)
        self.error_theta_pub = rospy.Publisher('/error_theta', Float32, queue_size=10)
        self.error_pos_pub = rospy.Publisher('/error_pos', Float32, queue_size=10)

        # Constants
        self.r = 0.04 # Wheel radius
        self.h = 0.15 # distance to turning point from wheels
        self.d = 0.10 # wheelbase

        self.q = np.mat([[0,0]]).T #Initial points for the rover
        self.q_dot = np.mat([[0,0]]).T
        self.theta = 0 # Initial angle for the rover
        self.theta_dot = 0

        self.desired_state_dot = np.mat([[0, 0]]).T
        self.desired_state = np.mat([[7, 5]]).T # location from origin in meters as reference (x,y)

        self.phi_mat = np.mat([[self.r/self.d, -self.r/self.d]])

        self.angles = []
        self.pos_x = []
        self.pos_y = []
        self.control = []

        _, self.sim_plot   = plt.subplots(1, figsize = (5,5))
        _, self.angle_plot = plt.subplots(1, figsize = (5,5))
        _, self.control_plot = plt.subplots(1, figsize = (5,5))

            
    def linear_sim(self):
        k_mat = np.mat([[1, 0],
                        [0, 1]])
        prev_time = time.time()
        #print("Simulating by Newton-Euler")
        while(True):
            dt = time.time() - prev_time
            D_mat = self.get_angle_matrix()
            D_mat_inverse = (1/(D_mat[0,0]*D_mat[1,1]-D_mat[1,0]*D_mat[0,1]))*np.mat([[D_mat[1,1],-D_mat[0,1]],
                                                         [-D_mat[1,0],D_mat[0,0]]])
            error = self.desired_state - self.q
            #print(D_mat@D_mat_inverse)
            control_out = D_mat_inverse*(self.desired_state_dot + k_mat*error)
            self.control.append((control_out[0,0]+control_out[1,0])/2)
            self.q_dot = self.get_angle_matrix() * control_out
            self.theta_dot = self.phi_mat*control_out
            self.q = self.q + self.q_dot * dt
            self.theta = self.theta + self.theta_dot[0,0] * dt
            self.angles.append(self.theta)
            
            # Publish (x,y,theta,error)
            self.publish_states(error= error)
            prev_time = time.time()
            self.rate.sleep()
                
            
    def get_angle_matrix(self):
        #print("Theta ", self.theta)
        return np.mat([[self.r/2*np.cos(self.theta)-self.h*self.r/self.d*np.sin(self.theta), self.r/2*np.cos(self.theta)+self.h*self.r/self.d*np.sin(self.theta)],
                      [self.r/2*np.sin(self.theta)-self.h*self.r/self.d*np.cos(self.theta), self.r/2*np.sin(self.theta)+self.h*self.r/self.d*np.cos(self.theta)]])
        


    def publish_states(self, error):
        x_msg, y_msg, theta_msg, angular_error, linear_error, ref_msg = (Float32(), Float32(), Float32(), Float32(), Float32(), Float32())

        x_msg.data = self.q[0,0]
        y_msg.data = self.q[1,0]
        theta_msg.data = self.theta

        linear_error.data = error[0,0]
        angular_error.data = error[1,0]
        
        self.x_pub.publish(x_msg)
        self.y_pub.publish(y_msg)
        self.theta_pub.publish(theta_msg)
        self.error_pos_pub.publish(linear_error)
        self.error_theta_pub.publish(angular_error)
        

if __name__ == "__main__":
    rover = RoverControl()
    rover.linear_sim()
