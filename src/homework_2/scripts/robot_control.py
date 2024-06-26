import time
import matplotlib

import math
import time
import numpy as np
import matplotlib.pyplot as plt


class Rover():
    def __init__(self):
        plt.ion()
        # Constants
        self.r = 0.04 # Wheel radius
        self.h = 0.15 # distance to turning point from wheels
        self.d = 0.10 # wheelbase

        self.q = np.mat([[0,0]]).T #Initial points for the rover
        self.q_dot = np.mat([[0,0]]).T
        self.theta = 0 # Initial angle for the rover
        self.theta_dot = 0

        self.desired_state_dot = np.mat([[0, 0]]).T
        self.desired_state = np.mat([[7, 8]]).T # location from origin in meters as reference (x,y)

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
        print("Simulating by Newton-Euler")
        while(True):
            dt = time.time()-prev_time
            D_mat = self.get_angle_matrix()
            D_mat_inverse = (1/(D_mat[0,0]*D_mat[1,1]-D_mat[1,0]*D_mat[0,1]))*np.mat([[D_mat[1,1],-D_mat[0,1]],
                                                                                      [-D_mat[1,0],D_mat[0,0]]])
            error = (self.desired_state - self.q)
            control_out = D_mat_inverse*(self.desired_state_dot + k_mat*error)
            self.control.append((control_out[0,0]+control_out[1,0])/2)
            self.q_dot = D_mat * control_out
            self.theta_dot = self.phi_mat*control_out
            self.q = self.q + self.q_dot * dt
            self.theta = self.theta + self.theta_dot[0,0] * dt
            self.angles.append(self.theta)
            prev_time = time.time()
            self.visualize_position()
                
            
    def get_angle_matrix(self):
        #print("Theta ", self.theta)
        return np.mat([[(self.r/2)*np.cos(self.theta)-((self.h*self.r)/self.d)*np.sin(self.theta), (self.r/2)*np.cos(self.theta)+((self.h*self.r)/self.d)*np.sin(self.theta)],
                       [(self.r/2)*np.sin(self.theta)+((self.h*self.r)/self.d)*np.cos(self.theta), (self.r/2)*np.sin(self.theta)-((self.h*self.r)/self.d)*np.cos(self.theta)]])
        


    def visualize_position(self):
        
        self.sim_plot.clear()
        self.angle_plot.clear()
        self.control_plot.clear()
        
        x1 = self.q[0,0]
        y1 = self.q[1,0]
        self.pos_x.append(x1)
        self.pos_y.append(y1)
        self.sim_plot.plot([0, x1], [0, y1], lw=2, c='k') 
        self.sim_plot.plot(x1, y1, 'bo', markersize=10, c= '#222E50')
        self.sim_plot.set_xlabel("x (m)")
        self.sim_plot.set_ylabel("y (m)")
        self.sim_plot.set_title("Position of robot")
        self.sim_plot.grid(True,"both")
        # self.sim_plot.plot(self.pos_x, c='r')
        # self.sim_plot.plot(self.pos_y, c='b')
        # self.sim_plot.plot(self.desired_state[0,0], self.desired_state[1,0], 'bo')
        self.angle_plot.plot(self.angles)
        self.angle_plot.set_ylabel("theta (rads)")
        self.angle_plot.set_xlabel("time (s)")
        self.angle_plot.set_title("Angle of robot")
        self.angle_plot.grid(True,"both")
        self.control_plot.plot(self.control)
        self.control_plot.set_xlabel("time (s)")
        self.control_plot.set_ylabel("linear velocity (m/s)")
        self.control_plot.set_title("Linear velocity")
        self.control_plot.grid(True,"both")
        self.sim_plot.set_xlim(-10,10)
        self.sim_plot.set_ylim(-10,10)

        plt.pause(0.0004)
        
if __name__ == "__main__":
    rover = Rover()

    rover.linear_sim()
