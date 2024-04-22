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
        self.theta = 3*np.pi/2 # Initial angle for the rover
        self.theta_dot = 0

        self.desired_state_dot = np.mat([[0, 0]]).T
        self.desired_state = np.mat([[2, 2]]).T # location from origin in meters as reference (x,y)

        self.phi_mat = np.mat([[self.r/self.d, -self.r/self.d]])

        _, self.sim_plot   = plt.subplots(1, figsize = (5,5))
            
    def linear_sim(self):
        k_mat = np.mat([[1, 0],
                       [0, 1]])

        prev_time = time.time()
        print("Simulating by Newton-Euler")
        
        while(True):
            dt = time.time() - prev_time
            error = self.desired_state-self.q
            self.q_dot = self.desired_state_dot + k_mat*(error)
            D_mat = self.get_angle_matrix()
            self.theta_dot = self.phi_mat*np.linalg.inv(D_mat)*(self.desired_state_dot + k_mat*error)
            print(self.theta_dot)
            self.q = self.q + self.q_dot *dt
            self.theta += self.theta + self.theta_dot[0,0] * dt

            prev_time = time.time()
            self.visualize_position()
                
            
    def get_angle_matrix(self):
        return np.mat([[self.r/2*np.cos(self.theta)-self.h*self.r/self.d*np.sin(self.theta), self.r/2*np.cos(self.theta)+self.h*self.r/self.d*np.sin(self.theta)],
                      [self.r/2*np.sin(self.theta)-self.h*self.r/self.d*np.cos(self.theta), self.r/2*np.sin(self.theta)+self.h*self.r/self.d*np.cos(self.theta)]])
        

    def visualize_position(self):
        
        self.sim_plot.clear()
        
        x1 = self.q[0,0]
        y1 = self.q[1,0]
        self.sim_plot.plot([0, x1], [0, y1], lw=2, c='k') 
        self.sim_plot.plot(x1, y1, 'bo', markersize=10, c= '#222E50')
        self.sim_plot.plot(self.desired_state[0,0], self.desired_state[1,0], 'bo')
        self.sim_plot.set_xlim(-10,10)
        self.sim_plot.set_ylim(-10,10)

        plt.pause(0.0004)
        
if __name__ == "__main__":
    rover = Rover()

    rover.linear_sim()
