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
        self.mu, self.sigma = 0, 0.1 

        self.q_dot = np.mat([[0,0]]).T
        self.theta_dot = 0

        self.desired_state_dot = np.mat([[0, 0]]).T
        self.desired_state = np.mat([[7, 8]]).T # location from origin in meters as reference (x,y)

        self.phi_mat = np.mat([[self.r/self.d, -self.r/self.d]])

        self.runs = 10
        intial_theta = np.pi/2 # Initial angle for the rover

        self.theta = []
        self.q = np.mat([[0,0]]).T #Initial points for the rover
        self.colors = []

        for i in range(self.runs):
            self.theta.append([intial_theta])
            self.colors.append(np.random.rand(3,))

        _, self.sim_plot   = plt.subplots(1, figsize = (5,5))
            
    def linear_sim(self):
        k_mat = np.mat([[1, 0],
                        [0, 1]])
        prev_time = time.time()
        print("Simulating by Newton-Euler")
        dt = 0.01
        while(True):
            for i in range(self.runs):
                noise = np.random.normal(self.mu, self.sigma)
                D_mat = self.get_angle_matrix(theta= self.theta[i][-1])
                D_mat_inverse = (1/(D_mat[0,0]*D_mat[1,1]-D_mat[1,0]*D_mat[0,1]))*np.mat([[D_mat[1,1],-D_mat[0,1]],
                                                                                        [-D_mat[1,0],D_mat[0,0]]])
                error = (self.desired_state - self.q)
                control_out = D_mat_inverse*(self.desired_state_dot + k_mat*error)
                self.q_dot = D_mat * control_out
                self.theta_dot = self.phi_mat*control_out
                self.q = self.q + self.q_dot * dt
                self.theta[i].append(self.theta[i][-1] + self.theta_dot[0,0] * dt + noise)
            
            self.visualize_position()           
            
    def get_angle_matrix(self, theta):
        #print("Theta ", self.theta)
        return np.mat([[(self.r/2)*np.cos(theta)-((self.h*self.r)/self.d)*np.sin(theta), (self.r/2)*np.cos(theta)+((self.h*self.r)/self.d)*np.sin(theta)],
                       [(self.r/2)*np.sin(theta)+((self.h*self.r)/self.d)*np.cos(theta), (self.r/2)*np.sin(theta)-((self.h*self.r)/self.d)*np.cos(theta)]])
        
    def visualize_position(self):
        self.sim_plot.clear()
        for i in range(self.runs):
            color = self.colors[i]
            plt.plot(self.theta[i], c = color, label=f'Run {i+1}', )
            plt.xlabel('Iterations')
            plt.ylabel('Angle')
            plt.title(f'Rover Angle Over Iterations, Sigma={self.sigma}')
            plt.legend()

        plt.pause(0.0004)
        
if __name__ == "__main__":
    rover = Rover()

    rover.linear_sim()
