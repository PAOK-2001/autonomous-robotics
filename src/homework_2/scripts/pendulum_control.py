import time
import matplotlib

import math
import time
import numpy as np
import matplotlib.pyplot as plt


class Pendulum():
    def __init__(self):
        plt.ion()
        # Constant
        self.LENGTH = 1.0 # meters
        self.MASS = 1.0 # kilograms
        self.GRAVITY = 9.81


        self.desired_state = np.mat([-np.pi/2, 0]).T

        self.control_mat = np.mat([self.GRAVITY/self.LENGTH - 317, -13])
        self.theta = np.pi/2

        _, self.sim_plot   = plt.subplots(1, figsize = (5,5))
            
    def linear_sim(self, method = 'rk'):
        coeff_mat = np.mat([[0, 1],
                            [-self.GRAVITY/self.MASS, 0]])
          
        input_mat = np.mat([0,1]).T

        states = np.mat([self.theta, 0]).T
        prev_time = time.time()
        if method == 'rk':
            print("Simulating by Runge Kutta")
            while(True):
                dt = time.time() - prev_time
                error = states - self.desired_state
                dot_states = input_mat*self.control_mat*error
                print(error[0,0])

                k1 = coeff_mat * states + dot_states
                k2 = coeff_mat * (states + k1 * dt / 2) + dot_states
                k3 = coeff_mat * (states + k2 * dt / 2) + dot_states
                k4 = coeff_mat * (states + k3 * dt) + dot_states

                states = states + (k1 + 2*k2 + 2*k3 + k4) * dt / 6

                self.theta = states[0, 0]

                prev_time = time.time()
                self.visualize_pendulum()

        else:
            print("Simulating by Newton-Euler")
            while(True):
                dt = time.time() - prev_time
                error = states - self.desired_state
                print(error[0,0])
                dot_states = coeff_mat*states + input_mat*self.control_mat*error
                states = states + dot_states *dt

                self.theta = states[0,0]

                prev_time = time.time()
                self.visualize_pendulum()
                
            

    def visualize_pendulum(self, variable_to_show = 'position'):
        
            self.sim_plot.clear()
            
            x1 = self.LENGTH * np.sin(self.theta) 
            y1 = -1 * self.LENGTH * np.cos(self.theta)
            self.sim_plot.plot([0, x1], [0, y1], lw=2, c='k') 
            self.sim_plot.plot(x1, y1, 'bo', markersize=10, c= '#222E50')   
            self.sim_plot.set_xlim(-self.LENGTH-0.5,self.LENGTH + 0.5)
            self.sim_plot.set_ylim(-self.LENGTH-0.5,self.LENGTH + 0.5)

            plt.pause(0.0004)
        
if __name__ == "__main__":
    pendulum = Pendulum()

    pendulum.linear_sim(method='rk')
