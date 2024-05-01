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
        self.mu, self.sigma = 0, 0.1 
        

        self.desired_state = np.mat([0, 0]).T

        self.control_mat = np.mat([self.GRAVITY/self.LENGTH - 10, -7])

        self.runs = 10
        intial_theta = -np.pi/2
        self.theta = []
        self.colors = []

        for i in range(self.runs):
            self.theta.append([intial_theta])
            self.colors.append(np.random.rand(3,))

        _, self.sim_plot   = plt.subplots(1, figsize = (5,5))
            
    def linear_sim(self, method = 'rk'):
        coeff_mat = np.mat([[0, 1],
                            [-self.GRAVITY/self.MASS, 0]])
          
        input_mat = np.mat([0,1]).T

        states = np.mat([self.theta[0][0], 0]).T
        prev_time = time.time()
        if method == 'rk':
            print("Simulating by Runge Kutta")
            dt = 0.01
            while(True):
                for i in range(self.runs):
                    noise = np.random.normal(self.mu, self.sigma, size= (2,1))
                    temp_states = states
                    temp_states = temp_states + noise
                    dot_states = input_mat*self.control_mat*temp_states

                    k1 = coeff_mat * temp_states + dot_states
                    k2 = coeff_mat * (temp_states + k1 * dt / 2) + dot_states
                    k3 = coeff_mat * (temp_states + k2 * dt / 2) + dot_states
                    k4 = coeff_mat * (temp_states + k3 * dt) + dot_states

                    temp_states = temp_states + (k1 + 2*k2 + 2*k3 + k4) * dt / 6

                    self.theta[i].append(temp_states[0, 0])

                states = temp_states
                self.visualize_pendulum()
                
            

    def visualize_pendulum(self, variable_to_show = 'position'):
            self.sim_plot.clear()
            for i in range(self.runs):
                color = self.colors[i]
                plt.plot(self.theta[i], c = color, label=f'Run {i+1}', )
                plt.xlabel('Iterations')
                plt.ylabel('Angle')
                plt.title(f'Pendulum Angle Over Iterations, Sigma={self.sigma}')
                plt.legend()
        
            plt.pause(0.0004)
        
if __name__ == "__main__":
    pendulum = Pendulum()

    pendulum.linear_sim(method='rk')
