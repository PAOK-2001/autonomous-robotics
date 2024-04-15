
import math
import time
import numpy as np
import matplotlib.pyplot as plt

class DoublePendulum():
    def __init__(self):
        plt.ion()
        # Constant
        self.MASS_1 = 1.0 # kilograms
        self.MASS_2 = 0.5
        self.M = self.MASS_1/self.MASS_2
        self.LENGTH_1 = 1.0 # meters
        self.LENGTH_2 = 1.0
        self.GRAVITY = 9.81
        self.SIM_TIME = 1500

        self.theta_1 = -np.pi/2
        self.theta_2 = -3*np.pi/2

        self.angle_1 = []
        self.angle_2 = []

        self.pos_x1 = []
        self.pos_x2 = []

        self.pos_y1 = []
        self.pos_y2 = []
        
        self.angular_speed_1 = []
        self.angular_speed_2 = []

        _, self.sim_plot   = plt.subplots(1, figsize = (5,5))

    def nonlinear_sim(self):
        X3 = 0
        X4 = 0
        prev_time = time.time()
        while (True):
            dt = time.time() - prev_time
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

            self.visualize_pendulum(variable_to_show = "angle")
            
    def linear_sim(self):
        coeff_mat = np.mat([[0,0,1,0],
                            [0,0,0,1],
                            [(-self.GRAVITY*self.MASS_2 - self.GRAVITY*(2*self.MASS_1 + self.MASS_2))/(2*self.LENGTH_1*self.MASS_1),self.GRAVITY*self.MASS_2/(self.LENGTH_1*self.MASS_1)/(self.LENGTH_1 * self.MASS_1),0,0],
                            [self.GRAVITY*(self.MASS_1 + self.MASS_2)/(self.LENGTH_2*self.MASS_1),-self.GRAVITY*(self.MASS_1 + self.MASS_2)/(self.LENGTH_2*self.MASS_1),0,0]])
    
    
        states = np.mat([self.theta_1, self.theta_2, 0, 0]).T
        prev_time = time.time()
        while(True):
            dt = time.time() - prev_time
            dot_states = coeff_mat*states
            states = states + dot_states * dt

            self.theta_1 = states[0,0]
            self.theta_2 = states[1,0]

            self.angle_1.append(self.theta_1)
            self.angle_2.append(self.theta_2)
            prev_time = time.time()
            self.visualize_pendulum()
            
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
            self.visualize_pendulum()
    
    def compute_dot_X3(self, X1, X2, X3, X4):
        numerator = -self.GRAVITY * (2 * self.MASS_1 + self.MASS_2) * math.sin(X1) - self.MASS_2 * self.GRAVITY * math.sin(X1 - 2 * X2) - 2 * math.sin(X1 - X2) * self.MASS_2 * (X4 ** 2 * self.LENGTH_2 + X3 ** 2 * self.LENGTH_1 * math.cos(X1 - X2))
        denominator = self.LENGTH_1 * (2 * self.MASS_1 + self.MASS_2 - self.MASS_2 * math.cos(2 * X1 - 2 * X2))
        dot_X3 = numerator / denominator
        return dot_X3

    def compute_dot_X4(self, X1, X2, X3, X4):
        numerator = 2 * math.sin(X1 - X2) * ((X3 ** 2) * self.LENGTH_1 * (self.MASS_1 + self.MASS_2) + self.GRAVITY * (self.MASS_1 + self.MASS_2) * math.cos(X1) + (X4 ** 2) * self.LENGTH_2 * self.MASS_2 * math.cos(X1 - X2))
        denominator = self.LENGTH_2 * (2 * self.MASS_1 + self.MASS_2 - self.MASS_2 * math.cos(2 * X1 - 2 * X2))
        dot_X4 = numerator / denominator
        return dot_X4

    def visualize_pendulum(self, variable_to_show = 'position'):
        if variable_to_show == 'position':
        
            self.sim_plot.clear()
            
            x1 = self.LENGTH_1 * np.sin(self.theta_1) 
            y1 = -1 * self.LENGTH_1 * np.cos(self.theta_1) 

            x2 = x1 + self.LENGTH_2*np.sin(self.theta_2)
            y2 = y1 - self.LENGTH_2*np.cos(self.theta_2)

            self.pos_x1.append(x1)
            self.pos_x2.append(x2)

            self.pos_y1.append(y1)
            self.pos_y2.append(y2)

            self.sim_plot.plot([0, x1], [0, y1], lw=2, c='k')
            self.sim_plot.plot([x1, x2], [y1, y2], lw=2, c='k')

            # Plot the bobs
            max_l = self.LENGTH_1 + self.LENGTH_2
            self.sim_plot.plot(x1, y1, 'bo', markersize=10, c= '#222E50')
            self.sim_plot.plot(self.pos_x1, self.pos_y1, c='#BCD8C1', alpha = 0.7)
            self.sim_plot.plot(x2, y2, 'ro', markersize=10, c = '#007991')
            self.sim_plot.plot(self.pos_x2, self.pos_y2, c='#439a86')
            self.sim_plot.set_xlim(-max_l-0.5,max_l + 0.5)
            self.sim_plot.set_ylim(-max_l-0.5,max_l + 0.5)
        
        else:
            self.sim_plot.plot(self.angle_1, self.angle_2, lw=2, c='#007991')
            self.sim_plot.set_title('Theta1 vs theta2')
            self.sim_plot.set_xlabel('Theta 1 (radians)')
            self.sim_plot.set_ylabel('Theta 2 (Radians)')
    



        plt.pause(0.0004)
        
if __name__ == "__main__":
    lineal = True
    method = 'rk'
    pendulum = DoublePendulum()
    if (lineal):
        print("Linear simulation started")
        if method == 'rk':
            pendulum.linear_sim_RK()
        else:
            pendulum.linear_sim()
    else:
        print("Non linear simulation started")
        pendulum.nonlinear_sim()        
