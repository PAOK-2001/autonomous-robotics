import cv2
import math
import time
import atexit
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.offsetbox import AnnotationBbox
from matplotlib.offsetbox import OffsetImage
from scipy import ndimage

def plot_agent(asset, heading, zoom = 0.015):
    img = ndimage.rotate(asset, heading) 
    img = np.fliplr(img) 
    img = OffsetImage(img, zoom=zoom)
    return img

class Rover:
    def __init__(self):
        plt.ion()
        #Turtle bot waffle pi
        self.WHEEL_RADIUS = 0.066 # meters
        self.WHEEL_BASE   = 0.16

        self.x = 0
        self.y = 0
        self.theta = np.pi/4
        self.velocity = 0
 
        self.pos_x1 = []
        self.pos_y1 = []
            
        _, self.sim_plot = plt.subplots(1, figsize = (5,5))

    def nonlinear_sim(self):
        prev_time = time.time()
        dot_X2 = 0
        dot_X1 = 0
        dot_X3 = 0
        
        while (True):
            left_velocity = 0.5
            right_velocity = 0.4
            
            self.velocity = (right_velocity + left_velocity)/2
            
            dt = time.time() - prev_time

            dot_X1 = self.velocity*np.cos(self.theta)
            dot_X2 = self.velocity*np.sin(self.theta)
            dot_X3 = (right_velocity - left_velocity) / self.WHEEL_BASE

            self.x += dot_X1 *dt
            self.y += dot_X2 *dt
            self.theta += dot_X3 *dt

            if(self.theta < 0): self.theta = self.theta + 2*np.pi

            self.pos_x1.append(self.x)
            self.pos_y1.append(self.y)

            prev_time = time.time()
            self.visualize_rover()
           
    def linear_sim(self):
        prev_time = time.time()
  
        mat_A = np.zeros([3,3])

        # Since all points for a rover are stable, we evaluate the input matrix for our intial conditions
        mat_B = np.mat([[np.cos(self.theta),0],
                        [np.sin(self.theta),0],
                        [0,1]])
        while (True):
            dt = time.time() - prev_time
            left_velocity = 0.5
            right_velocity = 0.5
            velocity = (right_velocity + left_velocity)/2
            angular_speed = (right_velocity - left_velocity) / self.WHEEL_BASE
            
            states = np.mat([self.x, self.y, self.theta]).T
            inputs = np.mat([velocity , angular_speed]).T

            dot_states = mat_A * states + mat_B*inputs
            states = states + dot_states * dt


            self.x = states[0,0]
            self.y = states[1,0]
            self.theta = states[2,0]

            # breakpoint()
            if(self.theta < 0): self.theta = self.theta + 2*np.pi

            self.pos_x1.append(self.x)
            self.pos_y1.append(self.y)

            prev_time = time.time()
            self.visualize_rover()

    def visualize_rover(self):
        self.sim_plot.clear()
        
        self.sim_plot.plot(self.x, self.y, 'bo', markersize=5, c= 'r')
        self.sim_plot.plot(self.pos_x1, self.pos_y1, c='r')

        self.sim_plot.set_xlim(-10,10)
        self.sim_plot.set_ylim(-10,10)
        plt.grid()

        plt.pause(0.0004)        
        
if __name__ == "__main__":
    lineal = True
    rover = Rover()
    if (lineal):
        print("Linear simulation started")
        rover.linear_sim()
    else:
        print("Non linear simulation started")
        rover.nonlinear_sim()        
        