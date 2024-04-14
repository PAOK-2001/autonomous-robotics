import cv2
import math
import time
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

        self.MASS = 1.8 # kg
        self.WHEEL_RADIUS = 0.066 # meters
        self.WHEEL_BASE   = 0.16
        self.INTERIA = 1 /2 * self.MASS * (self.WHEEL_BASE/2) **2

        self.right_torque = 0.1
        self.left_torque  = 0.15

        self.right_force = self.right_torque / self.WHEEL_RADIUS
        self.left_force = self.left_torque / self.WHEEL_RADIUS

        self.force = self.right_force + self.left_force
        self.velocity = 0

        self.x = 0
        self.y = 0
        self.theta = 0

        self.pos_x1 = []
        self.pos_y1 = []

            
        _, self.sim_plot = plt.subplots(1, figsize = (5,5))

    def nonlinear_sim(self):
        prev_time = time.time()
        dot_X2 = 0
        dot_X1 = 0
        dot_X3 = 0
        
        elapsed_time = 0
        while (True):
            if(elapsed_time > 1):
                self.right_torque = 0 
                self.left_torque = 0

            self.right_force = self.right_torque / self.WHEEL_RADIUS
            self.left_force = self.left_torque / self.WHEEL_RADIUS

            self.force = self.right_force + self.left_force

            dt = time.time() - prev_time
            dot_X4 = self.force/self.MASS * np.cos(self.theta) - self.velocity * np.sin(self.theta)*dot_X3
            dot_X5 = self.force/self.MASS * np.sin(self.theta) - self.velocity * np.cos(self.theta)*dot_X3
            dot_X6 = (self.right_force*self.WHEEL_BASE/2 - self.left_force*self.WHEEL_BASE/2)/self.INTERIA
            print(dot_X6)
            
            dot_X3 += dot_X6 * dt  
            dot_X2 += dot_X5 * dt
            dot_X1 += dot_X4 * dt
            self.velocity = math.sqrt(dot_X2**2 + dot_X1**2)

            self.x += dot_X1 * dt
            self.y += dot_X2 * dt
            self.theta += dot_X3 *dt

            self.pos_x1.append(self.x)
            self.pos_y1.append(self.y)

            prev_time = time.time()
            elapsed_time+= dt
            self.visualize_rover()
           
    def linear_sim(self):
        raise NotImplemented

    def visualize_rover(self):
        self.sim_plot.clear()
        
        self.sim_plot.plot(self.x, self.y, 'bo', markersize=5, c= 'r')
        self.sim_plot.plot(self.pos_x1, self.pos_y1, c='r')

        self.sim_plot.set_xlim(-10,10)
        self.sim_plot.set_ylim(-10,10)

        plt.pause(0.0004)        
        
if __name__ == "__main__":
    lineal = False
    rover = Rover()
    if (lineal):
        print("Linear simulation started")
        rover.linear_sim()
    else:
        print("Non linear simulation started")
        rover.nonlinear_sim()        
        