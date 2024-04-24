import time
import matplotlib
import math
import time
import numpy as np
import matplotlib.pyplot as plt

import rospy
from std_msgs.msg import Float32

class Pendulum():
    def __init__(self):
        plt.ion()

        rospy.init_node("pendulum_model")
        self.rate = rospy.Rate(100)
        self.theta_pub = rospy.Publisher('/theta', Float32, queue_size = 10)
        self.x_pub = rospy.Publisher('/x', Float32, queue_size = 10)
        self.y_pub = rospy.Publisher('/y', Float32, queue_size = 10)
        self.error_pub = rospy.Publisher('/error', Float32, queue_size = 10)
        self.ref_pub = rospy.Publisher('/ref', Float32, queue_size = 10)

        # Constant
        self.LENGTH = 1.0 # meters
        self.MASS = 1.0 # kilograms
        self.GRAVITY = 9.81


        self.desired_state = np.mat([0, 0]).T

        self.control_mat = np.mat([self.GRAVITY/self.LENGTH - 10, -7])
        self.theta = -np.pi/2

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
                dot_states = input_mat*self.control_mat*states

                k1 = coeff_mat * states + dot_states
                k2 = coeff_mat * (states + k1 * dt / 2) + dot_states
                k3 = coeff_mat * (states + k2 * dt / 2) + dot_states
                k4 = coeff_mat * (states + k3 * dt) + dot_states

                states = states + (k1 + 2*k2 + 2*k3 + k4) * dt / 6

                self.theta = states[0, 0]

                self.publish_states(error=error[0,0])
                prev_time = time.time()
                self.rate.sleep()

        else:
            print("Simulating by Newton-Euler")
            while(True):
                dt = time.time() - prev_time
                error = states - self.desired_state
                print(error[0,0])
                dot_states = coeff_mat*states + input_mat*self.control_mat*states
                states = states + dot_states *dt

                self.theta = states[0,0]

                self.publish_states(error=error[0,0])
                prev_time = time.time()
                
            

    def publish_states(self, error):
        x_msg, y_msg, theta_msg, error_msg, ref_msg = (Float32(), Float32(), Float32(), Float32(), Float32())

        theta_msg.data = self.theta
        x_msg.data = self.LENGTH * np.sin(self.theta) 
        y_msg.data = -1 * self.LENGTH * np.cos(self.theta)

        error_msg.data = error
        ref_msg.data = self.desired_state[0,0]
        
        self.error_pub.publish(error_msg)
        self.ref_pub.publish(ref_msg)

        self.theta_pub.publish(theta_msg)
        self.x_pub.publish(x_msg)
        self.y_pub.publish(y_msg)



       
        
if __name__ == "__main__":
    pendulum = Pendulum()

    pendulum.linear_sim(method='rk')
