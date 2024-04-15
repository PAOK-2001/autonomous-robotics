import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float32


class Rover:
    def __init__(self):
        rospy.init_node("rover_sim")

        self.x_pub = rospy.Publisher("/rover_x", Float32)
        self.y_pub = rospy.Publisher("/rover_y", Float32)
        self.theta_pub = rospy.Publisher("/rover_theta", Float32)

        self.rate = rospy.Rate(100)

        #Turtle bot waffle pi
        self.WHEEL_RADIUS = 0.066 # meters
        self.WHEEL_BASE   = 0.16

        self.x = 0
        self.y = 0
        self.theta = 0
        self.velocity = 0
 

    def nonlinear_sim(self):
        prev_time = time.time()
        dot_X2 = 0
        dot_X1 = 0
        dot_X3 = 0
        
        while (True):
            left_velocity = 0.5
            right_velocity = 0.5
            
            self.velocity = (right_velocity + left_velocity)/2
            
            dt = time.time() - prev_time

            dot_X1 = self.velocity*np.cos(self.theta)
            dot_X2 = self.velocity*np.sin(self.theta)
            dot_X3 = (right_velocity - left_velocity) / self.WHEEL_BASE

            self.x += dot_X1 *dt
            self.y += dot_X2 *dt
            self.theta += dot_X3 *dt

            if(self.theta < 0): self.theta = self.theta + 2*np.pi

            x_msg = Float32()
            y_msg = Float32()
            theta_msg = Float32()

            x_msg.data = self.x
            y_msg.data = self.y
            theta_msg.data = self.theta

            self.x_pub.publish(x_msg)
            self.y_pub.publish(y_msg)
            self.theta_pub.publish(theta_msg)

            prev_time = time.time()
            self.rate.sleep()
           
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
            right_velocity = 0.3
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

            x_msg = Float32()
            y_msg = Float32()
            theta_msg = Float32()

            x_msg.data = self.x
            y_msg.data = self.y
            theta_msg.data = self.theta

            self.x_pub.publish(x_msg)
            self.y_pub.publish(y_msg)
            self.theta_pub.publish(theta_msg)

            prev_time = time.time()
            self.rate.sleep()
        
if __name__ == "__main__":
    lineal = True
    rover = Rover()
    if (lineal):
        print("Linear simulation started")
        rover.linear_sim()
    else:
        print("Non linear simulation started")
        rover.nonlinear_sim()        
        