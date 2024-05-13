import rospy
import numpy as np
from numpy.linalg import inv
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class RoverSim:
    def __init__(self):
        np.random.seed(42)

        rospy.init_node('system_node')
        self.pub_state = rospy.Publisher('pose', Pose, queue_size=10)
        self.sub_control = rospy.Subscriber('cmd_vel', Twist , self.control_callback)
        self.rate = rospy.Rate(100)  
        # Simulation Details
        self.sim_step = 0.001
        self.r = 5
        self.h = 10
        self.d = 19.1

        self.u = None
        self.x_hat =  None 

        self.x = np.array([[0, 0, 0]]).T  # Initial States

    def control_callback(self, msg):
        u_l = msg.linear.x
        u_r = msg.linear.y

        self.u = np.array([[u_l, u_r]]).T

    def kf_callback(self, msg):
        self.x_hat = msg

    def simulate(self):
        state_msg = Pose()
        x = np.array([[0, 0, 0]]).T  # Initial States
        phi = np.array([[self.r/self.d, -self.r/self.d]])
        state_msg.position.x = x[0][0]
        state_msg.position.y = x[1][0]
        state_msg.orientation.z = x[2][0]
        self.pub_state.publish(state_msg)

        while not rospy.is_shutdown():
            if self.u is not None:
                state_msg = Pose()
                noise = np.random.normal(0, 2, 1) 
                D = np.array([[(self.r/2)*np.cos(x[2][0])-((self.h*self.r)/self.d)*np.sin(x[2][0]), (self.r/2)*np.cos(x[2][0])+((self.h*self.r)/self.d)*np.sin(x[2][0])],
                            [(self.r/2)*np.sin(x[2][0])+((self.h*self.r)/self.d)*np.cos(x[2][0]), (self.r/2)*np.sin(x[2][0])-((self.h*self.r)/self.d)*np.cos(x[2][0])]])
            
                M = np.array([D[0],
                            D[1],
                            phi[0]])
                
                x = np.array(x + self.sim_step*(M@self.u + noise)) 

                state_msg.position.x = x[0][0]
                state_msg.position.y = x[1][0]
                state_msg.orientation.w = x[2][0]

            self.pub_state.publish(state_msg)
            self.rate.sleep()


if __name__ == "__main__":
    sim = RoverSim()
    sim.simulate()