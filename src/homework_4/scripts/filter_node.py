import rospy
import numpy as np
from numpy.linalg import inv 
import rospy.rosconsole
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose, Twist
# Filter Node
class FilterNode:
    def __init__(self):
        # ROS params
        rospy.init_node('filter_node')
        self.rate = rospy.Rate(100)

        # Subscriber and publisher
        self.sub_robot_state     = rospy.Subscriber('pose', Pose, self.robot_state_callback)
        self.control_state       = rospy.Subscriber('cmd_vel', Twist, self.control_input_callback)
        self.pub_filtered_state  = rospy.Publisher('filtered_pose', Pose, queue_size=10)
        
        self.sim_step = 0.001
        self.K =  np.array([  [0.005,0,0], [0,0.005,0], [0,0,0.005] ])
        self.Q = np.eye(3,3)
        self.P = np.zeros([3,3])
        self.pose = None
        self.u = None
        self.r = 5
        self.h = 10
        self.d = 19.1
        self.phi = np.array([[self.r/self.d, - self.r/self.d]])

    def robot_state_callback(self, msg):
        x = msg.position.x
        y = msg.position.y
        theta = msg.orientation.z
        self.pose = np.array([[x, y, theta]]).T

    def control_input_callback(self, msg):
        u_l = msg.linear.x
        u_r = msg.linear.y
        self.u = np.array([[u_l, u_r]]).T

    def apply_kalman_filter(self):
            state_msg = Pose()
            x_hat = np.array([[0, 0, 0]]).T  # Initial States
            state_msg.position.x = x_hat[0][0]
            state_msg.position.y = x_hat[1][0]
            state_msg.orientation.z = x_hat[2][0]
            self.pub_filtered_state.publish(state_msg)
            
            while not rospy.is_shutdown():
                if self.pose is not None  and self.u is not None:
                    D = np.array([[(self.r/2)*np.cos(self.pose[2][0])-((self.h*self.r)/self.d)*np.sin(self.pose[2][0]), (self.r/2)*np.cos(self.pose[2][0])+((self.h*self.r)/self.d)*np.sin(self.pose[2][0])],
                            [(self.r/2)*np.sin(self.pose[2][0])+((self.h*self.r)/self.d)*np.cos(self.pose[2][0]), (self.r/2)*np.sin(self.pose[2][0])-((self.h*self.r)/self.d)*np.cos(self.pose[2][0])]])
            
                    M = np.array([D[0],
                                  D[1],
                                  self.phi[0]])
                    x_hat_dot = M@self.u - self.P@inv(self.K)@(x_hat - self.pose)
                    p_hat_dot = self.Q - self.P@inv(self.K)@self.P

                    x_hat = x_hat + x_hat_dot*self.sim_step
                    self.P = self.P + p_hat_dot*self.sim_step
            
                    # Implement Kalman filter for state estimation
                    state_msg = Pose()

                    state_msg.position.x = x_hat[0][0]
                    state_msg.position.y = x_hat[1][0]
                    state_msg.orientation.w = x_hat[2][0]
                    
                self.pub_filtered_state.publish(state_msg)
                


if __name__ == "__main__":
    filter = FilterNode()
    filter.apply_kalman_filter()