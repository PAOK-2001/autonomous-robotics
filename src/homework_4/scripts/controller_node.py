import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

# Controller Node
class ControllerNode:
    def __init__(self):
        rospy.init_node('controller_node')
        self.pub_input = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub_pose = rospy.Subscriber('filtered_pose', Pose, self.pose_callback)
        self.rate = rospy.Rate(100)
        self.qd = np.array([1.0, 1.0]).T
        self.r = 5
        self.h = 10
        self.d = 19.1
        self.k_mat = np.mat([[0.01, 0],
                    [0, 0.01]])

        self.pose = None

    def pose_callback(self, msg:Pose):
        x = msg.position.x
        y = msg.position.y
        theta = msg.orientation.z
        self.pose = np.array([[x, y, theta]]).T

    def compute_control(self):
        while not rospy.is_shutdown():
            if self.pose is not None:
                D = np.array([[(self.r/2)*np.cos(self.pose[2][0])-((self.h*self.r)/self.d)*np.sin(self.pose[2][0]), (self.r/2)*np.cos(self.pose[2][0])+((self.h*self.r)/self.d)*np.sin(self.pose[2][0])],
                            [(self.r/2)*np.sin(self.pose[2][0])+((self.h*self.r)/self.d)*np.cos(self.pose[2][0]), (self.r/2)*np.sin(self.pose[2][0])-((self.h*self.r)/self.d)*np.cos(self.pose[2][0])]])
            
                D_inv = -(self.h*self.r**2)/self.d * np.array([[D[1][1], -D[0][1]],
                                                [-D[1][0], D[0][0]]])

                q = np.array([self.pose[0], self.pose[1]])
                error = self.qd - q.T
                u = D_inv@(self.k_mat@error.T)
                
                control_msg = Twist()
                control_msg.linear.x = u[0,0]
                control_msg.linear.y = u[1,0]
                self.pub_input.publish(control_msg)


if __name__ == "__main__":
    controller = ControllerNode()
    controller.compute_control()