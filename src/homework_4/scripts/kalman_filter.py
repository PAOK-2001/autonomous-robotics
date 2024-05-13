import numpy as np
import pandas as pd
from numpy.linalg import inv 
import matplotlib.pyplot as plt


    
if __name__ == "__main__":
    np.random.seed(42)
    # Simulation Details
    sim_step = 0.001 
    sim_len = 20
    # Robot measurements
    r = 5
    h = 10
    d = 19.1
    
    x = np.array([[0, 0, 0]]).T  # Initial States
    u = np.array([[0.1, 0.1]]).T # Inputs
    qd = np.array([1.0, 1.0]).T
    phi = np.array([[r/d, -r/d]])
    
    t = np.arange(0, sim_len, sim_step)
    A = np.zeros([3,3])

    # Kalman parameters
    C = np.eye(3,3)
    Q = np.eye(3,3)
    K =  np.array([  [0.005,0,0], [0,0.005,0], [0,0,0.005] ])
    P = np.zeros([3,3])

    x_hat = np.array([[0, 0, 0]]).T  # Initial States
    out = np.concatenate((x.T,x_hat.T),axis = 1)

    k_mat = np.mat([[0.01, 0],
                    [0, 0.01]])
    
    for i in t[:-1]:
        noise = np.random.normal(0, 2, 1) 
        #noise = 0
        D = np.array([[(r/2)*np.cos(x[2][0])-((h*r)/d)*np.sin(x[2][0]), (r/2)*np.cos(x[2][0])+((h*r)/d)*np.sin(x[2][0])],
                    [(r/2)*np.sin(x[2][0])+((h*r)/d)*np.cos(x[2][0]), (r/2)*np.sin(x[2][0])-((h*r)/d)*np.cos(x[2][0])]])
      
        D_inv = -(h*r**2)/d * np.array([[D[1][1], -D[0][1]],
                                        [-D[1][0], D[0][0]]])
        
        M = np.array([D[0],
                    D[1],
                    phi[0]])
        
        x_hat_dot = M@u - P@inv(K)@(x_hat - x)
        p_hat_dot = Q - P@inv(K)@P

        x_hat = x_hat + x_hat_dot*sim_step
        P = P + p_hat_dot*sim_step
        
        x_hat = np.array(x_hat)
        q = np.array([x_hat[0], x_hat[1]])
        error = qd - q.T
        u = D_inv@(k_mat@error.T)
        
        x = np.array(x + sim_step*(M@u + noise)) 
        states = np.concatenate((x.T, x_hat.T), axis=1)
        out = np.concatenate((out,states), axis=0)
        
        
    t = t.reshape(-1, 1)
    out = np.concatenate((out, t), axis= 1)
    out_df = pd.DataFrame(out, columns=['Position X', 'Position Y', 'Theta', 'Kalman Filtered X', 'Kalman Filtered Y', 'Kalman Filtered Theta', 'Time'])
    out_df.plot(x='Time',y=['Position X',"Kalman Filtered X"])
    plt.xlabel('Time (seconds)', fontsize=18)
    plt.ylabel('X position (meters)', fontsize=16)
    plt.show()  