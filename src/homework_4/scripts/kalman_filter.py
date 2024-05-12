import numpy as np
from numpy.linalg import inv 
import pandas as pd
import matplotlib.pyplot as plt
    
if __name__ == "__main__":
    np.random.seed(42)
    # Simulation Details
    sim_step = 0.001 
    sim_len = 7
    # Robot measurements
    r = 5
    h = 10
    d = 19.1
    
    x = np.array([[0, 0, 0]]).T  # Initial States
   
    u = np.array([[0.1, 0.1]]).T # Inputs

    phi = np.array([[r/d, -r/d]])
    
    t = np.arange(0, sim_len, sim_step)
    A = np.zeros([3,3])
    P = np.eye(3,3)

    # Kalman parameters
    C = np.eye(3,3)
    K =  np.array([  [0.005,0,0], [0,0.005,0], [0,0,0.005] ])
    x_hat = np.array([[0, 0, 0]]).T  # Initial States
    p_hat = np.zeros([3,3])
    out = np.concatenate((x.T,x_hat.T),axis = 1)

    for i in t[:-1]:
        noise = np.random.normal(0, 10, 1) 
        D = np.array([[(r/2)*np.cos(x[2][0])-((h*r)/d)*np.sin(x[2][0]), (r/2)*np.cos(x[2][0])+((h*r)/d)*np.sin(x[2][0])],
                    [(r/2)*np.sin(x[2][0])+((h*r)/d)*np.cos(x[2][0]), (r/2)*np.sin(x[2][0])-((h*r)/d)*np.cos(x[2][0])]])
                
        D_inv = -(h*r**2)/d * np.array([[D[1][1], -D[0][1]],
                                        [-D[1][0], D[0][0]]])
        
        q = np.array([x[0], x[1]])
        M = np.array([D[0],
                    D[1],
                    phi[0]])
        
        
        x_hat_dot = M@u - P@C.T@inv(K)@(x_hat - x)
        p_hat_dot = A@P + P@A.T + P - P@C.T@inv(K)@C@P

        x_hat = x_hat + x_hat_dot*sim_step
        p_hat = p_hat + p_hat_dot*sim_step

        states = np.concatenate((x.T, x_hat.T), axis=1)
        out = np.concatenate((out,states), axis=0)
        
        x = x + sim_step*(M@u + noise) 
   
    t = t.reshape(-1, 1)
    out = np.concatenate((out, t), axis= 1)
    out_df = pd.DataFrame(out, columns=['x', 'y', 'theta', 'x_hat', 'y_hat', 'theta_hat', 'time'])
    out_df.plot(x='time',y=['x',"x_hat"])
    plt.show()  