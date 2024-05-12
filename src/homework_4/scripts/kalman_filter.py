import numpy as np
from numpy.linalg import inv 
import pandas as pd
import matplotlib.pyplot as plt


# class KalmanFilter():
#     def __init__(self):
#         pass

#     @staticmethod
#     def predict(system_coefficients, system_input, 
#                 prev_states, prev_covariance, process_noise, 
#                 sim_step):
        
#         A, B = system_coefficients
#         estimated_states = A@prev_states + B@system_input
#         estimated_covariance = A@prev_covariance@A.T + process_noise

#         return (estimated_states, estimated_covariance)
        
#     @staticmethod
#     def update(measurements, states, covariance, 
#                obser_mat, obser_noise, kalman_gain,
#                sim_step):
#         breakpoint()
#         error = measurements - obser_mat@states
#         error_covariance = obser_mat@(covariance@obser_mat.T) + obser_noise
#         # breakpoint()
#         updated_gain = (covariance@obser_mat.T)@inv(error_covariance) # must invert
#         updated_state = states + updated_gain@error
        
#         I = np.eye(len(updated_gain))
#         updated_covariance = (I-updated_gain@obser_mat)@covariance@(I-updated_gain*obser_mat).T + updated_gain@obser_noise@updated_gain.T
        
#         return (updated_state *sim_step, updated_covariance*sim_step, updated_gain)
    
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
        
        
        # x_hat_dot, p_hat_dot = KalmanFilter.predict(system_coefficients=(A, M),
        #                                 system_input= u,
        #                                 prev_states= x,
        #                                 prev_covariance=P,
        #                                 process_noise= noise, 
        #                                 sim_step = sim_step)
        
        # Simulate model step
        # x_hat = x_hat + x_hat_dot
        # p_hat = p_hat + p_hat_dot
        
        x = x + sim_step*(M@u + noise) 
        # x_hat_dot = M@u - P@C.T@inv(K)@(x_hat - x)
        # p_hat_dot = A@P + P@A.T + P - P@C.T@inv(K)@C@P
        # x_hat_dot, p_hat_dot, K = KalmanFilter.update(measurements = x,
        #                                     states= x_hat,
        #                                     covariance= p_hat,
        #                                     obser_mat= C, 
        #                                     obser_noise=P, 
        #                                     kalman_gain=K, 
        #                                     sim_step= sim_step)

        x_hat = x_hat + x_hat_dot*sim_step
        p_hat = p_hat + p_hat_dot*sim_step
        
        # breakpoint()

        states = np.concatenate((x.T, x_hat.T), axis=1)
        out = np.concatenate((out,states), axis=0)
   
    t = t.reshape(-1, 1)
    out = np.concatenate((out, t), axis= 1)
    out_df = pd.DataFrame(out, columns=['x', 'y', 'theta', 'x_hat', 'y_hat', 'theta_hat', 'time'])
    out_df.plot(x='time',y=['x',"x_hat"])
    plt.show()  