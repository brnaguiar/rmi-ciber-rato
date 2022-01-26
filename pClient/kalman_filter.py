import numpy as np
import math 


class KalmanFilter:

    def __init__(self, Q, R, w, v):
        self.Q = Q
        self.R = R
        self.w = w
        self.v = v
        self.dt = 1.0  # time tick [s]

    #motion_model 
    def g(self, x, u): 
        A = np.array([[1.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0],
                      [0.0, 0.0, 1.0]])   

        B = np.array([[math.cos(x[2]) * self.dt, 0.0],
                        [math.sin(x[2]) * self.dt, 0.0],
                        [0.0                , self.dt ]])

        return A @ x + B @ u + self.v     


    #observation_model 
    def h(self, x): 

        H = np.array([[1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0]]) 

        return H @ x + self.w 


    def compute_jacob(self, model=None,x=None,u=None):
        return [[1.0, 0.0, 0.0], #math.sin(yaw) * v * -dt, math.cos(yaw) * dt
                [0.0, 1.0, 0.0], #math.cos(yaw) * v *  dt, math.sin(yaw) * dt
                [0.0, 0.0, 1.0]]  #0.0, 0.0, dt]  ,  


    def ekf_estimation(self, x, cov_estimation_mtx, z, u):

        #####  Prediction #####
        # pred_μ_t = g(u_t, μ_t-1)
        state_predict = self.g(x, u)  

        print(f'State Estimate Befire EKF: {state_predict}')   
        # pred_Σ_t = G_t * Σ_(t-1) * G_t^T + Q 
        jg = self.compute_jacob("G", x, u)
        cov_predict_mtx = jg @ cov_estimation_mtx @ np.transpose(jg) + self.Q  

        #####  Correction ##### 
        jh = self.compute_jacob("H")
        # K_t = pred_Σ_t * H_t^T * (H_t * pred_Σ_t * H_t^T + R)^-1 --- Kalman gain: calcular o peso da observacao com base no ruido da mesma. se o ruido for zero: as matrizes vao dar a matriz indentidade.
        K_gain = cov_predict_mtx @ np.transpose(jh) @ np.linalg.inv(jh @ cov_predict_mtx @ np.transpose(jh) + self.R)
        # μ_t = predicted_μ_t + K_t * (z_t - H_t * h(predicted_μ_t)) --- z_t: o que estou a observar, h(predicted_mean_t): a minha previsao da observacao): erro
        state_estimation = state_predict + K_gain @ (z - self.h(state_predict))

        print(f'Observation={z}')
        # Σ_t = (I - K_t * H_t) * pred_Σ_t 
        cov_estimation_mtx = (np.eye(len(state_estimation)) - K_gain @ jh) @ cov_predict_mtx  

        print(f'State Estimate After EKF={state_estimation}') 

        return state_estimation, cov_estimation_mtx  


def main():

    t = 1

    #dt = 1     

    state_estimation =  np.array([0.0,0.0,0.0]) # = 

    control_v = np.array([4.5,0.0]) # . 
 
    cov_estimation_mtx = np.array([[0.1,  0,   0],
                                   [  0,0.1,   0],
                                   [  0,  0, 0.1]])

    
    kalman = KalmanFilter(Q, R,  w, v)

    for t, observation_vector in enumerate(z, start=1):
        print(f'Timestep: {t}')
        optimal_state_estimate, covariance_estimate = kalman.ekf_estimation(state_estimation, 
                                                                     cov_estimation_mtx, 
                                                                     observation_vector, 
                                                                     control_v)


        state_estimation = optimal_state_estimate
        cov_estimation_mtx = covariance_estimate 

        print()


main()


