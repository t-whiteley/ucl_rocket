# NOT SURE WHY UPDATE IS NOW WORKING WITH 2*2s instead 3*3 but it works

import numpy as np

class KF:
    def __init__(self, init_a, init_v, init_h, p_var, a_var, h_var):
        self.x = np.array([init_h, init_v, init_a])
        self.P = np.eye(3)
        self.p_var = p_var
        self.a_var = a_var
        self.h_var = h_var


    def predict(self, dt: float):
        F = np.array([[1, dt, dt*dt/2], [0, 1, dt], [0, 0, 1]])
        G = np.array([dt*dt*dt/6, dt*dt/2, dt]).reshape(3, 1)

        new_x = F.dot(self.x)
        new_P = F.dot(self.P).dot(F.T) + G.dot(G.T) * self.p_var

        self.x = new_x
        self.P = new_P

    def update(self, accel_meas, height_meas):
        z = np.array([height_meas, accel_meas]).reshape(2, 1)
        R = np.diag([self.h_var, self.a_var])

        # z = np.array([height_meas, accel_meas]).reshape(2, 1)
        # R = np.zeros((3, 3))
        # R[0, 0] = self.h_var
        # R[2, 2] = self.a_var

        H = np.array([[1, 0, 0], [0, 0, 1]]) 

        # y = z - H.dot(self.x)
        y = z - H.dot(self.x.reshape(-1, 1))
        
        # R and expr below are incompatible sizes: (2,2) (3,3)
        # H.dot(self.P).dot(H.T) -> should be 3,3
        S = H.dot(self.P).dot(H.T) + R
        K = self.P.dot(H.T).dot(np.linalg.inv(S))

        # new_x = self.x + K.dot(y)
        new_x = (self.x.reshape(-1, 1) + K.dot(y)).flatten()
        new_P = (np.eye(3) - K.dot(H)).dot(self.P)
        self.x = new_x
        self.P = new_P

    @property
    def h(self):
        return self.x[0]

    @property
    def v(self):
        return self.x[1]

    @property
    def a(self):
        return self.x[2]