import numpy as np
from unittest import TestCase

class KF:
    def __init__(self, init_a, init_v, init_h, j_var):
        self.x = np.array([init_a, init_v, init_h])
        self.P = np.eye(3)
        self.j_var = j_var


    def predict(self, dt: float) -> None:
        F = np.array([[1, 0, 0], [dt, 1, 0], [0.5 * dt**2, dt, 1]])
        G = np.array([dt, 0.5 * dt**2, 1/6 * dt**3]).reshape(3, 1)

        new_x = F.dot(self.x)
        new_P = F.dot(self.P).dot(F.T) + G.dot(G.T) * self.j_var

        self.x = new_x
        self.P = new_P


    def update(self, meas_val, meas_var):
        z = np.array([meas_val])
        R = np.array([meas_var])
        H = np.array([1, 0, 0]).reshape((1, 3))

        y = z - H.dot(self.x)
        S = H.dot(self.P).dot(H.T) + R
        K = self.P.dot(H.T).dot(np.linalg.inv(S))

        new_x = self.x + K.dot(y)
        new_P = (np.eye(3) - K.dot(H)).dot(self.P)
        self.x = new_x
        self.P = new_P

    
    @property
    def h(self):
        return self.x[2]
    
    @property
    def v(self):
        return self.x[1]
    
    @property
    def a(self):
        return self.x[0]