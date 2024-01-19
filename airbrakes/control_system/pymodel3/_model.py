import numpy as np


class Model:
    def __init__(self, dt, A, g, rho, Cd, thr, m, thr_time, var_a, var_h):
        self.dt = dt
        self.A = A
        self.g = g
        self.rho = rho
        self.Cd = Cd
        self.thr = thr
        self.m = m
        self.thr_time = thr_time

        self.var_a = var_a
        self.var_h = var_h

        self.v_real = 0
        self.s_real = 0
        self.t = 0
        self.a_real = self.__accel_curve()


    def update(self, area_adj):
        self.A = area_adj
        self.t += self.dt
        self.a_real = self.__accel_curve()
        self.v_real += self.a_real * self.dt
        self.s_real += self.v_real * self.dt


    # IMU SENSOR
    @property
    def a(self):
        return self.a_real + np.random.randn()*np.sqrt(self.var_a)
    
    # BAROMETER SENSOR
    @property
    def s(self):
        return self.s_real + np.random.randn()*np.sqrt(self.var_h)


    def __accel_curve(self):
        a = - self.g - (self.rho * self.Cd * self.A * self.v_real**2)/(2 * self.m)
        return a + self.thr/self.m if self.t < self.thr_time else a