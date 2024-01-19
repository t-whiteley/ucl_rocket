# here all the noise filterring and sensor fusion are done

from _model import Model
from _kf import KF


class FlightComp():
    def __init__(self, dt, A, g, rho, Cd, thr, m, thr_time):
        self.meas_every = 1
        self.step = 0
        self.var_process = 20**2
        self.var_a = 10**2
        self.var_h = 10**2

        self.vest = 0
        self.M = Model(dt, A, g, rho, Cd, thr, m, thr_time, self.var_a, self.var_h)
        self.KF = False
    
    def update(self, area_adj):
        self.M.update(area_adj)

        if self.M.t > self.M.thr_time * 1.3 and not self.KF:
            self.KF = KF(self.M.a, self.vest, self.M.s, self.var_process, self.var_a, self.var_h)
        elif self.M.t > self.M.thr_time * 1.3:
            self.KF.predict(self.M.dt)
            self.step += 1
            if not self.step % self.meas_every:
                self.KF.update(self.M.a, self.M.s)
        else:
            self.vest += self.M.a * self.M.dt

    
    @property
    def a(self):
        return self.KF.a if self.KF else self.M.a
    
    @property
    def v(self):
        return self.KF.v if self.KF else self.vest
    
    @property
    def h(self):
        return self.KF.h if self.KF else self.M.s