def accel_curve(t, v, A, g, rho, Cd, thr, m, thr_time):
    # print(v)
    a = - g - (rho * Cd * A * v**2)/(2 * m)
    return a + thr/m if t < thr_time else a