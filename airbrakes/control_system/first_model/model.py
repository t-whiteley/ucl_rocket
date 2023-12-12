import numpy as np
import matplotlib.pyplot as plt
from _apogee import predict_apogee

# Initialize empty lists intead of empty arrays
dt = 0.1
duration = 30
t_axis = np.arange(0, duration, dt)
a = [0]
v = [0]
s = [0]
a_adj = [0]
v_adj = [0]
s_adj = [0]

ap_pred = [0]
ap_des = 2000


thr = 1000
thr_time = 2
m = 5
g = 9.81
rho = 1
area = 0.25
area_adj = 0.25
Cd = 0.01





def accel_curve(t, v, A):
    a = - g - (rho * Cd * A * v**2)/(2 * m)
    return a + thr/m if t < thr_time else a


# main loop
for t in t_axis:

    a.append(accel_curve(t, v[-1], area))
    v.append(v[-1] + a[-1] * dt)
    s.append(s[-1] + v[-1] * dt)

    a_adj.append(accel_curve(t, v_adj[-1], area_adj))
    v_adj.append(v_adj[-1] + a_adj[-1] * dt)
    s_adj.append(s_adj[-1] + v_adj[-1] * dt)

    if t > thr_time * 1.2 and v_adj[-1] > 0:
        ap_pred.append(predict_apogee(v_adj[-1], s_adj[-1], a_adj[-1], rho, m, area_adj, g))
    else:
        ap_pred.append(None)

    # proportional controller feedback loop
    kp = 0.0001
    prev_ap_pred = ap_pred[-1]
    if not prev_ap_pred:
        pass
    elif prev_ap_pred > ap_des:
        area_adj += (prev_ap_pred - ap_des) * kp
        # here is the signal to send to servo



# info
ap_act = max(s)


# the plot
# plt.plot(t_axis, a[1:], label="a")
# plt.plot(t_axis, v[1:], label="v")
plt.plot(t_axis, s[1:], label="height (no control)")
plt.plot(t_axis, s_adj[1:], label="height (proportional control)")
plt.plot(t_axis, ap_pred[1:], color='red', label="apogee_predicted")

plt.axhline(y=ap_act, linestyle='dotted', color='red', label='apogee_actual')
plt.axhline(y=ap_des, linestyle='dotted', color='green', label='apogee_desired')
plt.axhline(y=0, color='black')
plt.legend()
plt.xlabel("Time (s)")
plt.ylabel("Height (m)")
plt.show()