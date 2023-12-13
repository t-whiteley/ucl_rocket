import numpy as np
import matplotlib.pyplot as plt
from _apogee import predict_apogee

# variable with CHANGE can change on different launch settings


# CONSTANTS TO SETUP ***CHANGE***
dt = 0.1 # time step in seconds
duration = 30 # duration of the flight CHANGE
ap_des = 4000 # desired apogee in m CHANGE
thr = 2000 # thrust in N
thr_time = 2 # thrust curve modelled as step, how long is step
m = 5 # mass of rocket
g = 9.81
rho = 1 # atmospheric density
area = 0.25 # default area of rocket m^2
Cd = 0.01 # default coefficient of drag


# initialise axis to collect and plot data
t_axis = np.arange(0, duration, dt)
a = [0]
v = [0]
s = [0]
a_adj = [0]
v_adj = [0]
s_adj = [0]
ap_pred = [0]
servo_sig = [0]
area_adj = area



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


    # only during thrust and before apogee
    if t > thr_time * 1.2 and v_adj[-1] > 0:
        ap_pred.append(predict_apogee(v_adj[-1], s_adj[-1], a_adj[-1], rho, m, area_adj, g))
    else:
        ap_pred.append(None)



    # PID controller
    kp = 0.0001
    ki = 0.001
    kd = 0.00001
    prev_ap_pred = ap_pred[-1]
    if not prev_ap_pred:
        servo_sig.append(0)
        pass
    elif prev_ap_pred > ap_des:
        curr_sig = (prev_ap_pred - ap_des)
        
        sig = kp * curr_sig
        if servo_sig[-1]:
            sig += kd * (curr_sig - servo_sig[-1])/dt
            sig += ki * sum(servo_sig)
        

        area_adj += sig
        servo_sig.append(sig)
        # here is the signal to send to servo











fig, axes = plt.subplots(nrows=2, ncols=1, figsize=(8, 6))

# Plot in the first subplot
axes[0].plot(t_axis, s[1:], label="height (no control)")
axes[0].plot(t_axis, s_adj[1:], label="height (PID control)")
axes[0].plot(t_axis, ap_pred[1:], color='red', label="apogee_predicted")

axes[0].axhline(y=max(s), linestyle='dotted', color='red', label='apogee_actual')
axes[0].axhline(y=ap_des, linestyle='dotted', color='green', label='apogee_desired')
axes[0].axhline(y=0, color='black')
axes[0].legend()
axes[0].set_xlabel("Time (s)")
axes[0].set_ylabel("Height (m)")

# Plot in the second subplot
axes[1].plot(t_axis, servo_sig[1:], color='red', label="servo signal")
axes[1].plot(t_axis, [0] * len(t_axis), color='black')
axes[1].legend()
axes[1].set_xlabel("Time (s)")
axes[1].set_ylabel("Servo Signal (deg)")

# Adjust layout to prevent overlapping
plt.tight_layout()

# Show the plots
plt.show()
