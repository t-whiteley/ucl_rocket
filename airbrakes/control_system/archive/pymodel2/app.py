import numpy as np
import matplotlib.pyplot as plt
from _apogee import predict_apogee
from _kf import KF
from _acurve import accel_curve


###CONSTANTS

# ROCKET PARAMETERS
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
a_real = [0]
v_real = [0]
s_real = [0]
a_rec = [0]
v_rec = [0]
s_rec = [0]
a_fil = [0]
v_fil = [0]
s_fil = [0]
a_nocon = [0]
v_nocon = [0]
s_nocon = [0]
ap_pred = [0]
error = [0]
servo_sig = [0]
area_adj = area

# KF
started = 0
MEAS_EVERY_STEPS = 1
VAR_PROCESS = 20**2 #5
VAR_A = 10**2 #1
VAR_H = 10**2 #2

# PD Control
kp = 0.00001
ki = 0.00001
kd = 0.0000000001
max_A = 0.75
min_A = area






### MAIN LOOP

for t in t_axis:
    curr_a = accel_curve(t, v_real[-1], area_adj, g, rho, Cd, thr, m, thr_time)

    a_nocon_curr = accel_curve(t, v_nocon[-1], area, g, rho, Cd, thr, m, thr_time)
    a_nocon.append(a_nocon_curr)
    v_nocon.append(v_nocon[-1] + a_nocon[-1] * dt)
    s_nocon.append(s_nocon[-1] + v_nocon[-1] * dt)
    

    a_real.append(curr_a)
    v_real.append(v_real[-1] + a_real[-1] * dt)
    s_real.append(s_real[-1] + v_real[-1] * dt)

    a_rec.append(a_real[-1] + np.random.randn()*np.sqrt(VAR_A))
    v_rec.append(a_rec[-1] * dt + v_rec[-1])
    s_rec.append(s_real[-1] + np.random.randn()*np.sqrt(VAR_H))


    # DO FILTERRING HERE
    if t > thr_time * 1.2:
        if not started:
            kf = KF(a_rec[-1], v_rec[-1], s_rec[-1], VAR_PROCESS, VAR_A, VAR_H)
            step = 0
            started = 1

        step += 1
        if not step % MEAS_EVERY_STEPS:
            kf.update(a_rec[-1], s_rec[-1])

        kf.predict(dt)
        a_fil.append(kf.a)
        v_fil.append(kf.v)
        s_fil.append(kf.h)
    else:
        a_fil.append(0)
        v_fil.append(0)
        s_fil.append(0)
    


    # ESTIMATE APOGEE HERE
    if t > thr_time * 1.2 and v_real[-1] > 0:

        ap_pred.append(predict_apogee(v_fil[-1], s_fil[-1], a_fil[-1], rho, m, area_adj, g))
    else:
        ap_pred.append(0)
    
    

    # DO CONTROL HERE
    prev_ap_pred = ap_pred[-1]

    if prev_ap_pred > ap_des:
        error.append(prev_ap_pred - ap_des)
        curr_sig = (error[-1])
        
        sig = kp * curr_sig
        sig += ki * sum(error) * dt
        sig += kd * (curr_sig - error[-1])/dt
        
        area_adj += sig
        area_adj = min(max_A, area_adj)
        area_adj = max(min_A, area_adj)

        servo_sig.append(sig)
    else:
        error.append(0)
        servo_sig.append(0)
    







# PLOTTING

# Create a 2x2 grid for plots
fig, axs = plt.subplots(2, 2, figsize=(10, 7))

# Plot for Acceleration
axs[0, 0].plot(t_axis, a_real[1:], label="a_real")
axs[0, 0].plot(t_axis, a_rec[1:], label="a_rec")
axs[0, 0].plot(t_axis, a_fil[1:], label="a_fil")
axs[0, 0].axhline(y=0, color='black')
axs[0, 0].legend()
axs[0, 0].set_xlabel("Time (s)")
axs[0, 0].set_ylabel("Acceleration")
axs[0, 0].set_title("Acceleration Curves")

# Plot for Height
axs[0, 1].plot(t_axis, s_nocon[1:], label="s_ncon", color='blue')
axs[0, 1].plot(t_axis, s_real[1:], label="s_real", color='black')
axs[0, 1].plot(t_axis, s_rec[1:], label="s_rec", color='red')
axs[0, 1].plot(t_axis, s_fil[1:], label="s_fil", color='green')
axs[0, 1].plot(t_axis, ap_pred[1:], label="ap_pred", color='purple')
axs[0, 1].axhline(y=max(s_nocon), linestyle='dotted', color='blue', label=f'ap_nocon {round(max(s_nocon))}')
axs[0, 1].axhline(y=max(s_real), linestyle='dotted', color='black', label=f'ap_real {round(max(s_real))}')
axs[0, 1].axhline(y=max(s_rec), linestyle='dotted', color='red', label=f'ap_rec {round(max(s_rec))}')
axs[0, 1].axhline(y=max(s_fil), linestyle='dotted', color='green', label=f'ap_fil {round(max(s_fil))}')
axs[0, 1].axhline(y=0, color='black')
axs[0, 1].legend()
axs[0, 1].set_xlabel("Time (s)")
axs[0, 1].set_ylabel("Height")
axs[0, 1].set_title("Height Curves")

# Plot for Servo
axs[1, 1].plot(t_axis, servo_sig[1:], label="servo_signal")
axs[1, 1].axhline(y=0, color='black')
axs[1, 1].legend()
axs[1, 1].set_xlabel("Time (s)")
axs[1, 1].set_ylabel("Servo Signal")
axs[1, 1].set_title("Servo Signal")

# Plot for Speed
axs[1, 0].plot(t_axis, v_fil[1:], label="v_fil")
axs[1, 0].plot(t_axis, v_real[1:], label="v_real")
axs[1, 0].axhline(y=0, color='black')
axs[1, 0].legend()
axs[1, 0].set_xlabel("Time (s)")
axs[1, 0].set_ylabel("Volts")
axs[1, 0].set_title("Speed Curves")

# Adjust layout
plt.subplots_adjust(hspace=0.4, wspace=0.3)

plt.tight_layout()
plt.show()