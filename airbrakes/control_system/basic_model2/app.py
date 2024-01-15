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
servo_sig = [0]
area_adj = area

# KF
started = 0
MEAS_EVERY_STEPS = 1
VAR_PROCESS = 200**2 #5
VAR_A = 100**2 #1
VAR_H = 150**2 #2

# PID Control
kp = 0.001
ki = 0.01
kd = 0.0001
max_A = 2
min_A = area






### MAIN LOOP

for t in t_axis:
    curr_a = accel_curve(t, v_real[-1], area_adj, g, rho, Cd, thr, m, thr_time)

    a_real.append(curr_a)
    v_real.append(v_real[-1] + a_real[-1] * dt)
    s_real.append(s_real[-1] + v_real[-1] * dt)

    a_rec.append(a_real[-1] + np.random.randn()*np.sqrt(VAR_A))
    v_rec.append(a_real[-1] * dt + v_rec[-1])
    s_rec.append(s_real[-1] + np.random.randn()*np.sqrt(VAR_H))

    a_nocon_curr = curr_a = accel_curve(t, v_real[-1], area, g, rho, Cd, thr, m, thr_time)
    a_nocon.append(a_nocon_curr)
    v_nocon.append(v_nocon[-1] + a_nocon[-1] * dt)
    s_nocon.append(s_nocon[-1] + v_nocon[-1] * dt)


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
    if not prev_ap_pred:
        servo_sig.append(0)
        pass
    elif prev_ap_pred > ap_des:
        curr_sig = (prev_ap_pred - ap_des)
        
        sig = kp * curr_sig
        if servo_sig[-1]:
            sig += kd * (curr_sig - servo_sig[-1])/dt
            # can add moving sum to find integral faster each time
            sig += ki * sum(servo_sig) * dt
        
        area_adj += sig


        area_adj = min(max_A, area_adj)
        area_adj = max(min_A, area_adj)

        servo_sig.append(sig)
        # here is the signal to send to servo
    else:
        servo_sig.append(0)
    








### PLOTTING
        
# Plot for Acceleration
plt.figure(figsize=(6, 6))

plt.subplot(3, 1, 1)
plt.plot(t_axis, a_real[1:], label="a_real")
plt.plot(t_axis, a_rec[1:], label="a_rec")
plt.plot(t_axis, a_fil[1:], label="a_fil")

plt.axhline(y=0, color='black')
plt.legend()
plt.xlabel("Time (s)")
plt.ylabel("Acceleration")
plt.title("Acceleration Curves")

# Plot for Height
plt.subplot(3, 1, 2)
plt.plot(t_axis, s_nocon[1:], label="s_nocon", color='blue')
plt.plot(t_axis, s_real[1:], label="s_real", color='black')
plt.plot(t_axis, s_rec[1:], label="s_rec", color='red')
plt.plot(t_axis, s_fil[1:], label="s_fil", color='green')
plt.axhline(y=max(s_nocon), linestyle='dotted', color='blue', label=f'apogee_nocon {round(max(s_nocon))}')
plt.axhline(y=max(s_real), linestyle='dotted', color='black', label=f'apogee_real {round(max(s_real))}')
plt.axhline(y=max(s_rec), linestyle='dotted', color='red', label=f'apogee_rec {round(max(s_rec))}')
plt.axhline(y=max(s_fil), linestyle='dotted', color='green', label=f'apogee_fil {round(max(s_fil))}')
plt.axhline(y=0, color='black')
plt.legend()
plt.xlabel("Time (s)")
plt.ylabel("Height")
plt.title("Height Curves")

# Plot for Servo
plt.subplot(3, 1, 3)
plt.plot(t_axis, servo_sig[1:], label="servo_signal")
plt.axhline(y=0, color='black')


plt.legend()
plt.xlabel("Time (s)")
plt.ylabel("Volts")
plt.title("Servo Signal")

plt.subplots_adjust(hspace=0.1)  # Adjust the hspace value
plt.tight_layout()
plt.show()