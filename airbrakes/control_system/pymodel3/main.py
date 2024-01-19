from _flightcomp import FlightComp
from _plotting import Plot
from apogee import predict_apogee


###CONSTANTS

# ROCKET PARAMETERS
dt = 0.1 # time step in seconds
duration = 30 # duration of the flight CHANGE
ap_des = 6000 # desired apogee in m CHANGE
thr = 4000 # thrust in N
thr_time = 2 # thrust curve modelled as step, how long is step
m = 5 # mass of rocket
g = 9.81
rho = 1 # atmospheric density
area = 0.25 # default area of rocket m^2
Cd = 0.01 # default coefficient of drag
area_adj = area

# PD Control
kp = 0.00001
ki = 0.00001
kd = 0.0000000001
max_A = 0.75
min_A = area
error_sum = 0





### MAIN LOOP

C = FlightComp(dt, area, g, rho, Cd, thr, m, thr_time)
P = Plot(ap_des)

while C.M.t < duration:
    C.update(area_adj)


    # ESTIMATE APOGEE HERE
    if C.M.t > thr_time * 1.2:
        ap_pred = predict_apogee(C.v, C.h, C.a, rho, m, area_adj, g)
    else:
        ap_pred = 0
    
    
    # CONTROL HERE
    if ap_pred > ap_des:
        error = ap_pred - ap_des
        curr_sig = error
        error_sum += error
        
        sig = kp * curr_sig
        sig += ki * error_sum * dt
        sig += kd * (curr_sig - error)/dt
        
        area_adj += sig
        area_adj = min(max_A, area_adj)
        area_adj = max(min_A, area_adj)

        servo_sig = sig
    else:
        error = 0
        servo_sig = 0


    P.append(C.M.t, C.M.a_real, C.M.v_real, C.M.s_real, C.a, C.v, C.h, ap_pred, error, servo_sig)

P.plot()