# positive is directed upwards


# might be able to include density in Cd function
# better to induce or plug in?
def drag_coeff(a, v, g, m, rho, area):
    return - (a + g) * 2*m / (rho * area * v**2)
    # return 0.1

# find the value of k, the derivative of v -> a, at different points
def k_vel(v, g, Cd, rho, m, area):
    return (-g - Cd*rho/(2*m) * area * v**2)

# numerical ODE sol from: dv/dt = -g - rhoCd/2m * (Av^2)
# 4th order runge kutte method
def KF_vel(v_n, Cd, rho, m, area, g, h):
    k1 = k_vel(v_n, g, Cd, rho, m, area)
    k2 = k_vel(v_n + h*k1/2, g, Cd, rho, m, area)
    k3 = k_vel(v_n + h*k2/2, g, Cd, rho, m, area)
    k4 = k_vel(v_n + h*k3, g, Cd, rho, m, area)

    v_1 = v_n + h/6 * (k1 + 2*k2 + 2*k3 + k4)
    return v_1


def predict_apogee(v_curr, s_curr, a_curr, rho, m, area, g):
    h = 0.1
    
    Cd = drag_coeff(a_curr, v_curr, g, m, rho, area)

    v_hist = []
    v = v_curr
    while (v := KF_vel(v, Cd, rho, m, area, g, h)) > 0:
        v_hist.append(v)
        # print(v)
    
    apo_pred = s_curr + sum(v_hist)*h
    # print("predicted apogee: " + str(apo_pred))
    # print(Cd, apo_pred)
    return apo_pred