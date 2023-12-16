#include "rocket_sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>


#define FREQ 120
#define DT 0.01
#define GRAV_CONST 9.81
#define RHO 1.293
#define MASS 5
#define DESIRED_APOGEE 800

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))


// (TODO) improve by CFD or other means
float drag_coeff() {
    return 0.1;
}

// derivative of v for numeric solution using runge-kutta
float k_vel(float v, float cd, float A) {
    return (-GRAV_CONST - cd*RHO/(2*MASS) * A * v*v);
}

// numeric solution to find velocity at every dt until apogee
float RK_vel(float v, float cd, float A, float h_RK) {
    float k1 = k_vel(v, cd, A);
    float k2 = k_vel(v + h_RK*k1/2, cd, A);
    float k3 = k_vel(v + h_RK*k2/2, cd, A);
    float k4 = k_vel(v + h_RK*k3, cd, A);
    float v_next = v + h_RK/6 * (k1 + 2*k2 + 2*k3 + k4);
    return v_next;
}

// solve ODE numerically to find height at all points
float predict_apogee(float v, float h, float A) {
    float h_RK = 0.1;
    float cd = drag_coeff();
    
    float sum_vel = 0;
    while ((v = RK_vel(v, cd, A, h_RK)) > 0) {
        sum_vel += v;
    }
    float ap_pred = h + sum_vel * h_RK;
    return ap_pred;
}


float PID(float e, float* e_prior, float* i_prior, float kp, float ki, float kd, float b) {
    float i = *i_prior + e * DT;
    float d = (e - *e_prior) / DT;
    float output = kp*e + ki*i + kd*d + b;

    *e_prior = e;
    *i_prior = i;
    return output;
}






int main() {
    // PARAMS FOR MODEL - DEL FOR REAL ONE
    Data d0 = {0, 0, 0, DT};
    Data* d = &d0;
    // float seconds = 1.f / FREQ;
    // unsigned int micro = (unsigned int)(seconds * 1e6);

    // PARAMS FOR DATA VIS - DEL FOR REAL ONE
    FILE *csv = fopen("output.csv", "w");
    fprintf(csv, "Time,Acceleration,Velocity,Altitude,Ap_pred,A\n");


    // setup
    float t = 0;
    float A_min = 0.25;
    float A = 0.25;
    float cd = 0.005;


    // setup PID and ramp
    float error_prior = 0;
    float integral_prior = 0;
    float kp = 0.0001;
    float ki = 0.005;
    float kd = 0;
    float b = 0;
    float sig_des = 0;
    float motor_step = 0.005;

    while (1) {
        if (d->h < 0 && t > 2) {
            break;
        }
        // THIS IS ALL PARAMS FOR MODEL AND DATA VIS - DEL FOR REAL ONE
        // usleep(micro);
        // printf("t: %f, a: %f, v: %f, h: %f\n", t, d->a, d->v, d->h);
        cd = drag_coeff();
        d = generate_model_data(d, t, A, cd);
        t += DT;
        float a = d->a;
        float v = d->v;
        float h = d->h;


        // condition to starts control system
        float ap_pred = 0;
        if (t > 2 && v > 0) {
            ap_pred = predict_apogee(v, h, A);
            float error = ap_pred - DESIRED_APOGEE;
            sig_des = PID(error, &error_prior, &integral_prior, kp, ki, kd, b);

            // motor follows ramp, can move maximum of motor step every interval
            float sign = (sig_des - A) / fabs(sig_des - A);
            A += sign * motor_step;
            A = MAX(A, A_min);
        }

        fprintf(csv, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", t, a, v, h, ap_pred, A);
    }


    fclose(csv);
    return 0;
}