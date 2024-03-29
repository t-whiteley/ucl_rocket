#include "IMU_sim.h"
#include "kf.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define FREQ 120
#define DT 0.01
#define GRAV_CONST 9.81
#define RHO 1.293
#define MASS 5
#define DESIRED_APOGEE 750
#define MOTOR_STEP 0.01
#define KF_EVERY 3





// (TODO) improve by CFD with lookup table or other means
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
    srand(time(NULL));
    FILE *csv = fopen("output.csv", "w");
    fprintf(csv, "Time,Accel_noisy,Accel_kalman,Accel_real\n");
    // float seconds = 1.f / FREQ;
    // unsigned int micro = (unsigned int)(seconds * 1e6);


    // initialise the IMU
    IMU* imu = (IMU*)malloc(sizeof(IMU));
    imu->a[0] = imu->a[1] = imu->a[2] = 0.0;
    imu->w[0] = imu->w[1] = imu->w[2] = 0.0;
    imu->dt = DT;
    imu->a_real = imu->v_real = imu->h_real = 0.0;


    // initialise PID and ramp
    float error_prior = 0, integral_prior = 0;
    float kp = 0.0001, ki = 0.01, kd = 0.00001, b = 0;
    float sig_des, sign;


    // setup variables
    float t = 0;
    float A_min = 0.25, A = 0.25;
    float cd = 0.1;
    float v = 0;
    float h = 0;
    float a_f;
    float a;


    // initialise the kalman filter
    int step = 0;
    float a_0 = accel_curve(0, cd, A, v);
    KF kf(a_0, 0.0, 0.0, 1000);


    while (1) {
        // usleep(micro);
        // printf("t: %f, a: %f, v: %f, h: %f\n", t, d->a, d->v, d->h);

        if (imu->h_real < 0 && t > 2) {
            break;
        }
        if (!(step++ % KF_EVERY)) {
            kf.update(a, 10);
        }

        kf.predict(DT);
        a_f = kf.x(0,0);
        v = kf.x(1,0);
        h = kf.x(2,0);

        //     a_f = a; // apply filterring here
        //     v = v + a_f * DT;
        //     h = h + v * DT;

        cd = drag_coeff();
        imu = generate_model_data(imu, t, A, cd);
        a = imu->a[0];
        t += DT;

        fprintf(csv, "%.4f,%.4f,%.4f,%.4f\n", t, a, a_f, imu->a_real);
    }


    fclose(csv);
    return 0;
}