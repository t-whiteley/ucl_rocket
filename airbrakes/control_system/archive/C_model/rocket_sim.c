// EVERYTHING IN THIS FILE IS NOT IMPORTANT TO CONTROL ALGORITHM
// IT IS JUST SIMULATING THE DYNAMICS OF THE ROCKET TO IMPROVE THE CONTROL

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>


#define THRUST_TIME 2 // in [s]
#define THRUST_FORCE 2000 // in [N]
#define GRAV_CONST 9.81
#define RHO 1.293
#define MASS 5



// this structure contains all data that sensors would record
// (TODO) expand to 3 axis v and a
typedef struct Data {
    float a; // accel ms^-2
    float v; // vel ms^-2
    float h; // altitude m
    float dt; // time step
} Data;


// (TODO) thrust curve modelled off real data is better
float accel_curve(float t, float cd, float A, float v) {
    int a;
    if (t > THRUST_TIME) {
        a = -GRAV_CONST - (v/fabs(v))*(RHO * cd * A * v*v) / (2*MASS);
    } else {
        // abs(v) accounts for the direction of drag opposite to vel
        a = -GRAV_CONST - (RHO * cd * A * v*v) / (2*MASS) + THRUST_FORCE/MASS;
    }
    return a;
}


Data* generate_model_data(Data* data_prev, float t, float A, float cd) {
    srand(time(NULL));

    float a_new = accel_curve(t, cd, A, data_prev->v);
    float v_new = data_prev->v + a_new * data_prev->dt;
    float h_new = data_prev->h + v_new * data_prev->dt;


    Data *data_new = (Data *)malloc(sizeof(Data));
    data_new->a = a_new;
    data_new->v = v_new;
    data_new->h = h_new;
    data_new->dt = data_prev->dt;
    return data_new;
}