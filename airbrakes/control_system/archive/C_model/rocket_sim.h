#ifndef ROCKET_SIM_H
#define ROCKET_SIM_H

typedef struct Data {
    float a;
    float v;
    float h;
    float dt;
} Data;

float accel_curve(float t, float cd, float A, float v);
Data* generate_model_data(Data* data_prev, float t, float A, float cd);

#endif // ROCKET_SIM_H