#ifndef IMU_SIM_H
#define IMU_SIM_H

typedef struct IMU {
    float a[3]; // 3 axis accel [x, y, z]
    float w[3]; // 3 axis rotation

    // dont touch these in algorithm, they are for modelling
    float a_real; // real a in z-dir
    float v_real; // real v in z-dir
    float h_real; // real h in z-dir
    float dt; // time step

} IMU;




float accel_curve(float t, float cd, float A, float v);
float rand_normal();
IMU* generate_model_data(IMU* data_prev, float t, float A, float cd);



#endif // IMU_SIM_H