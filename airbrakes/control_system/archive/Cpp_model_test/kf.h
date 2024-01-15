#include <iostream>
#include <eigen-3.4.0/Eigen/Dense>
#pragma once

class KF {
    public:
    Eigen::Matrix<float, 3, 1> x;
    Eigen::Matrix<float, 3, 3> P;
    float j;

    KF(float init_a, float init_v, float init_h, float j_var);
    void predict(float dt);
    void update(float meas_val, float meas_var);
};