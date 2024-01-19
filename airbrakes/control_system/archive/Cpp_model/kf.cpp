#include "kf.h"

KF::KF(float init_a, float init_v, float init_h, float j_var) {
    x << init_a, init_v, init_h;
    P = Eigen::Matrix3f::Identity();
    j = j_var;
}


void KF::predict(float dt) {
    Eigen::Matrix<float, 3, 3> F;
    Eigen::Matrix<float, 3, 1> G;

    F << 1.0, 0.0, 0.0,
            dt, 1.0, 0.0,
            1.0/2*dt*dt, dt, 1.0;
    G << dt, 1.0/2*dt*dt, 1.0/6*dt*dt*dt;

    x = F * x;
    P = F * P * F.transpose() + j * G * G.transpose();
}


void KF::update(float meas_val, float meas_var) {
    Eigen::Matrix<float, 1, 3> H;
    H << 1.0, 0.0, 0.0;

    float y = meas_val - H * x;
    float S = H * P * H.transpose() + meas_var;
    Eigen::Matrix<float, 3, 1> K;
    K = P * H.transpose() * 1.0/S;

    x = x + K * y;
    P = (Eigen::Matrix3f::Identity() - K*H) * P;
}