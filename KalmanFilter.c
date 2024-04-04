#include "KalmanFilter.h"
#include <math.h>

void Yiorderfilter(KalmanFilter *kalmanFilter, float angle_m, float gyro_m, float dt, float K1) {
  kalmanFilter->angle6 = K1 * angle_m + (1 - K1) * (kalmanFilter->angle6 + gyro_m * dt);
}

void Kalman_Filter(KalmanFilter *kalmanFilter, double angle_m, double gyro_m, float dt, float Q_angle, float Q_gyro, float R_angle, float C_0) {
  kalmanFilter->angle += (gyro_m - kalmanFilter->q_bias) * dt;
  kalmanFilter->angle_err = angle_m - kalmanFilter->angle;
  kalmanFilter->Pdot[0] = Q_angle - kalmanFilter->P[0][1] - kalmanFilter->P[1][0];
  kalmanFilter->Pdot[1] = - kalmanFilter->P[1][1];
  kalmanFilter->Pdot[2] = - kalmanFilter->P[1][1];
  kalmanFilter->Pdot[3] = Q_gyro;
  kalmanFilter->P[0][0] += kalmanFilter->Pdot[0] * dt;
  kalmanFilter->P[0][1] += kalmanFilter->Pdot[1] * dt;
  kalmanFilter->P[1][0] += kalmanFilter->Pdot[2] * dt;
  kalmanFilter->P[1][1] += kalmanFilter->Pdot[3] * dt;
  kalmanFilter->PCt_0 = C_0 * kalmanFilter->P[0][0];
  kalmanFilter->PCt_1 = C_0 * kalmanFilter->P[1][0];
  kalmanFilter->E = R_angle + C_0 * kalmanFilter->PCt_0;
  kalmanFilter->K_0 = kalmanFilter->PCt_0 / kalmanFilter->E;
  kalmanFilter->K_1 = kalmanFilter->PCt_1 / kalmanFilter->E;
  kalmanFilter->t_0 = kalmanFilter->PCt_0;
  kalmanFilter->t_1 = C_0 * kalmanFilter->P[0][1];
  kalmanFilter->P[0][0] -= kalmanFilter->K_0 * kalmanFilter->t_0;
  kalmanFilter->P[0][1] -= kalmanFilter->K_0 * kalmanFilter->t_1;
  kalmanFilter->P[1][0] -= kalmanFilter->K_1 * kalmanFilter->t_0;
  kalmanFilter->P[1][1] -= kalmanFilter->K_1 * kalmanFilter->t_1;
  kalmanFilter->angle += kalmanFilter->K_0 * kalmanFilter->angle_err;
  kalmanFilter->q_bias += kalmanFilter->K_1 * kalmanFilter->angle_err;
  kalmanFilter->angle_dot = gyro_m - kalmanFilter->q_bias;
}

void Angle(KalmanFilter *kalmanFilter, int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float dt, float Q_angle, float Q_gyro, float R_angle, float C_0, float K1) {
  float Angle = atan2f(ay , az) * 57.3;
  kalmanFilter->Gyro_x = gx;
  Kalman_Filter(kalmanFilter, Angle, kalmanFilter->Gyro_x, dt, Q_angle, Q_gyro, R_angle, C_0);
  kalmanFilter->Gyro_z = gz;
}

void KalmanFilter_init(KalmanFilter *kf) {
    kf->Gyro_x = 0.0f;
    kf->Gyro_y = 0.0f;
    kf->Gyro_z = 0.0f;
    kf->accelz = 0.0f;
    kf->P[0][0] = 1; kf->P[0][1] = 0;
    kf->P[1][0] = 0; kf->P[1][1] = 1;
    kf->Pdot[0];
}
