#ifndef KalmanFilter_h
#define KalmanFilter_h
#include <stdint.h>

// Struct definition
typedef struct {
    float Gyro_x, Gyro_y, Gyro_z;
    float accelz;
    float angle;
    float angle6;
    float angle_err;
    float q_bias;
    float Pdot[4];
    float P[2][2];
    float PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
    float angle_dot;
} KalmanFilter;

void KalmanFilter_init(KalmanFilter *kf);
void Yiorderfilter(KalmanFilter *kalmanFilter, float angle_m, float gyro_m, float dt, float K1);
void Kalman_Filter(KalmanFilter *kalmanFilter, double angle_m, double gyro_m, float dt, float Q_angle, float Q_gyro, float R_angle, float C_0);
void Angle(KalmanFilter *kalmanFilter, int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float dt, float Q_angle, float Q_gyro, float R_angle, float C_0, float K1);

#endif;