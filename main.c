#include "simpletools.h" // Include simpletools for convenience functions
#include "MPU6050.h"
#include "KalmanFilter.h"

int main() {
    float ax, ay, az, gx, gy, gz;
    float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.005;

    MPU6050 mpu;
    MPU6050_init(&mpu);
    KalmanFilter kf;
    KalmanFilter_init(&kf);
    // Main loop to read and print gyro data
    while(1) {
        MPU6050_readData(&mpu, &ax, &ay, &az, &gx, &gy, &gz);
        Angle(&kf, ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
        print("Accel X: %.2f, Accel Y: %.2f, Accel Z: %.2f, Gyro X: %.2f, Gyro Y: %.2f, Gyro Z: %.2f\n", ax, ay, az, gx, gy, gz);
        print("kalmanfilter x angle = %f", kf.angle);
        pause(5); // Wait for 1 second before reading again
    }
    return 0;
}
