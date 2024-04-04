#include "simpletools.h" // Include simpletools for convenience functions
#include "MPU6050.h"
#include "KalmanFilter.h"

int main() {
    float ax, ay, az, gx, gy, gz;
    float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;

    MPU6050 mpu;
    KalmanFilter kalmanfilter;

    MPU6050_init(&mpu);
    // Main loop to read and print gyro data
    float x_angle = 0;
    float y_angle = 0;
    float z_angle = 0;
    float scale = 11;
    float epsilon = 0.5;
    while(1) {
        MPU6050_readData(&mpu, &ax, &ay, &az, &gx, &gy, &gz);
        kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
        if (abs(gx) > epsilon) x_angle += gx;
        if (abs(gy) > epsilon) y_angle += gy;
        if (abs(gz) > epsilon) z_angle += gz;
        print("Accel X: %.2f, Accel Y: %.2f, Accel Z: %.2f, Gyro X: %.2f, Gyro Y: %.2f, Gyro Z: %.2f\n", ax, ay, az, gx, gy, gz);
        // print("kalmanfilter x angle = %f", kalmanfilter.angle);
        print("x_angle: %f, y_angle: %f, z_angle: %f \n", x_angle/scale, y_angle/scale, z_angle/scale);
        pause(5); // Wait for 1 second before reading again
    }
    return 0;
}
