#include "simpletools.h" // Include simpletools for convenience functions
#include "MPU6050.h"

int main() {
    int16_t ax, ay, az, gx, gy, gz;
    MPU6050* mpu;
    MPU6050_init(mpu);
    // Main loop to read and print gyro data
    while(1) {
        ax = MPU6050_readData(mpu, &ax, &ay, &az, &gx, &gy, &gz);
        print("Accel X: %.2f, Accel Y: %.2f, Accel Z: %.2f, Gyro X: %.2f, Gyro Y: %.2f, Gyro Z: %.2f\n", ax, ay, az, gx, gy, gz);
        pause(100); // Wait for 1 second before reading again
    }
}
