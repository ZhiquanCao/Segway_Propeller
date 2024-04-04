#include "simpletools.h" // Include simpletools for convenience functions
#include "MPU6050.h"
// MPU6050 I2C address and register definitions
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define GYRO_XOUT_H  0x43
#define GYRO_CONFIG 0x1B // The register for gyroscope configuration
#define ACCEL_XOUT_H    0x3B // The register for accelerometer X-axis high byte
#define ACCEL_CONFIG    0x1C // The register for accelerometer configuration

int GYRO_X_OFFSET = 0;
int GYRO_Y_OFFSET = 0;
int GYRO_Z_OFFSET = 0;

int ACCEL_X_OFFSET = 0;
int ACCEL_Y_OFFSET = 0;
int ACCEL_Z_OFFSET = 0;
    
int main() {
    MPU6050 mpu;
    // Main loop to read and print gyro data
    while(1) {
        mpu.readMPU6050Data();
        pause(100); // Wait for 1 second before reading again
    }
}
