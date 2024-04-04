#ifndef MPU6050_H
#define MPU6050_H

#include "simplei2c.h"   // Include the I2C library once

// Structure to represent the MPU6050 data and configuration
typedef struct {
    i2c* bus;  // Pointer to I2C bus mpu
    int sclPin; // Example SCL pin number
    int sdaPin; // Example SDA pin number
    int GYRO_X_OFFSET;
    int GYRO_Y_OFFSET;
    int GYRO_Z_OFFSET;
    int ACCEL_X_OFFSET;
    int ACCEL_Y_OFFSET;
    int ACCEL_Z_OFFSET;
} MPU6050;

// MPU6050 I2C address and register definitions
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define GYRO_XOUT_H  0x43
#define GYRO_CONFIG  0x1B
#define ACCEL_XOUT_H 0x3B
#define ACCEL_CONFIG 0x1C

// Function prototypes, operating on an MPU6050 mpu
void MPU6050_init(MPU6050* mpu);
void MPU6050_setup(MPU6050* mpu);
void MPU6050_readData(MPU6050* mpu, float *ax, float *ay, float *az, float *gx, float *gy, float *gz);
void MPU6050_calibrateSensors(MPU6050* mpu);

#endif // MPU6050_H
