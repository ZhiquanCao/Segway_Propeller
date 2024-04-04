#ifndef MPU6050_H
#define MPU6050_H

#include "simplei2c.h"   // Assuming simplei2c.h can be used in C++
#include "simplei2c.h"   // Include the I2C library

class MPU6050 {
public:
    void setup();
    void readGyroData();
    void readData();
    void calibrateGyro();
    void calibrateSensors();

private:
    i2c *bus;  // Pointer to I2C bus instance
    int sclPin = 1; // Example SCL pin number
    int sdaPin = 0; // Example SDA pin number
    int GYRO_X_OFFSET = 0;
    int GYRO_Y_OFFSET = 0;
    int GYRO_Z_OFFSET = 0;
    int ACCEL_X_OFFSET = 0;
    int ACCEL_Y_OFFSET = 0;
    int ACCEL_Z_OFFSET = 0;

    // MPU6050 I2C address and register definitions
    static const int MPU6050_ADDR = 0x68;
    static const int PWR_MGMT_1   = 0x6B;
    static const int GYRO_XOUT_H  = 0x43;
    static const int GYRO_CONFIG  = 0x1B;
    static const int ACCEL_XOUT_H = 0x3B;
    static const int ACCEL_CONFIG = 0x1C;
};

#endif // MPU6050_H
