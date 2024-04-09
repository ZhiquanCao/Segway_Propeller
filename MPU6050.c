#include "simpletools.h" 
#include "simplei2c.h"
#include "MPU6050.h"


void MPU6050_init(MPU6050* mpu) {
    mpu->sclPin = 1;
    mpu->sdaPin = 0;
    mpu->GYRO_X_OFFSET = 0;
    mpu->GYRO_Y_OFFSET = 0;
    mpu->GYRO_Z_OFFSET = 0;
    mpu->ACCEL_X_OFFSET = 0;
    mpu->ACCEL_Y_OFFSET = 0;
    mpu->ACCEL_Z_OFFSET = 0;

    MPU6050_setup(mpu);
    MPU6050_calibrateSensors(mpu);
}

void MPU6050_setup(MPU6050* mpu) {
    // Open and initialize the I2C mpu->bus
    i2c_open(mpu->bus, mpu->sclPin, mpu->sdaPin, 0);
    // Setup MPU6050 (wake up and prepare for gyro reading)
    // Wake up MPU6050 by writing 0 to the PWR_MGMT_1 register
    i2c_start(mpu->bus);
    i2c_writeByte(mpu->bus, MPU6050_ADDR << 1); // Write address
    i2c_writeByte(mpu->bus, PWR_MGMT_1);        // Register address
    i2c_writeByte(mpu->bus, 0);                 // Write value to wake up MPU6050
    i2c_stop(mpu->bus);


    // Set FS_SEL to 0 for +/- 250 degrees/second full-scale range
    i2c_start(mpu->bus);
    i2c_writeByte(mpu->bus, MPU6050_ADDR << 1); // Write address
    i2c_writeByte(mpu->bus, GYRO_CONFIG);       // Gyro configuration register address
    i2c_writeByte(mpu->bus, 0);                 // Write value for FS_SEL = 0
    i2c_stop(mpu->bus);
    
}

void MPU6050_readData(MPU6050* mpu, float *ax, float *ay, float *az, float *gx,
                         float *gy, float *gz) {
    unsigned char data[14];

    i2c_start(mpu->bus);
    i2c_writeByte(mpu->bus, MPU6050_ADDR << 1);
    i2c_writeByte(mpu->bus, ACCEL_XOUT_H); // Start from the first accelerometer register
    i2c_stop(mpu->bus);

    i2c_start(mpu->bus);
    i2c_writeByte(mpu->bus, (MPU6050_ADDR << 1) | 1);
    i2c_readData(mpu->bus, data, 14); // Read 14 bytes to cover accel, temp (ignored), and gyro
    i2c_stop(mpu->bus);
    
    // Convert the data to 16-bit integers and then to float values
    *ax = ((int16_t)(data[0] << 8 | data[1]) - mpu->ACCEL_X_OFFSET) / 16384.0f;
    *ay = ((int16_t)(data[2] << 8 | data[3]) - mpu->ACCEL_Y_OFFSET) / 16384.0f;
    *az = ((int16_t)(data[4] << 8 | data[5]) - mpu->ACCEL_Z_OFFSET) / 16384.0f;
    *gx = ((int16_t)(data[8] << 8 | data[9]) - mpu->GYRO_X_OFFSET) / 131.0f; // Assuming FS_SEL = 0
    *gy = ((int16_t)(data[10] << 8 | data[11]) - mpu->GYRO_Y_OFFSET) / 131.0f;
    *gz = ((int16_t)(data[12] << 8 | data[13]) - mpu->GYRO_Z_OFFSET) / 131.0f;
}

void MPU6050_calibrateSensors(MPU6050* mpu) {
    long sumGX = 0, sumGY = 0, sumGZ = 0;
    long sumAX = 0, sumAY = 0, sumAZ = 0;
    int count = 0;
    int i;
    for (i=0; i<100; i++) {
        unsigned char data[14];
        i2c_start(mpu->bus);
        i2c_writeByte(mpu->bus, MPU6050_ADDR << 1);
        i2c_writeByte(mpu->bus, ACCEL_XOUT_H); // Start from the first accelerometer register
        i2c_stop(mpu->bus);

        i2c_start(mpu->bus);
        i2c_writeByte(mpu->bus, (MPU6050_ADDR << 1) | 1);
        i2c_readData(mpu->bus, data, 14); // Read 14 bytes to cover accel, temp (ignored), and gyro
        i2c_stop(mpu->bus);
        
        // Accelerometer data
        int ax = (int16_t)(data[0] << 8 | data[1]);
        int ay = (int16_t)(data[2] << 8 | data[3]);
        int az = (int16_t)(data[4] << 8 | data[5]);
        // Gyroscope data
        int gx = (int16_t)(data[8] << 8 | data[9]);
        int gy = (int16_t)(data[10] << 8 | data[11]);
        int gz = (int16_t)(data[12] << 8 | data[13]);

        sumAX += ax;
        sumAY += ay;
        sumAZ += az;
        sumGX += gx;
        sumGY += gy;
        sumGZ += gz;
        count++;
        
        pause(10); // Small delay to avoid overwhelming the I2C mpu->bus
    }
    
    // Calculate and set global offsets for both gyro and accelerometer
    mpu->GYRO_X_OFFSET = sumGX / count;
    mpu->GYRO_Y_OFFSET = sumGY / count;
    mpu->GYRO_Z_OFFSET = sumGZ / count;
    // Assuming you have defined global offsets for the accelerometer
    mpu->ACCEL_X_OFFSET = sumAX / count;
    mpu->ACCEL_Y_OFFSET = sumAY / count;
    mpu->ACCEL_Z_OFFSET = sumAZ / count; // Assuming the sensor is stationary and facing upwards
    
    //print("Calibration complete. Gyro Offsets: X=%d, Y=%d, Z=%d\n", mpu->GYRO_X_OFFSET, mpu->GYRO_Y_OFFSET, mpu->GYRO_Z_OFFSET);
    //print("Calibration complete. Accel Offsets: X=%d, Y=%d, Z=%d\n", mpu->ACCEL_X_OFFSET, mpu->ACCEL_Y_OFFSET, mpu->ACCEL_Z_OFFSET);
}