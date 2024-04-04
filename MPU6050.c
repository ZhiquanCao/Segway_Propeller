#include "simpletools.h" // Include simpletools for convenience functions
#include "simplei2c.h"   // Include the I2C library
#include "MPU6050.h"     // Include the MPU6050 class header

void MPU6050::MPU6050() {
    setupMPU6050();
    calibrateSensors();
}

void MPU6050::setupMPU6050() {
    // Open and initialize the I2C bus
    i2c_open(&bus, sclPin, sdaPin, 0);
    
    // Setup MPU6050 (wake up and prepare for gyro reading)
    // Wake up MPU6050 by writing 0 to the PWR_MGMT_1 register
    i2c_start(bus);
    i2c_writeByte(bus, MPU6050_ADDR << 1); // Write address
    i2c_writeByte(bus, PWR_MGMT_1);        // Register address
    i2c_writeByte(bus, 0);                 // Write value to wake up MPU6050
    i2c_stop(bus);

    // Set FS_SEL to 0 for +/- 250 degrees/second full-scale range
    i2c_start(bus);
    i2c_writeByte(bus, MPU6050_ADDR << 1); // Write address
    i2c_writeByte(bus, GYRO_CONFIG);       // Gyro configuration register address
    i2c_writeByte(bus, 0);                 // Write value for FS_SEL = 0
    i2c_stop(bus);
    
}

void MPU6050::readGyroData() {
    unsigned char data[6];
    float gx, gy, gz;  // Change the type from int to float

    // Tell MPU6050 we want to start reading from the GYRO_XOUT_H register
    i2c_start(bus);
    i2c_writeByte(bus, MPU6050_ADDR << 1);
    i2c_writeByte(bus, GYRO_XOUT_H);
    i2c_stop(bus);
    
    i2c_start(bus);
    i2c_writeByte(bus, (MPU6050_ADDR << 1) | 1); // MPU6050 read address
    i2c_readData(bus, data, 6);
    i2c_stop(bus);
    
    // Convert the data to 16-bit integers and then to float values representing degrees/second
    gx = ((int16_t)(data[0] << 8 | data[1]) - GYRO_X_OFFSET) / 131.0f;
    gy = ((int16_t)(data[2] << 8 | data[3]) - GYRO_Y_OFFSET) / 131.0f;
    gz = ((int16_t)(data[4] << 8 | data[5]) - GYRO_Z_OFFSET) / 131.0f;
    
    // Print the gyroscopic data as floating-point numbers
    print("Gyro X: %.2f, Gyro Y: %.2f, Gyro Z: %.2f\n", gx, gy, gz);
}

void MPU6050::readMPU6050Data() {
    unsigned char data[14];
    float gx, gy, gz, ax, ay, az;

    // Start reading from ACCEL_XOUT_H, auto-increment through to GYRO_ZOUT_L
    i2c_start(bus);
    i2c_writeByte(bus, MPU6050_ADDR << 1);
    i2c_writeByte(bus, ACCEL_XOUT_H);
    i2c_stop(bus);
    
    i2c_start(bus);
    i2c_writeByte(bus, (MPU6050_ADDR << 1) | 1); // MPU6050 read address
    i2c_readData(bus, data, 14); // Read 14 bytes: ACC_XYZ + TEMP + GYRO_XYZ
    i2c_stop(bus);
    
    // Convert the data to 16-bit integers and then to float values
    ax = ((int16_t)(data[0] << 8 | data[1]) - ACCEL_X_OFFSET) / 16384.0f; // Assuming AFS_SEL = 0
    ay = ((int16_t)(data[2] << 8 | data[3]) - ACCEL_Y_OFFSET) / 16384.0f;
    az = ((int16_t)(data[4] << 8 | data[5]) - ACCEL_Z_OFFSET) / 16384.0f;
    gx = ((int16_t)(data[8] << 8 | data[9]) - GYRO_X_OFFSET) / 131.0f; // Assuming FS_SEL = 0
    gy = ((int16_t)(data[10] << 8 | data[11]) - GYRO_Y_OFFSET) / 131.0f;
    gz = ((int16_t)(data[12] << 8 | data[13]) - GYRO_Z_OFFSET) / 131.0f;
    
    // Print the data
    print("Accel X: %.2f, Accel Y: %.2f, Accel Z: %.2f, Gyro X: %.2f, Gyro Y: %.2f, Gyro Z: %.2f\n", ax, ay, az, gx, gy, gz);
}


void MPU6050::calibrateGyro() {
    const int durationMs = 3000; // Duration for calibration (3 seconds)
    const int startTime = CNT; // Current Propeller counter (CNT)
    const int clkfreqMs = CLKFREQ / 1000; // Clock ticks per millisecond
    
    long sumX = 0, sumY = 0, sumZ = 0;
    int count = 0;
    
    while(CNT - startTime < durationMs * clkfreqMs) {
        unsigned char data[6];
        i2c_start(bus);
        i2c_writeByte(bus, MPU6050_ADDR << 1);
        i2c_writeByte(bus, GYRO_XOUT_H);
        i2c_stop(bus);
    
        i2c_start(bus);
        i2c_writeByte(bus, (MPU6050_ADDR << 1) | 1);
        i2c_readData(bus, data, 6);
        i2c_stop(bus);
        
        int gx = (int16_t)(data[0] << 8 | data[1]);
        int gy = (int16_t)(data[2] << 8 | data[3]);
        int gz = (int16_t)(data[4] << 8 | data[5]);
        
        sumX += gx;
        sumY += gy;
        sumZ += gz;
        count++;
        
        pause(10); // Small delay to avoid overwhelming the I2C bus
    }
    
    // Calculate and set global offsets
    GYRO_X_OFFSET = sumX / count;
    GYRO_Y_OFFSET = sumY / count;
    GYRO_Z_OFFSET = sumZ / count;
    
    print("Calibration complete. Offsets: X=%d, Y=%d, Z=%d\n", GYRO_X_OFFSET, GYRO_Y_OFFSET, GYRO_Z_OFFSET);
}

void MPU6050::calibrateSensors() {
    const int durationMs = 3000; // Duration for calibration (3 seconds)
    const int startTime = CNT; // Current Propeller counter (CNT)
    const int clkfreqMs = CLKFREQ / 1000; // Clock ticks per millisecond
    
    long sumGX = 0, sumGY = 0, sumGZ = 0;
    long sumAX = 0, sumAY = 0, sumAZ = 0;
    int count = 0;
    
    while(CNT - startTime < durationMs * clkfreqMs) {
        unsigned char data[14];
        i2c_start(bus);
        i2c_writeByte(bus, MPU6050_ADDR << 1);
        i2c_writeByte(bus, ACCEL_XOUT_H); // Start from the first accelerometer register
        i2c_stop(bus);
    
        i2c_start(bus);
        i2c_writeByte(bus, (MPU6050_ADDR << 1) | 1);
        i2c_readData(bus, data, 14); // Read 14 bytes to cover accel, temp (ignored), and gyro
        i2c_stop(bus);
        
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
        
        pause(10); // Small delay to avoid overwhelming the I2C bus
    }
    
    // Calculate and set global offsets for both gyro and accelerometer
    GYRO_X_OFFSET = sumGX / count;
    GYRO_Y_OFFSET = sumGY / count;
    GYRO_Z_OFFSET = sumGZ / count;
    // Assuming you have defined global offsets for the accelerometer
    ACCEL_X_OFFSET = sumAX / count;
    ACCEL_Y_OFFSET = sumAY / count;
    ACCEL_Z_OFFSET = sumAZ / count; // Assuming the sensor is stationary and facing upwards
    
    print("Calibration complete. Gyro Offsets: X=%d, Y=%d, Z=%d\n", GYRO_X_OFFSET, GYRO_Y_OFFSET, GYRO_Z_OFFSET);
    print("Calibration complete. Accel Offsets: X=%d, Y=%d, Z=%d\n", ACCEL_X_OFFSET, ACCEL_Y_OFFSET, ACCEL_Z_OFFSET);
}