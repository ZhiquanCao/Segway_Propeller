#include "simpletools.h" // Include simpletools for convenience functions
#include "simplei2c.h"   // Include the I2C library

// MPU6050 I2C address and register definitions
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define GYRO_XOUT_H  0x43
#define GYRO_CONFIG 0x1B // The register for gyroscope configuration

// Function prototypes
void setupMPU6050(i2c *bus);
void readGyroData(i2c *bus);

int GYRO_X_OFFSET = 0;
int GYRO_Y_OFFSET = 0;
int GYRO_Z_OFFSET = 0;

int main() {
    i2c bus; // Declare an I2C bus instance
    int sclPin = 1; // Example SCL pin number
    int sdaPin = 0; // Example SDA pin number
    
    // Open and initialize the I2C bus
    i2c_open(&bus, sclPin, sdaPin, 0);
    
    // Setup MPU6050 (wake up and prepare for gyro reading)
    setupMPU6050(&bus);
    calibrateGyro(&bus);
    // Main loop to read and print gyro data
    while(1) {
        readGyroData(&bus);
        pause(100); // Wait for 1 second before reading again
    }
}

void setupMPU6050(i2c *bus) {
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

void readGyroData(i2c *bus) {
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


void calibrateGyro(i2c *bus) {
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