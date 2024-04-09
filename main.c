#include "simpletools.h"
#include "MPU6050.h"
#include "KalmanFilter.h"
#include "servo.h"


#define servoY 12
#define servoX 13
#define reset_pin 3

volatile float x_angle = 0;
volatile float y_angle = 0;
volatile float z_angle = 0;


void readAngle(){
  float ax, ay, az, gx, gy, gz;
  float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.003, R_angle = 0.03, C_0 = 1, K1 = 0.05;
  float local_x_angle =   0;
  float local_y_angle = 0;
  float local_z_angle = 0;
  float epsilon = 0.5;
  float threshold = 1.2; // *sensitive parameter*
  float scale1 = 10;
  float scale2 = 21.89; // *sensitive parameter*
  float power = 1.091; // *sensitive parameter*
  float multiplier = 1.62;
  
  
  MPU6050 mpu;
  KalmanFilter kalmanfilter;
  MPU6050_init(&mpu);

  while(1) {
      MPU6050_readData(&mpu, &ax, &ay, &az, &gx, &gy, &gz);
      Angle(&kalmanfilter, ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
      if (abs(gx) > epsilon) {
        if (abs(gx) > threshold) {
          x_angle += ((gx<0) ? -pow(abs(gx), power) : pow(gx, power)) / scale2 * multiplier;
        }else{
          x_angle += gx / scale1 * multiplier;
        }
      }        
      if (abs(gy) > epsilon) {
        if (abs(gy) > threshold) {
          y_angle += ((gy<0) ? -pow(abs(gy), power) : pow(gy, power)) / scale2 * multiplier;
        }else{
          y_angle += gy / scale1 * multiplier;
        }
      }       
      if (abs(gz) > epsilon) {
        if (abs(gz) > threshold) {
          z_angle += ((gz<0) ? -pow(abs(gz), power) : pow(gz, power)) / scale2 * multiplier;
        }else{
          z_angle += gz / scale1 * multiplier;
        }
      }    
      if (!input(reset_pin)){
        x_angle = 0;
        y_angle = 0;
        z_angle = 0;
      }        
      
      // print("Accel X: %.2f, Accel Y: %.2f, Accel Z: %.2f, Gyro X: %.2f, Gyro Y: %.2f, Gyro Z: %.2f\n", ax, ay, az, gx, gy, gz);
      // print("kalmanfilter x angle = %f", kalmanfilter.angle);
      print("                                          Gyro X: %.2f, Gyro Y: %.2f, Gyro Z: %.2f\n", gx, gy, gz);
      print("x_angle: %f, y_angle: %f, z_angle: %f \n", x_angle, y_angle, z_angle);
      
      pause(3);
  }
}


int main() {
    set_direction(reset_pin, 0);
    // initial servo setup
    servo_angle(12, 900);
    servo_angle(13, 900);

    int stack[256];
    // cogstart(&readAngle, NULL, stack, sizeof(stack)); 
    
    float y = 0;
    float x = 0;

    int cnt = 0;

    while(1){
      cnt += 1;
      readAngle();

      if (cnt > 10){
        cnt = 0;
        y = (int)y_angle;
        x = (int)x_angle;

        print("y_angle is %d", y);
        print("x_angle is %d", x);
        
        servo_angle(12, 900-(10*y));
        servo_angle(13, 900-(10*x));
      }      
      pause(3);
    }     
    return 0;
}
