#include "simpletools.h"
#include "MPU6050.h"
#include "KalmanFilter.h"
#include "servo.h"


#define servoY 12
#define servoX 13
#define reset_pin 3

static volatile float x_angle = 0;
static volatile float y_angle = 0;
static volatile float z_angle = 0;


void readAngle(){
  //simpleterm_open();
  //print("in angle");
  float ax, ay, az, gx, gy, gz;
  float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.003, R_angle = 0.03, C_0 = 1, K1 = 0.05;
  float local_x_angle =   0;
  float local_y_angle = 0;
  float local_z_angle = 0;
  float epsilon = 0.7;
  float threshold = 1.2; // *sensitive parameter*
  float scale1 = 10;
  float scale2 = 21.78; // *sensitive parameter*
  float power = 1.095; // *sensitive parameter*
  float multiplier = 0.90;
  
  
  MPU6050 mpu;
  KalmanFilter kalmanfilter;
  MPU6050_init(&mpu);
  
  while(1) {
    
      MPU6050_readData(&mpu, &ax, &ay, &az, &gx, &gy, &gz);
      Angle(&kalmanfilter, ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
      if (abs(gx) > epsilon) {
        if (abs(gx) > threshold) {
          x_angle += ((gx<0) ? -pow(abs(gx), power) : pow(gx, power)) / scale2 * (multiplier*0.9);
        }else{
          x_angle += gx / scale1 * (multiplier*0.9);
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
      //print("                                          Gyro X: %.2f, Gyro Y: %.2f, Gyro Z: %.2f\n", gx, gy, gz);
      //print("x_angle: %f, y_angle: %f, z_angle: %f \n", x_angle, y_angle, z_angle);
      
      pause(3);
  }
}


int main() {
  
  //simpleterm_close();
  set_direction(reset_pin, 0);
  // initial servo setup
  servo_angle(12, 900);
  servo_angle(13, 900);
  //readAngle();
  int stack[512];
  cogstart(&readAngle, NULL, stack, sizeof(stack)); 
  
  float y = 0;
  float x = 0;

  int cnt = 0;
  //simpleterm_close();
  //pause(10);
  
  //simpleterm_open();
  //pause(10);
  while(1){
    
    cnt += 1;
    

    if (cnt > 10){
      cnt = 0;
      //simpleterm_open();
      pause(10);
      print("y_angle is %f \n", y_angle);
      print("x_angle is %f \n", x_angle);
      
      //servo_angle(12, 900-(10*y));
      //servo_angle(13, 900-(10*x));
      //simpleterm_close();
    }      
    
    pause(10);
  }     
  
  return 0;
}
