#include "simpletools.h" // Include simpletools for convenience functions
#include "MPU6050.h"
#include "KalmanFilter.h"
#include "servo.h"


#define servoY = 12;
#define servoX = 13;

void servoControl() {
    // Loop indefinitely for servo control
    
        // Example servo control code
        // You can replace this with your servo control logic
        int l;
        for (l=0;l<=100;l+=1){
          servo_angle(12, 900);
          pause(100);
          servo_angle(12,1800);
          pause(100);
        }           // Assuming servo is connected to pin 12 and angle is set to 900 (example)
        
        // Adjust the delay based on your requirements
        pause(20); // Example pause time
    
}

volatile float x_angle = 0;
volatile float y_angle = 0;
volatile float z_angle = 0;



void readAngle(){
  float ax, ay, az, gx, gy, gz;
  float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.003, R_angle = 0.03, C_0 = 1, K1 = 0.05;
  MPU6050 mpu;
  KalmanFilter kalmanfilter;
  MPU6050_init(&mpu);
  // Main loop to read and print gyro data
  float local_x_angle =   0;
  float local_y_angle = 0;
  float local_z_angle = 0;
  float scale = 11;
  float epsilon = 0.5;
  while(1) {
      MPU6050_readData(&mpu, &ax, &ay, &az, &gx, &gy, &gz);
      kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
      if (abs(gx) > epsilon) local_x_angle += gx;
      if (abs(gy) > epsilon) local_y_angle += gy;
      if (abs(gz) > epsilon) local_z_angle += gz;
      x_angle = local_x_angle/scale;
      y_angle = local_y_angle/scale;
      z_angle = local_z_angle/scale;
      print("Accel X: %.2f, Accel Y: %.2f, Accel Z: %.2f, Gyro X: %.2f, Gyro Y: %.2f, Gyro Z: %.2f\n", ax, ay, az, gx, gy, gz);
      // print("kalmanfilter x angle = %f", kalmanfilter.angle);
      print("x_angle: %f, y_angle: %f, z_angle: %f \n", local_x_angle/scale, local_y_angle/scale, local_z_angle/scale);
      
      pause(3);
  }
}



int main() {
    servo_angle(12, 900);
    servo_angle(13, 900);
    int stack[128];
    //cogstart(&readAngle, NULL, stack, sizeof(stack)); 
    int cnt = 0;
    float y = 0;
    float x = 0;
    while(1){
      readAngle();
      cnt += 1;
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
//-(500+ x_angle*10)