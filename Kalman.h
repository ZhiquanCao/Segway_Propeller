*/
#include "simpletools.h"                      // Include simple tools
#define 
// 𝑥t =𝐴t 𝑥(t-1)+ 𝐵t𝑢t+ 𝑛oise

double kalman(double newAngle, double gyro_ang, double dt){
    angle += dt * (gyro_ang - bias);
  
  //Now the covariance matrix
  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_gyroBias * dt;
  
  // Innovation 
  y = newAngle - angle;
  
  //Intermediate to Kalman filter S 
  S = P[0][0] + R_measure;
  
  // Kalman gain 
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;
  
  // New estimate 
  angle += K[0] * y;
  bias += K[1] * y;
  
  // New covariance 
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
  
  return angle;
}  
  
  // To set Q_angle, Q_bias and R_measurement 

  
