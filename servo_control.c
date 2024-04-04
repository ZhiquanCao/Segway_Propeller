#include "servo.h"
#include "simpletools.h" 

int main()
{
  int n;
  for(n = 0; n <= 5; n+=1)
          {
          servo_set(12, 500); // Hold for 500us position: 0degree
          pause(2000); // For 2 seconds
          servo_set(12, 1400); // Hold the 1400us position: 90degree
          pause(2000); // For 2 seconds
          servo_set(12, 2300); // Hold the 2400us position: 180degree
          pause(2000); // For 2 seconds
        }          
          
 }
                  
