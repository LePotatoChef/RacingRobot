/* Include the Pololu library */
#include "Make4e2ndChassis.h"

/* Create the motor driver object */
Make4e2ndChassis drive;  //建立对象

/* Wrap the motor driver initialization */
void initMotorController() {
  drive.init();  //初始化pin脚输入输出模式 并初始化定时器1 设置PWM频率为20MHZ
  //Serial.println("init Make4e2ndChassis");
}

/* Wrap the drive motor set speed function */
void setMotorSpeed(int i, int spd) {
  if (i == LEFT) 
  {
    drive.setM1Speed(spd);  //设置左轮轮速
  }
  else
  {
    drive.setM2Speed(spd);  //设置右轮轮速
  }
 
}

// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed) {  
  setMotorSpeed(LEFT, leftSpeed);  //LEFT=0 设置左轮
  setMotorSpeed(RIGHT, rightSpeed); //RIGHT=1 设置右轮
//  Serial.print("leftPWM:");
//  Serial.print(leftSpeed);
//  Serial.print(" rightPWM:");
//  Serial.println(rightSpeed);
  
}


