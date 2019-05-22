#include "Make4e2ndChassis.h"

// Constructors ////////////////////////////////////////////////////////////////

Make4e2ndChassis::Make4e2ndChassis()
{
  _M1PWM = 6;
  _M1DIR = 7;
  _M1DIR_B=8;

  _M2PWM = 9 ;
  _M2DIR = 11;
  _M2DIR_B=10;  
}

void Make4e2ndChassis::init()
{
// Define pinMode for the pins and set the frequency for timer1.

  pinMode(_M1DIR,OUTPUT);  //7
  pinMode(_M1DIR_B,OUTPUT);  //8
  pinMode(_M1PWM,OUTPUT);  //6
  pinMode(_M1FB,INPUT);  
  pinMode(_M2DIR,OUTPUT);  //11
  pinMode(_M2DIR_B,OUTPUT);  //10
  pinMode(_M2PWM,OUTPUT);  //9
  pinMode(_M2FB,INPUT);  
  pinMode(_nD2,OUTPUT);
  digitalWrite(_nD2,HIGH); // default to on
  pinMode(_nSF,INPUT);

  #if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__)  //如果是AVR_ATmega168或AVR_ATmega328P
  // Timer 1 configuration  定时器1配置
  // prescaler: clockI/O / 1 分频器系数
  // outputs enabled  使能输出
  // phase-correct PWM PWM输出
  // top of 400  最高400
  //
  // PWM frequency calculation PWM频率计算
  // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
  TCCR1A = 0b10100000; //寄存器配置
  TCCR1B = 0b00010001;
  ICR1 = 400;
  #endif
}

// Set speed for motor 1, speed is a number betwenn -400 and 400
void Make4e2ndChassis::setM1Speed(int speed)  //设置左轮速度
{
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction 方向
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;
  #if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__)
  OCR1A = speed;
  #else
  analogWrite(_M1PWM,speed * 51 / 80); // default to using analogWrite, mapping 400 to 255  //设置左轮速度  将0-400映射到0-255中
  #endif
  
  if (reverse)  //设置方向
  {
    digitalWrite(_M1DIR,HIGH);
    digitalWrite(_M1DIR_B,LOW);
      
  }
  else
  {
    digitalWrite(_M1DIR,LOW);
    digitalWrite(_M1DIR_B,HIGH);
  }
  
}

// Set speed for motor 2, speed is a number betwenn -400 and 400
void Make4e2ndChassis::setM2Speed(int speed)  //设置右轮速度
{
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;
  #if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__)
  OCR1B = speed;
  #else
  analogWrite(_M2PWM,speed * 51 / 80); // default to using analogWrite, mapping 400 to 255  
  #endif
  if (reverse)
  {
    digitalWrite(_M2DIR,HIGH);
    digitalWrite(_M2DIR_B,LOW);
  }
  else
  {
    digitalWrite(_M2DIR,LOW);
    digitalWrite(_M2DIR_B,HIGH);

  }
 
}

// Set speed for motor 1 and 2
void Make4e2ndChassis::setSpeeds(int m1Speed, int m2Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}

