#ifndef Make4e2ndChassis_h
#define Make4e2ndChassis_h

#include <Arduino.h>

class Make4e2ndChassis
{
  public:  
    void init();
    // CONSTRUCTORS
    Make4e2ndChassis(); // Default pin selection.
    Make4e2ndChassis(unsigned char M1DIR, unsigned char M1PWM, unsigned char M1FB,
                           unsigned char M2DIR, unsigned char M2PWM, unsigned char M2FB,
                           unsigned char nD2, unsigned char nSF); // User-defined pin selection. 
    
    // PUBLIC METHODS
    void setM1Speed(int speed); // Set speed for M1.
    void setM2Speed(int speed); // Set speed for M2.
    void setSpeeds(int m1Speed, int m2Speed); // Set speed for both M1 and M2.
    unsigned int getM1CurrentMilliamps(); // Get current reading for M1. 
    unsigned int getM2CurrentMilliamps(); // Get current reading for M2.
    unsigned char getFault(); // Get fault reading.
    
  private:
    unsigned char _nD2;
    unsigned char _M1DIR;
    unsigned char _M1DIR_B;
    unsigned char _M2DIR;
    unsigned char _M2DIR_B;
    unsigned char _M1PWM;
    unsigned char _M2PWM;
    unsigned char _nSF;
    unsigned char _M1FB;
    unsigned char _M2FB;
};

#endif
