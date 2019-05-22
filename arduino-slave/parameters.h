#ifndef PARAMETERS_H
#define PARAMETERS_H

#define SERIAL_BAUD 115200  // Baudrate
#define MOTOR_PIN 3
#define DIRECTION_PIN 4
#define SERVOMOTOR_PIN 6   //伺服电机引脚
#define INITIAL_THETA 110  // Initial angle of the servomotorv 伺服电机初始角度

// Min and max values for motors
#define THETA_MIN 70  //角度 以前为60
#define THETA_MAX 150
#define SPEED_MAX 100 //速度

// If DEBUG is set to true, the arduino will send back all the received messages
#define DEBUG true

#define LEFT            0
#define RIGHT           1

#endif

