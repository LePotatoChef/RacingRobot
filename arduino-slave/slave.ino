#include <Arduino.h>
#include <Servo.h>

#include "order.h"
#include "slave.h"
#include "parameters.h"
#include "motor_drive.h"
#include "encoder_driver.h"
#include <PID_v1.h>

typedef struct
{
  double target;
  double currentEncoder;
  double lastEncoder;
  double error;
  double input;
  double output;
} PIDInfo;
PIDInfo leftInfo, rightInfo;
double wheeldiameter = 0.115;
//double encoderresolution = 1560.0;
double encoderresolution = 7390.0;

double Kp = 2.0, Ki = 5.0, Kd = 0.003;
PID leftPID(&leftInfo.input, &leftInfo.output, &leftInfo.target, Kp, Ki, Kd, DIRECT);  //输入值 输出值 目标值 比例 积分 微分 调整方向
PID rightPID(&rightInfo.input, &rightInfo.output, &rightInfo.target, Kp, Ki, Kd, DIRECT);
double pid_rate = 30.0;
double pidinterval = 1000.0 / pid_rate;
long serialtime;
long nextmotion;
int  moving;  //小车行动标志位 0 停止 1 行动


#define BTN_TEST A4
int testMode = 0;
long testDelayStepMillis = 33;
long testTimeCounter;
long testDuration=30000;
long nextTestStep;
int testStep = 0;
int testSpeed;
const int maxTestSpeed = 150;

// A pair of varibles to help parse serial commands (thanks Fergs)  帮忙解析串行命令的一对变量
int arg = 0;  //参数标志 初始化为0
int index = 0;

// Variable to hold an input character  保存一个输入字符的变量
char chr;

// Variable to hold the current single-character command 保存当前单字符命令的变量
char cmd;

// Character arrays to hold the first and second arguments  保存第一个和第二个参数的数组
char argv1[16];
char argv2[16];

// The arguments converted to integers  //参数转化为整数
long arg1;  //
long arg2;

//**角速度angular(0-1.6) 线速度line(0-0.25)
float angular=110;
float line=0;
int v_des_left=0;
int v_des_right=0;


#define AUTO_STOP_INTERVAL 1000
long lastMotorCommand = AUTO_STOP_INTERVAL;

bool is_connected = false; ///< True if the connection with the master is available
int8_t motor_speed = 0;  //电机速度
int16_t servo_angle = INITIAL_THETA;  //伺服电机初始角度 110
Servo servomotor;

void setup()
{
  // Init Serial
  Serial.begin(SERIAL_BAUD);   //波特率 115200
  
  initMotorController();  //初始化pin脚输入输出模式 并初始化定时器1 设置PWM频率为20MHZ  motor_driver.c

  pinMode(BTN_TEST, INPUT);  //A4
  digitalWrite(BTN_TEST, LOW);  //A4  设为u=LOW

  leftPID.SetMode(AUTOMATIC); //指定PID算法的运行计算过程是自动（AUTOMATIC）还是手动（MANUAL）。
                              //手动就是关闭PID算法的计算功能。调为AUTOMATIC模式时才会初始化PID算法，进行输出。 
  leftPID.SetSampleTime(pidinterval);//用以控制PID算法的采样时间，默认采样时间为200ms。 1000/30
  leftPID.SetOutputLimits(-255, 255); //调用此函数，将会使得Output输出范围钳制在一定的范围之内。 否则默认为0-255

  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(pidinterval);
  rightPID.SetOutputLimits(-255, 255);

  testMode = digitalRead(BTN_TEST); //读A4引脚的电平
  
  // Stop the car
  stop();

  // Init Servo  
  //servomotor.attach(SERVOMOTOR_PIN);  // SERVOMOTOR_PIN 伺服电机引脚
  // Order between 0 and 180
  //servomotor.write(INITIAL_THETA);  // Initial angle of the servomotorv 伺服电机初始角度 110

  // Wait until the arduino is connected to master
  
  while(!is_connected)
  {
    write_order(HELLO);  //写命令 hello=0
    wait_for_bytes(1, 1000);  //延时 等消息
    get_messages_from_serial(); //接收命令消息并处理
  }

}

void loop()
{
  get_messages_from_serial();  //接收命令消息并处理
  update_motors_orders();  //更新电机状态

  if (nextmotion <= millis() && moving == 1)  //当小车在运动时,每隔一段时间pidinterval进行一次速度调整 保持匀速
  {
    leftInfo.currentEncoder = readEncoder(LEFT);  //读取当前旋转的角度
    leftInfo.input = leftInfo.currentEncoder - leftInfo.lastEncoder;  //输入值为这一期间转过的角度

    leftInfo.error = leftInfo.target - leftInfo.input;  //误差值 target是速度 input是转过的角度
    leftPID.Compute(); //返回true 输出是经过计算的输出
    leftInfo.lastEncoder = readEncoder(LEFT);


    rightInfo.currentEncoder = readEncoder(RIGHT);
    rightInfo.input = rightInfo.currentEncoder - rightInfo.lastEncoder;

    rightInfo.error = rightInfo.target - rightInfo.input;
    rightPID.Compute();
    rightInfo.lastEncoder = readEncoder(RIGHT);
    
    setMotorSpeeds(leftInfo.output, rightInfo.output);  //设置速度
    nextmotion = millis() + pidinterval;  //下一个调整节点
  }
}

void update_motors_orders()
{
//  servomotor.write(constrain(servo_angle, THETA_MIN, THETA_MAX));   //角度 初始值为110 最小值为60 最大值为150
//  motor_speed = constrain(motor_speed, -SPEED_MAX, SPEED_MAX);      //速度 
 
  // Send motor speed order
//  if (motor_speed > 0)
//  {
//    digitalWrite(DIRECTION_PIN, LOW);
//  }
//  else
//  {
//    digitalWrite(DIRECTION_PIN, HIGH);
//  }
//  analogWrite(MOTOR_PIN, convert_to_pwm(float(motor_speed)));

servo_angle = constrain(servo_angle, THETA_MIN, THETA_MAX);
motor_speed = constrain(motor_speed, -SPEED_MAX, SPEED_MAX);

//**角度转换** 默认110为直走//
 angular = ((servo_angle - 110.0)/40.0)*1.6;

//**速度转换**//
 line = motor_speed/100.0*0.25;

convert(line,angular);  //速度值的转换

//Serial.println("line:");
//Serial.println(line);
//Serial.println("angular");
//Serial.println(angular);
//Serial.println("v_left:");
//Serial.println(v_des_left);

setTargetTicksPerFrame(v_des_left,v_des_right); //驱动电机
}

void stop()
{
  servo_angle = 110;
  motor_speed = 0;
  setTargetTicksPerFrame(0, 0);  //电机速度置零
}

int convert_to_pwm(float motor_speed)
{
  // TODO: compensate the non-linear dependency speed = f(PWM_Value)
  return (int) round(abs(motor_speed)*(255./100.));
}

//接收反馈消息
void get_messages_from_serial()
{
  if(Serial.available() > 0)
  {
    // The first byte received is the instruction
    Order order_received = read_order();  //读消息
    //Serial.println(order_received);
    
    if(order_received == HELLO)  //1 打招呼
    {
      is_connected = true;
      // If the cards haven't say hello, check the connection
      if(!is_connected)
      {
        is_connected = true;
        write_order(HELLO);
      }
      else
      {
        // If we are already connected do not send "hello" to avoid infinite loop
        write_order(ALREADY_CONNECTED);  //发消息 已经连接
      }
    }
    else if(order_received == ALREADY_CONNECTED)  //2 连接
    {
      is_connected = true;
    }
    else    //3 其他运动命令
    {
      switch(order_received)
      {
        case STOP:  //停
        {
          motor_speed = 0;
          stop();
          if(DEBUG)
          {
            write_order(STOP);  //发反馈消息
          }
          break;
        }
        case SERVO:  //改拐弯角度值
        {
          servo_angle = read_i16();  //从串口中读拐弯角度值
          if(DEBUG)
          {
            write_order(SERVO);
            write_i16(servo_angle);
          }
          break;
        }
        case MOTOR:  //改电机速度值
        {
          // between -100 and 100
          motor_speed = read_i8(); //读电机速度值
          if(DEBUG)
          {
            write_order(MOTOR);
            write_i8(motor_speed);
          }
          break;
        }
        // Unknown order
        default:
          write_order(ERROR);
          write_i16(404);
          //Serial.println(404);
          return;
      }
    }
    write_order(RECEIVED); // Confirm the reception
  }
}

//读消息
Order read_order()
{
//  chr = Serial.read();
//  Serial.println(chr);
//  
//  if('m' == chr || chr == 4)
//  {
//    Serial.println("go");
//    is_connected = true;
//    //servo_angle = 110;
//    motor_speed = 30;
//    //setTargetTicksPerFrame(100, 100);  //设置左右两个速度
//  }
//  
//  if('s' == chr)
//  {
//    Serial.println("stop");
//    stop();
//  }
     
  return (Order) Serial.read();
}

//延时 等消息
void wait_for_bytes(int num_bytes, unsigned long timeout)
{
  unsigned long startTime = millis();
  //Wait for incoming bytes or exit if timeout
  while ((Serial.available() < num_bytes) && (millis() - startTime < timeout)){}
}

// NOTE : Serial.readBytes is SLOW
// this one is much faster, but has no timeout
void read_signed_bytes(int8_t* buffer, size_t n)
{
  size_t i = 0;
  int c;
  while (i < n)
  {
    c = Serial.read();
    if (c < 0) break;
    *buffer++ = (int8_t) c; // buffer[i] = (int8_t)c;
    i++;
  }
}

int8_t read_i8()
{
  wait_for_bytes(1, 100); // Wait for 1 byte with a timeout of 100 ms
  return (int8_t) Serial.read();
}

int16_t read_i16()
{
  int8_t buffer[2];
  wait_for_bytes(2, 100); // Wait for 2 bytes with a timeout of 100 ms
  read_signed_bytes(buffer, 2);
  return (((int16_t) buffer[0]) & 0xff) | (((int16_t) buffer[1]) << 8 & 0xff00);
}

int32_t read_i32()
{
  int8_t buffer[4];
  wait_for_bytes(4, 200); // Wait for 4 bytes with a timeout of 200 ms
  read_signed_bytes(buffer, 4);
  return (((int32_t) buffer[0]) & 0xff) | (((int32_t) buffer[1]) << 8 & 0xff00) | (((int32_t) buffer[2]) << 16 & 0xff0000) | (((int32_t) buffer[3]) << 24 & 0xff000000);
}

//写回馈消息
void write_order(enum Order myOrder)
{
  uint8_t* Order = (uint8_t*) &myOrder;
  Serial.write(Order, sizeof(uint8_t));
  
//  Serial.println(*Order);
//  if(!is_connected)
//  Serial.println("no connect");
//  else
//  Serial.println("yes connect");
//  
//  Serial.println("Hello Raspberry,I am Arduino.");
}

void write_i8(int8_t num)
{
  Serial.write(num);
}

void write_i16(int16_t num)
{
  int8_t buffer[2] = {(int8_t) (num & 0xff), (int8_t) (num >> 8)};
  Serial.write((uint8_t*)&buffer, 2*sizeof(int8_t));
}

void write_i32(int32_t num)
{
  int8_t buffer[4] = {(int8_t) (num & 0xff), (int8_t) (num >> 8 & 0xff), (int8_t) (num >> 16 & 0xff), (int8_t) (num >> 24 & 0xff)};
  Serial.write((uint8_t*)&buffer, 4*sizeof(int8_t));
}

//*****************************************************
void setSpeedWithWheel(int leftSpeed, int rightSpeed)  //设置轮速
{
  if (leftSpeed == 0 && rightSpeed == 0)
  {
    setMotorSpeeds(0, 0);
    moving = 0;  //小车运动标志位 停止
  }
  else
  {
    moving = 1;
  }
  
  double leftPIDSpeed = double(leftSpeed) / 1000.0 * encoderresolution / wheeldiameter / PI / pid_rate;
  leftInfo.target = leftPIDSpeed;
  double rightPIDSpeed = double(rightSpeed) / 1000.0 * encoderresolution / wheeldiameter / PI / pid_rate;
  rightInfo.target = rightPIDSpeed;
}

void setTargetTicksPerFrame(int left, int right)
{
  if (left == 0 && right == 0)
  {
    setMotorSpeeds(0, 0);
    moving = 0;  //电机标志为0 停止
  }
  else
  {
    moving = 1;
  }
  leftInfo.target = left;
  rightInfo.target = right;
}

void convert(float line,float angular)
{
  float x = line;   //线速度
  float th = angular;   //角速度
  float left = 0;
  float right = 0;
  
  if (x == 0)
  {
    //原地旋转
    right = th * 0.25  * 1.0 / 2.0;
    left = (-1)*right;
  }
 else if(th == 0)
 {
  //直线向前 或向后
  left = right = x;
 }
 else
 {
  //Rotation about a point in space
  left = x - th * 0.25  * 1.0 / 2.0;
  right = x + th * 0.25  * 1.0 / 2.0;
 }

// How many encoder ticks are there per meter? 每米编码器记录数据的次数
float ticks_per_meter = 7392 * 1.0  / (0.115 * 3.14);

v_des_left = int(left * ticks_per_meter /30);
v_des_right = int(right * ticks_per_meter / 30);  
}

