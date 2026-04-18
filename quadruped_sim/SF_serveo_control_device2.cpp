#include <Arduino.h>
#include "SF_Servo.h"
#include "SF_IMU.h"
#include <math.h>
#include "bipedal_data.h"
#include "MPU6050_tockn.h"
#include "SF_CAN.h"
#include "config.h"
#include "SF_BLDC.h"

SF_BLDC motors = SF_BLDC(Serial2);
SF_BLDC_DATA BLDCData;

#define myID 0x02


SF_CAN CAN;
#define TRANSMIT_RATE_MS 1
unsigned long previousMillis = 0;  // will store last time a message was send


uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void can_control();


float t;

motor_data MotorData;
motorstatus motorStatus;

// motor_data

void setup()
{
  Serial.begin(115200); // 用于调试
  CAN.init(CAN_TX, CAN_RX);
  CAN.setMode(1);
  CAN.setDeviceID(0x02);
  motors.init();
  motors.setModes(4, 4);
  // Serial.begin(115200);
  delay(10000);
}

void getMotorValue()
{
  BLDCData = motors.getBLDCData();
  motorStatus.M0Speed = BLDCData.M0_Vel;
  motorStatus.M1Speed = BLDCData.M1_Vel;
}


float motor1target, motor2target;
// float forwardBackward,steering;

float uint_to_float(int x_int, float x_min, float x_max, int bits){
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits){
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}
// uint8_t r_buf[8]={128, 0, 128, 0, 0, 0, 0, 0};
uint8_t r_buf[8];
void can_control()
{
  
  
  // unsigned long currentMillis = millis();
  // if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
  //   previousMillis = currentMillis;
  //   getMotorValue();

  //   uint8_t motorcommand[8];
  //   motorStatus.M0Speed = _constrain(motorStatus.M0Speed,-100,100);
  //   motorStatus.M1Speed = _constrain(motorStatus.M1Speed,-100,100);
  //   uint16_t target1_san = float_to_uint(motorStatus.M0Speed, -100.0, 100.0, 16); // 压缩为 16 位
  //   uint16_t target2_san = float_to_uint(motorStatus.M1Speed, -100.0, 100.0, 16); // 压缩为 16 位

  //   uint32_t t_id;
  //   t_id = 0x01;
  //   motorcommand[0] = target1_san >> 8;          // target1 高 8 位
  //   motorcommand[1] = target1_san & 0xFF;        // target1 低 8 位
  //   motorcommand[2] = target2_san >> 8;          // target2 高 8 位
  //   motorcommand[3] = target2_san & 0xFF;        // target2 低 8 位
  //   motorcommand[4] = 0;         
  //   motorcommand[5] = 0;       
  //   motorcommand[6] = 0;         
  //   motorcommand[7] = 0;

  //   CAN.sendMsg(&t_id, motorcommand);
  // }
  CAN.receiveMsg(r_buf);
  uint16_t target1_rec = (r_buf[0] << 8) | r_buf[1];
  uint16_t target2_rec = (r_buf[2] << 8) | r_buf[3];
  
    // 解压为浮点数
  motor1target = uint_to_float(target1_rec, -100.0, 100.0, 16);
  motor2target = uint_to_float(target2_rec, -100.0, 100.0, 16);
  // CAN.listenAllMsg(10);//监视接收到的信息

}
uint8_t cnt = 0;


  unsigned long lastTime = 0;
  unsigned long currentTime = micros();
  unsigned long loopTime;
  

void loop()
{
  // currentTime = micros();
  // loopTime = currentTime - lastTime;

  // lastTime = currentTime;


  can_control();
  // Serial.print("Motor Target 1: ");
  //   Serial.println(motorTargets.target1, 2);

  //   Serial.print("Motor Target 2: ");
  //   Serial.println(motorTargets.target2, 2);
  // motor1target = 0.18 * (forwardBackward + 0 * motorStatus.M0Speed) - 1 * steering / 20;
  // motor2target = 0.18 * (forwardBackward - 0 * motorStatus.M1Speed) - 1 * steering / 20;
  // motor1target = -motor1target;
  // motor2target = -motor2target;
  cnt++;
  if (cnt > 10)
  {
    cnt = 0;
    Serial.print(motor1target);
    Serial.print(",");
    Serial.println(motor2target);
    motors.setTargets(motor1target, motor2target);
    // motors.setTargets(2, 2);
  //   // Serial.print(",");
    // Serial.println(loopTime);
  // //   // Serial.printf("0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x \n ", 
  // //   //             r_buf[0], r_buf[1], r_buf[2], r_buf[3], r_buf[4], r_buf[5], r_buf[6], r_buf[7]);
  // //   // Serial.println("test");
  }
  //  motors.setTargets(motor1target, motor2target);
  
  // delay(10);
}
