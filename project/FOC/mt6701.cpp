#include "mt6701.h"
#define _2PI 6.28318530718f
//extern u32 micros(void);
// mt6701 相关
double Sensor_MT6701::getSensorAngle() {

  u8 readArray[2];
  I2C0_read(0x06, 0x03,readArray, 2);
  I2C0_read_waiting();
  int16_t readValue =((int16_t)readArray[0] << 6) | (readArray[1] >> 2);
  return (readValue/ 16383.0 * _2PI); 

}

//MT6701 相关

//=========角度处理相关=============
Sensor_MT6701::Sensor_MT6701(int Mot_Num) {
   _Mot_Num=Mot_Num;  //使得 Mot_Num 可以统一在该文件调用
   
}
void Sensor_MT6701::Sensor_init() {
    I2C0_Configuration();
    delay(500);
    getSensorAngle(); 
    delay(24000);
    vel_angle_prev = getSensorAngle(); 
    vel_angle_prev_ts = micros();
    delay(1);
    getSensorAngle(); 
    delay(24000);
    angle_prev = getSensorAngle(); 
    angle_prev_ts = micros();
}
void Sensor_MT6701::Sensor_update() {
    float val = getSensorAngle();
    angle_prev_ts = micros();
    float d_angle = val - angle_prev;
    // 圈数检测
    if(abs(d_angle) > (0.8f*_2PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
}

float Sensor_MT6701::getMechanicalAngle() {
    return angle_prev;
}

float Sensor_MT6701::getAngle(){
    return (float)full_rotations * _2PI + angle_prev;
}

float Sensor_MT6701::getVelocity() {
    // 计算采样时间
    float Ts = (angle_prev_ts - vel_angle_prev_ts)*1e-6;
    // 快速修复奇怪的情况（微溢出）
    if(Ts <= 0) Ts = 1e-3f;
    // 速度计算
    float vel = ( (float)(full_rotations - vel_full_rotations)*_2PI + (angle_prev - vel_angle_prev) ) / Ts;    
    // 保存变量以待将来使用
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
    return vel;
}
