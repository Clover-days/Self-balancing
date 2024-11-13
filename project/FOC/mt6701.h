#ifndef __MT6701_H
#define __MT6701_H
extern "C" {
#include "delay.h"
#include "SysTick.h"
#include "I2C0.h"
#include "math.h"
}
class Sensor_MT6701
{
  public:
    Sensor_MT6701(int Mot_Num);
    void Sensor_init();
    void Sensor_update();
    float getAngle();
    float getVelocity();
    float getMechanicalAngle();
    double getSensorAngle();
  private:
    int _Mot_Num;
    //MT6701 变量定义
    //int sensor_direction=1;       //编码器旋转方向定义
    float angle_prev; // 最后一次调用 getSensorAngle() 的输出结果，用于得到完整的圈数和速度
    long angle_prev_ts; // 上次调用 getAngle 的时间戳
    float vel_angle_prev; // 最后一次调用 getVelocity 时的角度
    long vel_angle_prev_ts; // 最后速度计算时间戳
    int32_t full_rotations; // 总圈数计数
    int32_t vel_full_rotations; //用于速度计算的先前完整旋转圈数
};
void MT6701_Init(void);
float MT6701_rad(void);
#endif
