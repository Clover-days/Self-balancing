#ifndef __MPU6050_H
#define __MPU6050_H
extern "C"{
#include "I2C1.h"
#include "string.h"
#include "math.h"
#include "SysTick.h"
}
#include "Kalman.h"
void MPU6050_Init(void);
float MPU6050_read(void);
#endif