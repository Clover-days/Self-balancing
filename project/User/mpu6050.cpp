#include "mpu6050.h"
#define gyroZ_OFF -0.19
Kalman kalmanZ;
double gyroZangle;  // 仅使用陀螺仪计算角度
double compAngleZ;  // 使用互补滤波片计算角度
double kalAngleZ;   // 使用卡尔曼滤波计算角度
float target_angle = 89.5;     //平衡角度 例如TA89.3 设置平衡角度89.3
const u8  MPU6050I2C = 0x68;
u8 mpu6050data[14];
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
uint32_t timer;
extern u32 micros(void);
static void __Delay(u32 count)
{
  while (count--)
  {
    __NOP(); // Prevent delay loop be optimized
  }
}

void I2c_printf(u8 dev_addr, u16 reg_addr, u16 len) {
  u8 data[len];
  char str[7*len];
  I2C1_read(dev_addr, reg_addr, data, len);
  I2C1_read_waiting();
  u8 i;
  for (i = 0;i < len;i++) {
    char temp[7];
    sprintf(temp, "%X:%d;", reg_addr + i, data[i]);
    strcat(str, temp);
  }
  strcat(str, "\n");
  printf("%s", str);
}

/* mpu6050加速度转换为角度
            acc2rotation(ax, ay)
            acc2rotation(az, ay) */
double acc2rotation(double x, double y) {
  double tmp_kalAngleZ = (atan(x / y) / 1.570796 * 90);
  if (y < 0) {
    return (tmp_kalAngleZ + 180);
  } else if (x < 0) {
    //将当前值与前值比较，当前差值大于100则认为异常
    if (!isnan(kalAngleZ) && (tmp_kalAngleZ + 360 - kalAngleZ) > 100) {
      if (tmp_kalAngleZ < 0 && kalAngleZ < 0)  //按键右边角
        return tmp_kalAngleZ;
      else  //按键边异常处理
        return tmp_kalAngleZ;
    } else
      return (tmp_kalAngleZ + 360);
  } else {
    return tmp_kalAngleZ;
  }
}
//将角度限制在-60~60之间的函数
float constrainAngle(float x) {
  float a = 0;
  if (x < 0) {
    a = 120 + x;
    if (a < abs(x))
      return a;
  }
  return x;
}
//-----------------------------------------------------------------------------------------------------------------------------------
void MPU6050_Init(void) {
    I2C1_Configuration();
    mpu6050data[0] = 0x19;                      //mpu6050写入寄存器地址位
    mpu6050data[1] = 7;                         // 将采样率设置为 1000Hz - 8kHz/（7+1） = 1000Hz
    mpu6050data[2] = 0x00;                      // 禁用 FSYNC 并设置 260 Hz Acc 滤波、256 Hz 陀螺仪滤波、8 KHz 采样
    mpu6050data[3] = 0x00;                      // 将陀螺仪满量程范围设置为 ±250deg/s
    mpu6050data[4] = 0x00;                      // 将加速度计满量程范围设置为 ±2g
    I2C1_write(MPU6050I2C, mpu6050data, 5);     // 一次写入所有四个寄存器
    I2C1_write_waiting();
    I2c_printf(MPU6050I2C, 0x19, 5);
    u8 temp[2] = { 0x6B, 0x01 };
    I2C1_write(MPU6050I2C, temp, 2);
    I2C1_write_waiting();
    I2c_printf(MPU6050I2C, 0x6B, 1);
    I2C1_read(MPU6050I2C, 0x75, mpu6050data, 1);
    I2C1_read_waiting();
    u8 temp1[32];
    if (mpu6050data[0] != 0x68) {
      printf("Error reading sensor%d\n", mpu6050data[0]);
    }
    I2C1_read(MPU6050I2C, 0x3B, mpu6050data, 14);
    I2C1_read_waiting();
    accX = (int16_t)((mpu6050data[0] << 8) | mpu6050data[1]);
    accY = (int16_t)((mpu6050data[2] << 8) | mpu6050data[3]);
    accZ = (int16_t)((mpu6050data[4] << 8) | mpu6050data[5]);
    double pitch = acc2rotation(accX, accY);
    kalmanZ.setAngle(pitch);
    gyroZangle = pitch;
    timer = micros();
    printf("kalman mpu6050 init\n");
}
//-----------------------------------------------------------------------------------------------------------------------------------
float MPU6050_read(void) {
  I2C1_read(MPU6050I2C, 0x3B, mpu6050data, 14);
  I2C1_read_waiting();
  accX = (int16_t)((mpu6050data[0] << 8) | mpu6050data[1]);
  accY = (int16_t)((mpu6050data[2] << 8) | mpu6050data[3]);
  accZ = (int16_t)((mpu6050data[4] << 8) | mpu6050data[5]);
  gyroX = (int16_t)((mpu6050data[8] << 8) | mpu6050data[9]);
  gyroY = (int16_t)((mpu6050data[10] << 8) | mpu6050data[11]);
  gyroZ = (int16_t)((mpu6050data[12] << 8) | mpu6050data[13]);
  double dt = (double)(micros() - timer) / 1000000;  // 计算增量时间
  timer = micros();
  double pitch = acc2rotation(accX, accY);
  double gyroZrate = gyroZ / 131.0;  // 转换为 deg/s

  kalAngleZ = kalmanZ.getAngle(pitch, gyroZrate + gyroZ_OFF, dt);
  gyroZangle += (gyroZrate + gyroZ_OFF) * dt;
  compAngleZ = 0.93 * (compAngleZ + (gyroZrate + gyroZ_OFF) * dt) + 0.07 * pitch;

  //当陀螺仪漂移过大时重置陀螺仪角度
  if (gyroZangle < -180 || gyroZangle > 180)
    gyroZangle = kalAngleZ;

  float pendulum_angle = constrainAngle(fmod(kalAngleZ, 120) - target_angle);
  return pendulum_angle;
}