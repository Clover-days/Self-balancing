#include "ht32.h"
int Sensor_DIR=-1;    //传感器方向
int Motor_PP=7;    //电机极对数
extern "C" {
#include "delay.h"
#include "GPIO.h"
#include "BFTM0.h"
//#include "ADC.h"
#include "UART1.h"
#include "SysTick.h"
//#include "Emulate_EEPROM.h"
//#include "WIFI.h"
//#include "FOC.h"
}
#include "mpu6050.h"
#include "foc.h"

void Init(void) {
  GPIO_Configuration();
  BFTM0_Configuration();
  //ADC_Configuration();
  UART1_Configuration();
  //Emulate_EEPROM_Configuration();
  SysTick_Configuration();
  MPU6050_Init();
  DFOC_Vbus(12.6);   //设定驱动器供电电压
  DFOC_M0_alignSensor(Motor_PP,Sensor_DIR);
	//WIFI_Init();
}


//-----------------------------------------------------------------------------
int main(void)
{
  Init();
  while (1)
  {
    if(HT_CKCU->APBCCR1 & (1 << 4))
      WDT_Restart();
    //GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_14, (FlagStatus)(!(bool)GPIO_ReadOutBit(HT_GPIOC, GPIO_PIN_14)));
    float pendulum_angle = MPU6050_read();
    printf("MPU6050_read():%f\n", pendulum_angle);
		delay(50000000);
  }
}

