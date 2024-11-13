#include "ht32.h"

extern "C" {
#include <stdio.h>
#include "GPIO.h"
#include "BFTM0.h"
//#include "ADC.h"
#include "UART1.h"
#include "SysTick.h"
//#include "Emulate_EEPROM.h"
#include "SSI.h"
//#include "WIFI.h"
//#include "FOC.h"
}
#include "mpu6050.h"

static void __Delay(u32 count)
{
  while (count--)
  {
    __NOP(); // Prevent delay loop be optimized
  }
}

void Init(void) {
  GPIO_Configuration();
  BFTM0_Configuration();
  //ADC_Configuration();
  UART1_Configuration();
  //Emulate_EEPROM_Configuration();
  SysTick_Configuration();
  MPU6050_Init();
	MT6701_Init();
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
    float age = MT6701_rad();
    printf("MT6701_rad():%f\n", age);
		__Delay(50000000);
  }
}

