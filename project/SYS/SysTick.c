 /************************************************************************************************************
 * @attention
 *
 * Firmware Disclaimer Information
 *
 * 1. The customer hereby acknowledges and agrees that the program technical documentation, including the
 *    code, which is supplied by Holtek Semiconductor Inc., (hereinafter referred to as "HOLTEK") is the
 *    proprietary and confidential intellectual property of HOLTEK, and is protected by copyright law and
 *    other intellectual property laws.
 *
 * 2. The customer hereby acknowledges and agrees that the program technical documentation, including the
 *    code, is confidential information belonging to HOLTEK, and must not be disclosed to any third parties
 *    other than HOLTEK and the customer.
 *
 * 3. The program technical documentation, including the code, is provided "as is" and for customer reference
 *    only. After delivery by HOLTEK, the customer shall use the program technical documentation, including
 *    the code, at their own risk. HOLTEK disclaims any expressed, implied or statutory warranties, including
 *    the warranties of merchantability, satisfactory quality and fitness for a particular purpose.
 *
 * <h2><center>Copyright (C) Holtek Semiconductor Inc. All rights reserved</center></h2>
 ************************************************************************************************************/
//Generated by HT32CodeConfig V1.1.1 Build 231207
//-----------------------------------------------------------------------------
#include "SysTick.h"

//-----------------------------------------------------------------------------
#define TICK_TIMER_BASE            ((long long)SystemCoreClock * 10/1000000)

//-----------------------------------------------------------------------------
vu32 tick_ct;

//-----------------------------------------------------------------------------
void SysTick_Configuration(void)
{
  SYSTICK_ClockSourceConfig(SYSTICK_SRC_FCLK);
  SYSTICK_SetReloadValue(TICK_TIMER_BASE);
  SYSTICK_IntConfig(ENABLE);
  SYSTICK_CounterCmd(SYSTICK_COUNTER_CLEAR);
  SYSTICK_CounterCmd(SYSTICK_COUNTER_ENABLE);
}

//-----------------------------------------------------------------------------
void SysTick_Handler(void)
{
  tick_ct++;
}

//-----------------------------------------------------------------------------
u32 micros(void){
  return tick_ct*10;
}


