#include "delay.h"

void delay(u32 count){
	  while (count--)
  {
    __NOP(); // Prevent delay loop be optimized
  }
}