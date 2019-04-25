#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "arm_math.h"
//#include "HwCfg.h"
#include "RM3100.h"
int main(void)
{
	delay_init(168);
	SystemConfig();
  while(1){
		MagIC_Measurement_All(); 		
	}
}

 