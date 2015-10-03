void incTicks(void);
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "UART.h"
#include "adc.h"

#ifndef __MAINHEADERFILE
#define __MAINHEADERFILE

#define ADC_BUF_LEN 30		

#define PRETRIGGER 5

#define GreenLED GPIO_Pin_5
#define OrangeLED GPIO_Pin_5
#define StepUpPin GPIO_Pin_6

#define ENOrangeLED()	GPIOD->BSRRL=OrangeLED
#define DISOrangeLED()	GPIOD->BSRRH =OrangeLED
#define ENGreenLED()	GPIOA->BSRRL=GreenLED
#define DISGreenLED()	GPIOA->BSRRH =GreenLED
#define ENStepUp()	GPIOC->BSRRL=StepUpPin
#define DISStepUp()	GPIOC->BSRRH =StepUpPin

#define setDAC(val)			*DAC_DEST=val & 0xFFF;


#define MAP_SIZE (2<<12)

#endif

uint32_t measureOffset(void);
void TimingDelay_Decrement(void);
