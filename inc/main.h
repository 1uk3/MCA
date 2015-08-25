void incTicks(void);
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "UART.h"
#include "adc.h"

#ifndef __MAINHEADERFILE
#define __MAINHEADERFILE

//buffer has to be 2^x
#define ADC_BUF_LEN 128		

#if (ADC_BUF_LEN & ADC_BUF_LEN-1) || (ADC_BUF_LEN<=0)
	#error Invalide ADC buffer size!
#endif

#define PRETRIGGER 25
#define TRIGGERLEVEL 150



#define GreenLED GPIO_Pin_12
#define OrangeLED GPIO_Pin_13
#define StepUpPin GPIO_Pin_6

#define ENOrangeLED()	GPIOD->BSRRL=OrangeLED
#define DISOrangeLED()	GPIOD->BSRRH =OrangeLED
#define ENGreenLED()	GPIOD->BSRRL=GreenLED
#define DISGreenLED()	GPIOD->BSRRH =GreenLED
#define ENStepUp()	GPIOC->BSRRL=StepUpPin
#define DISStepUp()	GPIOC->BSRRH =StepUpPin

#define setDAC(val)			*DAC_DEST=val & 0xFFF;


#define MAP_SIZE (2<<12)

#endif

uint32_t measureOffset(void);
void TimingDelay_Decrement(void);
