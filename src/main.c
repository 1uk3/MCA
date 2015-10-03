#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include "stm32f4xx_conf.h"  
#include <stm32f4xx_dac.h>

#include "main.h"

uint32_t *DAC_DEST= (uint32_t *) (DAC_BASE + 0x00000008 + DAC_Align_12b_R);

//can be modified over UART/USB
volatile uint32_t thre=15;

//send map to the pc flag
uint8_t sendUpdate=0;

//value for the auto offset adjustment
uint32_t DACoffset=0;

volatile int32_t remainingSamples=0;
volatile int32_t peak =0;
//ADC buffer
volatile uint16_t data[ADC_BUF_LEN];
//spectrum
uint32_t map[MAP_SIZE];

void initDMA(void);
void initADC(void);
void initDAC(void);
void initIO(void);
void initUpdateTimer(void);
void delay_ms(__IO uint32_t nTime);

int main(){

  while(SysTick_Config(180000)==1); //1ms
  
  initADC();
  
  UART_Init();
  
  initIO();
	
	initUpdateTimer();
	
	ENGreenLED();
  ENStepUp();
  delay_ms(1000); //charge the big caps
	delay_ms(1000); //wait for the CW-Multiplier
	DISGreenLED();
	
	volatile uint32_t i;
	while(1){
		while(remainingSamples >= 0){}
		__disable_irq();
		//TODO do signal processing here
		map[peak]++;
			
		//send map
		if(sendUpdate==1){
			sendMap(&map[0]);
			sendUpdate=0;
		}
		
		remainingSamples=0;
		peak=0;
		initADC();
		__enable_irq();
    
  }
  return 0;
}

void ADC_IRQHandler(){
	uint16_t val;
	if(ADC_GetITStatus(ADC1,ADC_IT_EOC)==SET){ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);val = ADC1->DR;}
	else if(ADC_GetITStatus(ADC2,ADC_IT_EOC)==SET){ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);val = ADC2->DR;}
	else if(ADC_GetITStatus(ADC3,ADC_IT_EOC)==SET){ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);val = ADC3->DR;}
	else return;
	
	if(remainingSamples>0){
		if(val>peak)peak=val;
		if(remainingSamples==1){
			DMA_Cmd(DMA2_Stream0, DISABLE);
			remainingSamples=-1;
			return;
		}
		remainingSamples--;
		return;
	}

	if(remainingSamples==-1)return;
	
	if(val > thre){
		remainingSamples = ADC_BUF_LEN - PRETRIGGER;
	}
}

void TIM2_IRQHandler(){
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	
	sendUpdate=1;
}

void initDAC(){
  GPIO_InitTypeDef      GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  //DAC init
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC,ENABLE);
  
  DAC_InitTypeDef DAC_InitStruct;
  DAC_InitStruct.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
  DAC_InitStruct.DAC_Trigger = DAC_Trigger_None;
  DAC_Init(DAC_Channel_1,&DAC_InitStruct);
  
  DAC_Cmd(DAC_Channel_1, ENABLE);

  DAC_SetChannel1Data(DAC_Align_12b_R, DACoffset);
}

void initDMA(){
	DMA_DeInit(DMA2_Stream0);
	DMA_InitTypeDef       DMA_InitStructure;
		/* DMA2 Stream0 channel0 configuration */
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = ((uint32_t)0x40012308);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&data;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = ADC_BUF_LEN;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);

  /* DMA2_Stream0 enable */
  DMA_Cmd(DMA2_Stream0, ENABLE);
}

void initADC()
{
  ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef       ADC_CommonInitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  /* Enable ADC3, DMA2 and GPIO clocks ****************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3, ENABLE);

  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  ADC_Cmd(ADC1, DISABLE);
  
  ADC_DeInit();
	
	initDMA();
    
/******************************************************************************/
/*  ADCs configuration: triple interleaved with 5cycles delay to reach 6Msps  */
/******************************************************************************/

  /* ADC Common configuration *************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_Interl;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;  
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4; 
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC1 regular channel 10 configuration ************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);  
  
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);

  /* ADC2 regular channel 10 configuration ************************************/
  ADC_Init(ADC2, &ADC_InitStructure);
  
    /* ADC2 regular channel 10 configuration */
    ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);  

  /* ADC3 regular channel 10 configuration ************************************/
  ADC_Init(ADC3, &ADC_InitStructure); 
  
  /* ADC3 regular channel 10 configuration *************************************/
  ADC_RegularChannelConfig(ADC3, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);    

  /* Enable DMA request after last transfer (multi-ADC mode) ******************/
  ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure and enable ADC interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);
	ADC_ITConfig(ADC2,ADC_IT_EOC,ENABLE);
	ADC_ITConfig(ADC3,ADC_IT_EOC,ENABLE);

  /* Enable ADC1 **************************************************************/
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC2 **************************************************************/
  ADC_Cmd(ADC2, ENABLE);

  /* Enable ADC3 **************************************************************/
  ADC_Cmd(ADC3, ENABLE);
    
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConv(ADC1);
}



void initIO(void){
    
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC , ENABLE);
  
  GPIO_InitTypeDef      GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = OrangeLED;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GreenLED;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = StepUpPin;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void initUpdateTimer(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  /* TIM3 clock enable */
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 90000; // 1sec @ 180MHz
  TIM_TimeBaseStructure.TIM_Prescaler = 1000; 
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV4;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
		
	NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure and enable ADC interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
	TIM_ITConfig(TIM2, TIM_IT_Update,ENABLE);
	 
	TIM_Cmd(TIM2, ENABLE);

}

volatile uint32_t TimingDelay=0;

void delay_ms(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}


void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
