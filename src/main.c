
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include "stm32f4xx_conf.h"  
#include <stm32f4xx_dac.h>

#include "HD44780_F3.h"
#include "main.h"

#define FIR_ORDER 21
//TODO THE FILTER IS A WILD GUESS! 
//fg= 100kHz
float fir_coeff[] ={ -0.0000, -0.0021, -0.0063, -0.0116, -0.0124, 0.0000, 0.0318, 0.0814, 0.1375, 0.1821,
    0.1992, 0.1821, 0.1375, 0.0814, 0.0318, 0.0000, -0.0124, -0.0116, -0.0063, -0.0021, -0.0000
};



uint32_t *DAC_DEST= (uint32_t *) (DAC_BASE + 0x00000008 + DAC_Align_12b_R);




//can be modified over UART/USB
volatile uint32_t thre=0;
volatile uint8_t sendCurveEnable=0;

//value for the auto offset adjustment
uint32_t DACoffset=0;

//ADC buffer
volatile uint16_t data[ADC_BUF_LEN];
//spectrum
uint32_t map[MAP_SIZE];


void filter(uint32_t ptr);
void initADC(void);
void initDAC(void);
void initIO(void);
uint32_t measureOffset(void);
void autoOffset(void);
void delay_ms(__IO uint32_t nTime);

int main(){
  while(SysTick_Config(48000)==1); //1ms
  
  initADC();
  
  UART_Init();
  
  initIO();

  ENGreenLED();
  ENStepUp();
  delay_ms(1000); //charge the big caps
  DISGreenLED();

  //initDAC();
  //autoOffset();
  thre = measureOffset()-TRIGGERLEVEL;
  UARTsendOffset(thre);
  
  //init map
  for(int i=0;i<MAP_SIZE;i++){
    map[i]=0;
  }
  
  uint32_t tmp;
  uint32_t cnt;
  uint32_t ptr=0;
  //main loop
  while(1){
    tmp = 99999;
    cnt = ADC_BUF_LEN - PRETRIGGER;
    
    uint32_t peak_ptr=0;
    uint32_t best=5000;
    
    //wait for trigger
    while(tmp > thre){
      waitADC();
      tmp = ADC1->DR; //read ADC result
      data[ptr]=tmp;
      ptr++;
      ptr=ptr & (ADC_BUF_LEN-1); //wrap
    }
    //bigger but faster ^^
    //fill the rest of the buffer
    while(cnt>0){
      waitADC();
      uint16_t tmp= ADC1->DR;
      data[ptr]=tmp;
      if(tmp<best){
        best=tmp;
        peak_ptr=ptr;
      }
      ptr++;
      ptr=ptr & (ADC_BUF_LEN-1); //wrap
      cnt--;
    }

    if(sendCurveEnable==1) UARTsendCurve(data,ADC_BUF_LEN,((ptr+1) & (ADC_BUF_LEN-1)));
    if(sendCurveEnable==1) UARTsendPeak(data[peak_ptr]);
    map[data[peak_ptr]]++;
    
  }
  return 0;
}

//ptr: index of the next write location (1 ahead of the last element)
//y(k)=sum(a(i)*x(k-i))
void filter(uint32_t ptr){
  
  int end_ptr = ptr -1;
  if(end_ptr<0)end_ptr=ADC_BUF_LEN-end_ptr;
  
  int last_el=end_ptr+FIR_ORDER+1;
  if(last_el>=ADC_BUF_LEN)last_el-=ADC_BUF_LEN;
  
  for(int32_t k=end_ptr;k!=last_el;k--){
    
    float sum=0.0f;
    for(int i=0;i<FIR_ORDER;i++){
      int index=k-i;
      if(index<0)index=ADC_BUF_LEN+index;
      
      sum+=fir_coeff[i]*(float)data[index];
    }
     data[k]=(int)sum;
    if(k==0)k=ADC_BUF_LEN;
  }
}

//TODO limit max number of iterations
void autoOffset(){
  //set 4096-200 < offset <=4096-100
  
  while(thre == 0){
    
    uint32_t avg = measureOffset();
    
    if(avg > MAP_SIZE-100){          //decrease avg
      DACoffset += 50;
    }else if(avg < MAP_SIZE-200){      //increase avg
      if(DACoffset>=25) DACoffset -= 25;
      else DACoffset=0;
    }else{                //offset perfect
      thre = avg - TRIGGERLEVEL;
    }
    setDAC(DACoffset);

    //wait for analog signal to settle
    delay_ms(1);
  }
}

uint32_t measureOffset(){
  uint32_t avg=0;
    for(int i=0;i<ADC_BUF_LEN;i++){
      waitADC();
      avg+=ADC1->DR;
    }
  avg=avg/ADC_BUF_LEN;
  
  return avg;
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

void initADC()
{
  ADC_InitTypeDef       ADC_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable ADC3, DMA2 and GPIO clocks ****************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  ADC_Cmd(ADC1, DISABLE);
  
  ADC_DeInit();
   
  /* ADC3 Init ****************************************************************/
  ADC_StructInit(&ADC_InitStructure);
  /* Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits  */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_Init(ADC1, &ADC_InitStructure);
  
  /*NVIC_InitStructure.NVIC_IRQChannel = ADC1_COMP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);     */
  
  ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_15Cycles);
  

  ADC_Cmd(ADC1, ENABLE);

  ADC_SoftwareStartConv(ADC1);
}

void initIO(void){
    
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOC , ENABLE);
  
  GPIO_InitTypeDef      GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = OrangeLED;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GreenLED;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = StepUpPin;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
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
