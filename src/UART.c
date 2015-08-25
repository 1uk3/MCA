#include "UART.h"
#include "main.h"

DMA_InitTypeDef  DMA_InitStructure;

uint8_t TxBuffer[TX_BUFFER_SIZE];



extern volatile uint32_t trigger_lvl;

struct command{
	uint8_t cmd;
	uint8_t argc;
	uint8_t args[256];
	uint16_t rcnt;
}c;

static void USART_Config(void);
void setWE(void);
void setRE(void);
void sendCMD(uint8_t cmd);

void UART_Init(void)
{
    c.rcnt=0;
		USART_Config();
		
		//DMA_DeInit(DMA1_Stream1);
		//DMA_DeInit(USARTx_RX_DMA_CHANNEL);
		USART_DeInit(USARTx);

		USART_Config();
		
		//DMA_Cmd(USARTx_RX_DMA_CHANNEL,ENABLE);
		//USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE);
}

void parse()
{
	switch(c.cmd){
			case OFFSET:{
        UARTsendOffset(measureOffset());
				break;
			}
			case FLASH_LED:{
        DISOrangeLED();
        for(int i=0;i<10;i++){
        __asm("NOP");
        }
        ENOrangeLED();
				break;
			}
      case SENDCURVEEN:{
        sendCurveEnable =1;
        break;
      }
      case SENDCURVEDIS:{
        sendCurveEnable =0;
        break;
      }
      case TRIGGER_LVL:{
        thre=c.args[0] | c.args[1]<<8;
        break;
      }
      case GET_MAP:{
        UARTsendCurve(data, ADC_BUF_LEN, 0);
        break;
      }
			case EMPTY:{
				break;
			}
			default:{c.rcnt=0;break;}
		}
}


int UARTsendCMD(uint8_t cmd, uint8_t wait)
{
	while(!USART_GetFlagStatus(USARTx,USART_FLAG_TXE)){}
	USART_SendData(USARTx,cmd);
	while(wait && !USART_GetFlagStatus(USARTx,USART_FLAG_TC)){}
	
	return 0;
}

void UARTsend2Byte(uint16_t val){
  UARTsendCMD(val&0xFF,0);
  UARTsendCMD(val>>8,0);
}

void UARTsendSingle16(uint8_t type, uint16_t value){
  UARTsendCMD('\n',0);
  UARTsendCMD(type,0); 
  UARTsend2Byte(2);//2 arg bytes
  UARTsend2Byte(value);
}

void UARTsendOffset(uint16_t val){
  UARTsendSingle16(OFFSET,val);
}

void UARTsendPeak(uint16_t val){
  UARTsendSingle16(NEW_PEAK,val);
}

void UARTsendCurve(uint16_t buffer[], uint16_t len, uint16_t trigger_pos){
  UARTsendCMD('\n',1);
  UARTsendCMD(1,1);
  
  UARTsend2Byte(len*2+2);
  UARTsend2Byte(trigger_pos);
  for(int i=0;i<len;i++){
    UARTsend2Byte(buffer[i]);
  }
}

void USARTx_IRQHandler(void)
{
	
	if(USART_GetFlagStatus(USARTx,USARTx_FLAG_RXNE) == SET)
	{
		
		uint8_t in = USART_ReceiveData(USARTx);
		//c.args[c.rcnt]=in;
		if(c.rcnt==0)c.cmd=in;
		else if(c.rcnt==1)c.argc=in;
		else c.args[c.rcnt-2]=in;
		c.rcnt++;	
		
		if(c.rcnt == c.argc+2){
			parse();
			c.argc=0;
			c.rcnt=0;
		}
	}
}

/*void USARTx_DMA_IRQHandler(void)
{
	//TX
	if (DMA_GetFlagStatus(DMA1_Stream1,USARTx_TX_DMA_FLAG_TCIF)==SET){		
		
		DMA_ClearFlag(DMA1_Stream1, USARTx_TX_DMA_FLAG_TCIF);
		
		USART_ClearFlag(USARTx,USART_FLAG_TC);  
	}
}


void UART_SendBlock(uint8_t *buffer, uint32_t BufferSize)
{
	USART_ClearFlag(USARTx,USART_FLAG_TC);
	
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)buffer;
	DMA_InitStructure.DMA_BufferSize = BufferSize;
  DMA_Init(DMA1_Stream1,&DMA_InitStructure);
	

	DMA_Cmd(DMA1_Stream1,ENABLE);
	USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);
}*/



/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
static void USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

  /* Peripheral Clock Enable -------------------------------------------------*/
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
  /* Enable USART clock */
//USART_ClockInit(USARTx,);
  /* Enable the DMA clock */
  //RCC_AHBPeriphClockCmd(USARTx_DMAx_CLK, ENABLE);
  
  /* USARTx GPIO configuration -----------------------------------------------*/ 
  /* Connect USART pins to AF7 */
  GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);
  GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;
  GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;
  GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);
 
  /* USARTx configuration ----------------------------------------------------*/
  /* Enable the USART OverSampling by 8 */
  //USART_OverSampling8Cmd(USARTx, ENABLE); 
  
  
  USART_InitStructure.USART_BaudRate = 1000000;
  USART_InitStructure.USART_WordLength = USART_WordLength_9b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* When using Parity the word length must be configured to 9 bits */
  USART_InitStructure.USART_Parity = USART_Parity_Even;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USARTx, &USART_InitStructure);

  /* Configure DMA controller to manage USART TX and RX DMA request ----------*/ 
   
  /* Configure DMA Initialization Structure */
  
  /*DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  // Configure TX DMA 
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USARTx->DR)) ;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)TxBuffer;//(uint32_t)Buffer ;
	DMA_InitStructure.DMA_BufferSize = TX_BUFFER_SIZE;
  DMA_Init(DMA1_Stream1,&DMA_InitStructure);
	
	DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE);
	
	  // Enable the DMA Stream IRQ Channel 
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);   		   
	
  USART_ITConfig(USARTx,USART_IT_RXNE,ENABLE);*/
  
		  // Enable the IRQ 
  NVIC_InitStructure.NVIC_IRQChannel = USARTx_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
				 
  // Enable USART */
  USART_Cmd(USARTx, ENABLE);
}

