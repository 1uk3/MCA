#include "stm32f4xx.h"

#ifndef __UARTHEADERDEFINE
#define __UARTHEADERDEFINE

void UART_Init(void);

void UART_SendBlock(uint8_t *buffer, uint32_t BufferSize);

int UARTsendCMD(uint8_t cmd, uint8_t wait);
void UARTsendOffset(uint16_t val);
void UARTsendPeak(uint16_t val);
inline void UARTsend2Byte(uint16_t val);

void UARTsendCurve(uint16_t buffer[], uint16_t len, uint16_t trigger_pos);

extern volatile uint32_t thre;
extern volatile uint8_t sendCurveEnable;
extern volatile uint16_t data[];

typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

typedef enum {EMPTY=0,GET_DATA=1,OFFSET=2,FLASH_LED=3,GET_PPM=4,TRIGGER_LVL=5, NEW_PEAK=6,SENDCURVEEN=7,SENDCURVEDIS=8, GET_MAP=9} Commands;

 #define TEST_DATA_SIZE 16
 #define TX_BUFFER_SIZE 16
 #define UART_PORT	GPIOA
 #define UART_WRE	GPIO_Pin_8
 #define UART_RDE	GPIO_Pin_11

 #define USARTx                           USART3
  #define USARTx_CLK                       RCC_APB2Periph_USART3
  #define USARTx_CLK_INIT                  RCC_APB2PeriphClockCmd
	//RCC_APB2PeriphClockCmd
  #define USARTx_IRQn                      USART3_IRQn
  #define USARTx_IRQHandler                USART3_IRQHandler
	#define USARTx_FLAG_RXNE									USART_FLAG_RXNE

  #define USARTx_TX_PIN                    GPIO_Pin_10               
  #define USARTx_TX_GPIO_PORT              GPIOB                       
  #define USARTx_TX_GPIO_CLK               RCC_AHB1Periph_GPIOB
  #define USARTx_TX_SOURCE                 GPIO_PinSource10
  #define USARTx_TX_AF                     GPIO_AF_USART3

  #define USARTx_RX_PIN                    GPIO_Pin_11             
  #define USARTx_RX_GPIO_PORT              GPIOB                   
  #define USARTx_RX_GPIO_CLK               RCC_AHB1Periph_GPIOB
  #define USARTx_RX_SOURCE                 GPIO_PinSource11
  #define USARTx_RX_AF                     GPIO_AF_USART3

  /* Definition for DMAx resources **********************************************/
  #define USARTx_DR_ADDRESS                ((uint32_t)USART1 + 0x04) 

  #define USARTx_DMA                       DMA1
  #define USARTx_DMAx_CLK                  RCC_AHB1Periph_DMA1
     
  #define USARTx_TX_DMA_CHANNEL            DMA1_Channel2
  #define USARTx_TX_DMA_FLAG_FEIF          DMA1_FLAG_FE2
  #define USARTx_TX_DMA_FLAG_DMEIF         DMA1_FLAG_DME2
  #define USARTx_TX_DMA_FLAG_TEIF          DMA1_FLAG_TE2
  #define USARTx_TX_DMA_FLAG_HTIF          DMA1_FLAG_HT2
  #define USARTx_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF2
              
  #define USARTx_RX_DMA_CHANNEL            DMA1_Channel3
  #define USARTx_RX_DMA_FLAG_FEIF          DMA1_FLAG_FE3
  #define USARTx_RX_DMA_FLAG_DMEIF         DMA1_FLAG_DME3
  #define USARTx_RX_DMA_FLAG_TEIF          DMA1_FLAG_TE3
  #define USARTx_RX_DMA_FLAG_HTIF          DMA1_FLAG_HT3
  #define USARTx_RX_DMA_FLAG_TCIF          DMA1_FLAG_TC3

  #define USARTx_DMA_TX_IRQn               DMA1_Channel2_3_IRQn
  #define USARTx_DMA_RX_IRQn               DMA1_Channel2_3_IRQn
  #define USARTx_DMA_IRQHandler         DMA1_Channel2_3_IRQHandler

#endif
