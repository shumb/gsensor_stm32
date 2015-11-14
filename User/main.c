#include "ALL_Includes.h"//包含所需的头文件
#include <stdio.h>
#include <string.h>
#include "stm320518_eval.h"


void ALL_Config(void);

u16 cnt=0;

//#define CONFIG_SOFT_DEBUG

#ifdef CONFIG_SOFT_DEBUG
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))

#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000
struct __FILE { int handle; /* Add whatever needed */ };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) {
  if (DEMCR & TRCENA) {
    while (ITM_Port32(0) == 0);
    ITM_Port8(0) = ch;
  }
  return(ch);
}

#else /* CONFIG_SOFT_DEBUG */

USART_InitTypeDef USART_InitStructure;  
   
/* Private function prototypes -----------------------------------------------*/  
#ifdef __GNUC__  
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf 
     set to 'Yes') calls __io_putchar() */  
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)  
#else  
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)  
#endif /* __GNUC__ */ 

PUTCHAR_PROTOTYPE  
{  
  /* Place your implementation of fputc here */  
  /* e.g. write a character to the USART */  
  USART_SendData(EVAL_COM1, (uint8_t) ch);  
  
  /* Loop until the end of transmission */  
  while(USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET)
  {
  }

  return ch;  
}
#endif /* CONFIG_SOFT_DEBUG */


void USART_Configuration(void)//??.?.??..?
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE );

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;//?????.??.?
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//?????.?
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//????..?
    USART_InitStructure.USART_Parity = USART_Parity_No;//????..?
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//????..?
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//?????????
    USART_Init(USART1, &USART_InitStructure); //?.?????.?

    USART_Cmd(USART1, ENABLE);//??.??.1
}

int main(void)
{	
#if 1
  /* USARTx configured as follow: 
        - BaudRate = 115200 baud   
        - Word Length = 8 Bits 
        - One Stop Bit 
        - No parity 
        - Hardware flow control disabled (RTS and CTS signals) 
        - Receive and transmit enabled 
  */  
  USART_InitStructure.USART_BaudRate = 115200;  
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
  USART_InitStructure.USART_StopBits = USART_StopBits_1;  
  USART_InitStructure.USART_Parity = USART_Parity_No;  
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  
  
  STM_EVAL_COMInit(COM1, &USART_InitStructure); 
#else
	USART_Configuration();
#endif
	
	//ALL_Config();
		
	while(1)
	{
//			USART_SendData(USART1,'a');
//				delay_ms(1);
//				USART_SendData(USART1,'5');
//				delay_ms(1);
				printf("start: %d\n\r",cnt++);
//			LED_ON();
//			delay_ms(500);
//			LED_OFF();
//			delay_ms(500);
	}
}

/************************
函数功能：总初始化
输入参数：无
输出参数：无
备    注：无
************************/
void ALL_Config(void)
{
    Delay_Init(48);
		LED_Init();
}






