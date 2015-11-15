/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : platform_config.h
* Author             : MSH Application Team
* Revision           : $Revision: 1.4 $
* Date               : $Date: 2011/10/04 08:34:46 $
* Description        : Descriptor Header for platform config
* HISTORY:
* Date        | Modification                                | Author
* 04/05/2010  | Initial Revision                            | Andrea Labombarda
* 20/10/2010  | FS pin support                              | Andrea Labombarda
* 28/04/2011  | CS & SA0 pin support                        | Andrea Labombarda
* 04/10/2011  | New Interrupts Definitions                  | Andrea Labombarda

********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

/* Includes ------------------------------------------------------------------*/
//#include "stm32f10x.h"
#include "stm32f0xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/



/*  #define USB_DISCONNECT                      GPIOD  
  #define USB_DISCONNECT_PIN                  GPIO_Pin_9
  #define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOD
  #define EVAL_COM1_IRQn                      USART1_IRQn*/
  #define USB_DISCONNECT                      GPIOB  
  #define USB_DISCONNECT_PIN                  GPIO_Pin_12
  #define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOB
  #define EVAL_COM1_IRQn                      USART1_IRQn
/*  #define USB_DISCONNECT                      GPIOA  
  #define USB_DISCONNECT_PIN                  GPIO_Pin_10
  #define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOB
  #define EVAL_COM1_IRQn                      USART1_IRQn*/


#define COMn                        2

/**
 * @brief Definition for COM port1, connected to USART1
 */ 
/*
#define EVAL_COM1                   USART1
#define EVAL_COM1_GPIO              GPIOA
#define EVAL_COM1_CLK               RCC_APB2Periph_USART1
//#define EVAL_COM1_GPIO_CLK          RCC_APB2Periph_GPIOA
#define EVAL_COM1_RxPin             GPIO_Pin_10
#define EVAL_COM1_TxPin             GPIO_Pin_9
*/

/**
 * @brief Definition for COM port2, connected to USART2 (USART2 pins remapped on GPIOD)
 */ 

/*
#define EVAL_COM2                   USART2
#define EVAL_COM2_GPIO              GPIOD
#define EVAL_COM2_CLK               RCC_APB1Periph_USART2
#define EVAL_COM2_GPIO_CLK          RCC_APB2Periph_GPIOD
#define EVAL_COM2_RxPin             GPIO_Pin_6
#define EVAL_COM2_TxPin             GPIO_Pin_5

*/


/**
 * @brief Definition for COM port1, connected to USART1
 */ 
#define EVAL_COM1                        USART1
#define EVAL_COM1_CLK                    RCC_APB2Periph_USART1
#define EVAL_COM1_TX_PIN                 GPIO_Pin_9
#define EVAL_COM1_TX_GPIO_PORT           GPIOA
//#define EVAL_COM1_TX_GPIO_CLK            RCC_APB2Periph_GPIOA
#define EVAL_COM1_RX_PIN                 GPIO_Pin_10
#define EVAL_COM1_RX_GPIO_PORT           GPIOA
//#define EVAL_COM1_RX_GPIO_CLK            RCC_APB2Periph_GPIOA
#define EVAL_COM1_IRQn                   USART1_IRQn

/**
 * @brief Definition for COM port2, connected to USART2 (USART2 pins remapped on GPIOD)
 */ 
#define EVAL_COM2                        USART2
#define EVAL_COM2_CLK                    RCC_APB1Periph_USART2
#define EVAL_COM2_TX_PIN                 GPIO_Pin_5
#define EVAL_COM2_TX_GPIO_PORT           GPIOD
#define EVAL_COM2_TX_GPIO_CLK            RCC_APB2Periph_GPIOD
#define EVAL_COM2_RX_PIN                 GPIO_Pin_6
#define EVAL_COM2_RX_GPIO_PORT           GPIOD
#define EVAL_COM2_RX_GPIO_CLK            RCC_APB2Periph_GPIOD
#define EVAL_COM2_IRQn                   USART2_IRQn

/**
 * @brief INT2 line
 */
#define INT2_PORT                   GPIOC
#define INT2_CLK                    RCC_APB2Periph_GPIOC
#define INT2_PIN                    GPIO_Pin_2
#define INT2_EXTI_LINE              EXTI_Line2
#define INT2_PORT_SOURCE            GPIO_PortSourceGPIOC
#define INT2_PIN_SOURCE             GPIO_PinSource2
#define INT2_IRQn                   EXTI2_IRQn
   
/**
 * @brief INT1 line
 */
#define INT1_PORT                   GPIOC
#define INT1_CLK                    RCC_APB2Periph_GPIOC
#define INT1_PIN                    GPIO_Pin_3
#define INT1_EXTI_LINE              EXTI_Line3
#define INT1_PORT_SOURCE            GPIO_PortSourceGPIOC
#define INT1_PIN_SOURCE             GPIO_PinSource3
#define INT1_IRQn                   EXTI3_IRQn 

/**
 * @brief INT3 line
 */
#define INT3_PORT                   GPIOC
#define INT3_CLK                    RCC_APB2Periph_GPIOC
#define INT3_PIN                    GPIO_Pin_13
#define INT3_EXTI_LINE              EXTI_Line13
#define INT3_PORT_SOURCE            GPIO_PortSourceGPIOC
#define INT3_PIN_SOURCE             GPIO_PinSource13
#define INT3_IRQn                   EXTI15_10_IRQn 

/**
 * @brief INT4 line
 */
#define INT4_PORT                   GPIOC
#define INT4_CLK                    RCC_APB2Periph_GPIOC
#define INT4_PIN                    GPIO_Pin_12
#define INT4_EXTI_LINE              EXTI_Line12
#define INT4_PORT_SOURCE            GPIO_PortSourceGPIOC
#define INT4_PIN_SOURCE             GPIO_PinSource12
#define INT4_IRQn                   EXTI15_10_IRQn 



/** @addtogroup STM3210B_EVAL_BUTTON
  * @{
  */  
//#define BUTTONn                     2

#define SW2_BUTTON_PORT             GPIOB
#define SW2_BUTTON_CLK              RCC_APB2Periph_GPIOB
#define SW2_BUTTON_PIN              GPIO_Pin_9
#define SW2_BUTTON_EXTI_LINE        EXTI_Line9
#define SW2_BUTTON_PORT_SOURCE      GPIO_PortSourceGPIOB
#define SW2_BUTTON_PIN_SOURCE       GPIO_PinSource9
#define SW2_BUTTON_IRQn             EXTI9_5_IRQn

#define SW1_BUTTON_PORT             GPIOB
#define SW1_BUTTON_CLK              RCC_APB2Periph_GPIOB
#define SW1_BUTTON_PIN              GPIO_Pin_8
#define SW1_BUTTON_EXTI_LINE        EXTI_Line8
#define SW1_BUTTON_PORT_SOURCE      GPIO_PortSourceGPIOB
#define SW1_BUTTON_PIN_SOURCE       GPIO_PinSource8
#define SW1_BUTTON_IRQn             EXTI9_5_IRQn

#define LED1_GPIO_PORT              GPIOC
#define LED1_GPIO_CLK               RCC_APB2Periph_GPIOC  
#define LED1_GPIO_PIN               GPIO_Pin_10
  
#define LED2_GPIO_PORT              GPIOC
#define LED2_GPIO_CLK               RCC_APB2Periph_GPIOC  
#define LED2_GPIO_PIN               GPIO_Pin_11   
 /*  
#define LED3_GPIO_PORT              GPIOB
#define LED3_GPIO_CLK               RCC_APB2Periph_GPIOB
#define LED3_GPIO_PIN               GPIO_Pin_5
*/
#define PD_GPIO_PORT                GPIOD
#define PD_GPIO_CLK                 RCC_APB2Periph_GPIOD
#define PD_GPIO_PIN                 GPIO_Pin_2

#define ST_GPIO_PORT                GPIOC
#define ST_GPIO_CLK                 RCC_APB2Periph_GPIOC
#define ST_GPIO_PIN                 GPIO_Pin_12

#define HP_GPIO_PORT                GPIOA
//#define HP_GPIO_CLK                 RCC_APB2Periph_GPIOA
#define HP_GPIO_PIN                 GPIO_Pin_8

#define FS_GPIO_PORT                GPIOC
#define FS_GPIO_CLK                 RCC_APB2Periph_GPIOC
#define FS_GPIO_PIN                 GPIO_Pin_13

#define CS_GPIO_PORT                GPIOC
#define CS_GPIO_CLK                 RCC_APB2Periph_GPIOC
#define CS_GPIO_PIN                 GPIO_Pin_6

#define CS_2_GPIO_PORT              GPIOD
#define CS_2_GPIO_CLK               RCC_APB2Periph_GPIOD
#define CS_2_GPIO_PIN               GPIO_Pin_2

#define SA0_GPIO_PORT               GPIOB
#define SA0_GPIO_CLK                RCC_APB2Periph_GPIOB
#define SA0_GPIO_PIN                GPIO_Pin_14

#define BT_RESET_GPIO_PORT          GPIOB
#define BT_RESET_GPIO_CLK           RCC_APB2Periph_GPIOB
#define BT_RESET_GPIO_PIN           GPIO_Pin_11

#define JP9_GPIO_PORT				GPIOB
#define JP9_GPIO_CLK				RCC_APB2Periph_GPIOB
#define JP9_GPIO_PIN				GPIO_Pin_2


#define OUT6                        ADC_Channel_15
#define OUT1                        ADC_Channel_8
#define O1                          ADC_Channel_4
#define OUT4                        ADC_Channel_3
#define VREF                        ADC_Channel_2
#define OUT5                        ADC_Channel_1
#define OUT3                        ADC_Channel_0
#define O3                          ADC_Channel_10
#define OUT2                        ADC_Channel_9
#define O2                          ADC_Channel_14

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __PLATFORM_CONFIG_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
