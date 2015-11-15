/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : spi_mems.h
* Author             : MSH Application Team
* Version            : V1.1
* Date               : 04/05/2010
* Description        : Descriptor Header for spi mems file
* HISTORY:
* Date			| Modification										| Author
* 04/05/2010	| Initial Revision									| Andrea Labombarda
* 28/02/2012	| SPI DeInit function added							| Andrea Labombarda
* 22/01/2013	| CS pin declaration corrected						| Andrea Labombarda

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
#ifndef __SPI_MEMS_H
#define __SPI_MEMS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "platform_config.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define USE_SPI_1
//#define USE_SPI_2

#ifdef USE_SPI_1
/* Defines for the SPI and GPIO pins used to drive the SPI MEMS */
#define SPI_MEMS                 SPI1
#define SPI_MEMS_CLK             RCC_APB2Periph_SPI1
#define SPI_MEMS_GPIO            GPIOA
#define SPI_MEMS_GPIO_CLK        RCC_AHBPeriph_GPIOA  
#define SPI_MEMS_PIN_SCK         GPIO_Pin_5
#define SPI_MEMS_PIN_MISO        GPIO_Pin_6
#define SPI_MEMS_PIN_MOSI        GPIO_Pin_7

/* DMA channel definition for SPI1 */
#define SPI_MEMS_DMA             DMA1                   // DMA1 used
#define SPI_MEMS_DMA_CLK         RCC_AHBPeriph_DMA1     // DMA1 clk source
#define SPI_MEMS_Rx_DMA_Channel  DMA1_Channel2          // SPI1_RX connected to channel 2
#define SPI_MEMS_Rx_DMA_FLAG     DMA1_FLAG_TC2          // TC2 flag used
#define SPI_MEMS_Tx_DMA_Channel  DMA1_Channel3          // SPI1_TX connected to channel 3
#define SPI_MEMS_Tx_DMA_FLAG     DMA1_FLAG_TC3          // TC3 flag used
#define SPI_MEMS_DR_Base         (SPI1_BASE+0x0C)       // SPI_DR address
/* End DMA channel definition for SPI1 */
#endif

#ifdef USE_SPI_2
#define SPI_MEMS                 SPI2
#define SPI_MEMS_CLK             RCC_APB1Periph_SPI2
#define SPI_MEMS_GPIO            GPIOB
#define SPI_MEMS_GPIO_CLK        RCC_APBPeriph_GPIOB  
#define SPI_MEMS_PIN_SCK         GPIO_Pin_13
#define SPI_MEMS_PIN_MISO        GPIO_Pin_14
#define SPI_MEMS_PIN_MOSI        GPIO_Pin_15

/* DMA channel definition for SPI2 */
#define SPI_MEMS_DMA             DMA1                   // DMA1 used
#define SPI_MEMS_DMA_CLK         RCC_AHBPeriph_DMA1     // DMA1 clk source
#define SPI_MEMS_Rx_DMA_Channel  DMA1_Channel4          // SPI1_RX connected to channel 4
#define SPI_MEMS_Rx_DMA_FLAG     DMA1_FLAG_TC4          // TC4 flag used
#define SPI_MEMS_Tx_DMA_Channel  DMA1_Channel5          // SPI1_TX connected to channel 5
#define SPI_MEMS_Tx_DMA_FLAG     DMA1_FLAG_TC5          // TC5 flag used
#define SPI_MEMS_DR_Base         (SPI2_BASE+0x0C)       // SPI_DR address
/* End DMA channel definition for SPI2 */
#endif

#ifdef USE_SPI_1
 #define SPI_MEMS_CS             GPIO_Pin_2
 #define SPI_MEMS_CS_GPIO        GPIOA
 #define SPI_MEMS_CS_GPIO_CLK    RCC_AHBPeriph_GPIOA
#endif
#ifdef USE_SPI_2
 #define SPI_MEMS_CS             CS_GPIO_PIN
 #define SPI_MEMS_CS_GPIO        CS_GPIO_PORT
 #define SPI_MEMS_CS_GPIO_CLK    CS_GPIO_CLK
#endif


/* Exported macro ------------------------------------------------------------*/
/* Select SPI MEMS: Chip Select pin low  */
static __inline void SPI_MEMS_CS_LOW(void)
{
  GPIO_ResetBits(SPI_MEMS_CS_GPIO, SPI_MEMS_CS);
}
/* Deselect SPI MEMS: Chip Select pin high */
static __inline void SPI_MEMS_CS_HIGH(void)
{
  GPIO_SetBits(SPI_MEMS_CS_GPIO, SPI_MEMS_CS);
}

/* Deselect SPI MEMS: Chip Select #2 pin low */
static __inline void SPI_MEMS_CS_2_LOW(void)
{
  GPIO_ResetBits(CS_2_GPIO_PORT, CS_2_GPIO_PIN);
}
/* Select SPI MEMS: Chip Select #2 pin high  */
static __inline void SPI_MEMS_CS_2_HIGH(void)
{
  GPIO_SetBits(CS_2_GPIO_PORT, CS_2_GPIO_PIN);
}

#define SPI_I2S_SendData		SPI_I2S_SendData16
#define SPI_I2S_ReceiveData	SPI_I2S_ReceiveData16

/* Exported functions --------------------------------------------------------*/
/*----- High layer function -----*/
void SPI_Mems_Init(void);
void SPI_Mems_DeInit(void);
uint8_t SPI_Mems_Read_Reg(uint8_t reg);
void SPI_Mems_Write_Reg(uint8_t regAddr, uint8_t data);
void SPI_Read_MultiData(uint8_t regAddr, uint8_t length, uint8_t* buffer);
void SPI_Read_MultiData_Raw(uint8_t regAddr, uint16_t length, uint8_t* buffer);
void SPI_Write_MultiData_Raw(uint8_t regAddr, uint16_t length, uint8_t* buffer);

/*----- 3-wire SPI High layer function -----*/
void SPI_3W_Mems_Init(uint8_t spi_use_cs2);
void SPI_3W_Mems_DeInit(void);
uint8_t SPI_3W_Mems_Read_Reg(uint8_t cs_num, uint8_t reg);
void SPI_3W_Mems_Write_Reg(uint8_t cs_num, uint8_t address, uint8_t data);
void SPI_3W_Read_MultiData(uint8_t cs_num, uint8_t regAddr, uint8_t length, uint8_t* buffer);

/*----- Low layer function -----*/
uint8_t SPI_Mems_SendByte(uint8_t byte);
uint8_t SPI_3W_Mems_SendByte(uint8_t byte);

#endif /* __SPI_MEMS_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
