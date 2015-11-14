/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : communication_wrapper.c
* Author             : MSH Application Team
* Author             : andrea labombarda
* Version            : V1.0
* Date               : 19/01/2012
* Description        : This file provides a set of functions needed to wrap the communication protocol 
*                     chosen
* HISTORY:
* Date			| Modification                              | Author
* 19/01/2012    | Initial Revision                          | Andrea Labombarda

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

/* Includes ------------------------------------------------------------------*/
#include "i2c_mems.h"

#include "platform_config.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t busSelected = SPI;
/* Private function prototypes -----------------------------------------------*/
void CommInit(uint8_t bus);
uint8_t CommRead(uint8_t* pBuffer, uint8_t deviceAddr, uint8_t readAddr, uint16_t numByteToRead);
uint8_t CommWrite(uint8_t* pBuffer, uint8_t deviceAddress, uint8_t WriteAddr);
/* Private functions ---------------------------------------------------------*/

void CommInit(uint8_t bus) {
  busSelected = bus;
  
  switch(bus) {
  case SPI:
	SPI_Mems_Init();
	break;
  case I2C:
	I2C_MEMS_Init();
	break;
  default:
	  
  }
}


uint8_t CommRead(uint8_t* pBuffer, uint8_t deviceAddr, uint8_t readAddr, uint16_t numByteToRead) {
  
  switch(busSelected):
  return 1;
}

uint8_t CommWrite(uint8_t* pBuffer, uint8_t deviceAddress, uint8_t WriteAddr) {
  
  return 1;
}



