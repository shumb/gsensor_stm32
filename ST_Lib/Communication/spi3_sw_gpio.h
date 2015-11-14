/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
* File Name          : spi3_sw_gpio.h
* Author             : AMS Application Team
* Version            : $Revision: 1.0 $
* Date               : $Date: 2013/02/12 10:37:09 $
* Description        : Descriptor Header for spi3_sw_gpio file
* HISTORY:
* Date        | Modification                                | Author
* 12/02/2012  | Initial Revision                            | Fabio Tota

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

#ifndef __SPI3_MACHINE_H_INCLUDED
#define __SPI3_MACHINE_H_INCLUDED

#include "stm32f10x.h"


// Platform dependent definitions
#define SCL_MASK			GPIO_Pin_13
#define SCL_PORT			GPIOB                                                                                                                                                                                                                                                                                                  
#define SCL_CLK				RCC_APB2Periph_GPIOB
#define SCL_PORT_SOURCE		GPIO_PortSourceGPIOB
#define SCL_PIN_SOURCE		GPIO_PinSource13

#define SDIO_MASK			GPIO_Pin_15
#define SDIO_PORT			GPIOB
#define SDIO_CLK			RCC_APB2Periph_GPIOB
#define SDIO_PORT_SOURCE	GPIO_PortSourceGPIOB
#define SDIO_PIN_SOURCE		GPIO_PinSource15

#define CS_MASK				GPIO_Pin_6
#define CS_PORT				GPIOC
#define CS_CLK				RCC_APB2Periph_GPIOC
#define CS_PORT_SOURCE		GPIO_PortSourceGPIOC
#define CS_PIN_SOURCE		GPIO_PinSource6

extern uint8_t scl_state;
extern uint8_t SPI3_CS_FLAG;

typedef enum {
  WRITE_BYTE,
  READ_BYTE
} SPI3_SW_R_W;

typedef enum {
  SPI3_SW_MASTER,
  SPI3_SW_SLAVE
} SPI3_SW_MODE;

typedef enum {
  SPI3_BUS_STATE_IDLE,
  SPI3_BUS_STATE_BUSY,
  SPI3_BUS_STATE_START_0,
  SPI3_BUS_STATE_START_1,
  SPI3_BUS_STATE_TX_0,
  SPI3_BUS_STATE_TX_1,
  SPI3_BUS_STATE_RX_0,	
} SPI3_BUS_STATE;


typedef struct {
  SPI3_SW_MODE mode; // master-slave
  SPI3_BUS_STATE bus_state;
  
  uint8_t status;
  uint8_t address; // register address
  SPI3_SW_R_W rw_operation;  //read or write operation
  uint8_t *buffer;
  uint8_t length; // length of the data
  uint8_t byte_cntr; // index for buffer
  
  uint8_t byte;
  uint8_t byte_bit_cntr;
  
  uint8_t action;
} SPI3_STRUCT;

void SPI3_sw_Init(void);
void SPI3_sw_DeInit(void);
void InitTimer2SPI(void);

uint8_t SPI3_sw_Action(uint8_t addr, uint8_t *data, uint8_t length); //to initial set spi register 
void SPI3_sw_BitBanging(void);  //If use timer interrupt called from the interrupt with 2x SPI baud rate

u8 get_sdio(void);
u8 get_scl(void);
void set_sdio(void);
void set_scl(void);
void clr_sdio(void);
void clr_scl(void);
void toggle_scl(void);
void high_cs(void);
void low_cs(void);
void sdio_receiver(void);
void sdio_transmitter(void);
void cs_init(void);
void scl_receiver(void);
void scl_transmitter(void);

//read write function
u8 SPI3_sw_read_register(u8 register_add, u8 *buffer, u8 lengh);
u8 SPI3_sw_write_register(u8 register_add, u8 *buffer, u8 lengh);


#endif

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/