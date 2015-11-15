/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : spi_mems.c
* Author             : MSH Application Team
* Author             : andrea labombarda
* Version            : V1.1
* Date               : 04/05/2010
* Description        : This file provides a set of functions needed to manage the
*                     communication between SPI peripheral and SPI MEMS.
* HISTORY:
* Date			| Modification									| Author
* 04/05/2010	| Initial Revision								| Andrea Labombarda
* 16/06/2010	| header fixed									| Andrea Labombarda
* 28/02/2012	| SPI deInit Added								| Andrea Labombarda

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
#include "spi_mems.h"
#include "stm32f0xx_dma.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DUMMY_BYTE    0xA5

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t cr1 = 0;
uint16_t cr2 = 0;
uint32_t registers;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the peripherals used by the SPI Mems driver.
  * @param  None
  * @retval None
  */

/*******************************************************************************
* Function Name  : SPI_Mems_Init
* Description    : Initializes the peripherals used by the SPI Mems driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_Mems_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

#ifdef USE_SPI_1
  /* Enable SPI and GPIO clocks */
  RCC_APB2PeriphClockCmd(SPI_MEMS_CLK | SPI_MEMS_GPIO_CLK | SPI_MEMS_CS_GPIO_CLK, ENABLE);
#endif
 
#ifdef USE_SPI_2
  RCC_APB1PeriphClockCmd(SPI_MEMS_CLK , ENABLE);
  RCC_APB2PeriphClockCmd(SPI_MEMS_GPIO_CLK | SPI_MEMS_CS_GPIO_CLK, ENABLE);    
#endif
  
  /* Configure SPI pins: SCK, MISO and MOSI */
  GPIO_InitStructure.GPIO_Pin = SPI_MEMS_PIN_SCK | SPI_MEMS_PIN_MISO | SPI_MEMS_PIN_MOSI;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(SPI_MEMS_GPIO, &GPIO_InitStructure);

  /* Configure I/O for MEMS Chip select */
  GPIO_InitStructure.GPIO_Pin = SPI_MEMS_CS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(SPI_MEMS_CS_GPIO, &GPIO_InitStructure);

  /* Deselect Mems Sensor: Chip Select high */
  SPI_MEMS_CS_HIGH();  

  /* SPI configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  //SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI_MEMS, &SPI_InitStructure);

  /* Enable the SPI  */
  SPI_Cmd(SPI_MEMS, ENABLE);
}

/*******************************************************************************
* Function Name  : SPI_Mems_DeInit
* Description    : DeInitializes the peripherals used by the SPI Mems driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_Mems_DeInit(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin = SPI_MEMS_PIN_SCK | SPI_MEMS_PIN_MISO | SPI_MEMS_PIN_MOSI;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_Init(SPI_MEMS_GPIO, &GPIO_InitStructure);  
  
  GPIO_InitStructure.GPIO_Pin = SPI_MEMS_CS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_Init(SPI_MEMS_CS_GPIO, &GPIO_InitStructure);
  
}

/*******************************************************************************
* Function Name  : SPI_Mems_Read_Reg
* Description    : Reads Mems Register
* Input          : Register 
* Output         : None
* Return         : Register Content
*******************************************************************************/
uint8_t SPI_Mems_Read_Reg(uint8_t reg) {
  uint8_t Temp;

  reg += 0x80; //reading procedure has to set the most significant bit
  // Select Mems Sensor: Chip Select low 
  SPI_MEMS_CS_LOW();

  // Send Register Address
  SPI_Mems_SendByte(reg);
  /* Read a byte from the MEMS Sensor */
  Temp = SPI_Mems_SendByte(DUMMY_BYTE);  

  /* Deselect Mems Sensor: Chip Select high */
  SPI_MEMS_CS_HIGH();  

  return Temp;
}

/*******************************************************************************
* Function Name  : SPI_Mems_Write_Reg
* Description    : Write Data in Mems Register
* Input          : Register, data
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_Mems_Write_Reg(uint8_t regAddr, uint8_t data) {

  // Select Mems Sensor: Chip Select low 
  SPI_MEMS_CS_LOW();

  // Send Register Address
  SPI_Mems_SendByte(regAddr);
  /* Write a byte into the MEMS Sensor Register*/
  SPI_Mems_SendByte(data);  

  /* Deselect Mems Sensor: Chip Select high */
  SPI_MEMS_CS_HIGH();  

}


/*******************************************************************************
* Function Name  : SPI_Read_MultiData
* Description    : Reads Mems Registers in auto increment mode
* Input          : Start Address, number of register to read, Buffer
* Output         : None
* Return         : Register Content
*******************************************************************************/
void SPI_Read_MultiData(uint8_t regAddr, uint8_t length, uint8_t* buffer) {       
  uint8_t i = 0;
  
  regAddr += 0xc0; //reading procedure has to set the most significant bit
  // Select Mems Sensor: Chip Select low 
  SPI_MEMS_CS_LOW();

  // Send Register Address
  SPI_Mems_SendByte(regAddr);
  
  while(i < length){
    /* Read a byte from the MEMS Sensor */
    buffer[i] = SPI_Mems_SendByte(DUMMY_BYTE);  
    i++;
  }
  
  /* Deselect Mems Sensor: Chip Select high */
  SPI_MEMS_CS_HIGH(); 
  
}

/*******************************************************************************
* Function Name  : SPI_Read_MultiData_Raw
* Description    : Multiple read in raw mode (addr and data are sent 'as it is')
* Input          : Start Address, number of register to read, Buffer
* Output         : None
* Return         : Register Content
*******************************************************************************/
void SPI_Read_MultiData_Raw(uint8_t regAddr, uint16_t length, uint8_t* buffer)
{
  uint16_t i = 0;

  // Select Mems Sensor: Chip Select low
  SPI_MEMS_CS_LOW();

  // Send Register Address
  SPI_Mems_SendByte(regAddr);

  while(i < length){
    /* Read a byte from the MEMS Sensor */
    buffer[i] = SPI_Mems_SendByte(DUMMY_BYTE);
    i++;
  }

  /* Deselect Mems Sensor: Chip Select high */
  SPI_MEMS_CS_HIGH();
}

/*******************************************************************************
* Function Name  : SPI_Write_MultiData_Raw
* Description    : Multiple read in raw mode (addr and data are sent 'as it is')
* Input          : Start Address, number of register to read, Buffer
* Output         : None
* Return         : Register Content
*******************************************************************************/
void SPI_Write_MultiData_Raw(uint8_t regAddr, uint16_t length, uint8_t* buffer)
{
  uint16_t i = 0;

  // Select Mems Sensor: Chip Select low
  SPI_MEMS_CS_LOW();

  // Send Register Address
  SPI_Mems_SendByte(regAddr);

  while(i < length){
    /* Read a byte from the MEMS Sensor */
    SPI_Mems_SendByte(buffer[i++]);
  }

  /* Deselect Mems Sensor: Chip Select high */
  SPI_MEMS_CS_HIGH();
}

/*******************************************************************************
* Function Name  : SPI_Mems_SendByte
* Description    : Sends a byte through the SPI interface and return the byte
*                  received from the SPI bus.
* Input          : byte : byte to send.
* Output         : None
* Return         : The value of the received byte.
*******************************************************************************/
uint8_t SPI_Mems_SendByte(uint8_t byte) {
  
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI_MEMS, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(SPI_MEMS, byte);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(SPI_MEMS, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI_MEMS);
}

/*******************************************************************************
********************************************************************************
*              3-WIRE SPI COMMUNICATION                                        *
********************************************************************************
********************************************************************************/
#ifdef SPI_3W_CONFIG

#define RX_BUFFER_SIZE  32      // Rx buffer size for DMA transfer
#define SPI_TIME_DELAY  1       // Delay required before SPI disable in 3-wire mode

uint8_t SPI_RX_Buffer[RX_BUFFER_SIZE];  // Rx buffer used for DMA transfer

/*******************************************************************************
* Function Name  : SPI_3W_Mems_Init
* Description    : Initializes the peripherals used by the 3 wires SPI Mems driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_3W_Mems_Init(uint8_t spi_use_cs2)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  DMA_InitTypeDef  DMA_InitStructure;

#ifdef USE_SPI_1
  /* Enable SPI and GPIO clocks */
  RCC_APB2PeriphClockCmd(SPI_MEMS_CLK | SPI_MEMS_GPIO_CLK | SPI_MEMS_CS_GPIO_CLK, ENABLE);
  if (spi_use_cs2)
    RCC_APB2PeriphClockCmd(CS_2_GPIO_CLK, ENABLE);
#endif
 
#ifdef USE_SPI_2
  RCC_APB1PeriphClockCmd(SPI_MEMS_CLK , ENABLE);
  RCC_APB2PeriphClockCmd(SPI_MEMS_GPIO_CLK | SPI_MEMS_CS_GPIO_CLK, ENABLE); 
  RCC_AHBPeriphClockCmd(SPI_MEMS_DMA_CLK, ENABLE);
  if (spi_use_cs2)
    RCC_APB2PeriphClockCmd(CS_2_GPIO_CLK, ENABLE);
#endif
  
  /* SPI_MASTER_Rx_DMA_Channel configuration ---------------------------------------------*/
  DMA_DeInit(SPI_MEMS_Rx_DMA_Channel);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SPI_MEMS_DR_Base;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)SPI_RX_Buffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(SPI_MEMS_Rx_DMA_Channel, &DMA_InitStructure);
  
  /* Configure SPI pins: SCK, MISO and MOSI */
  GPIO_InitStructure.GPIO_Pin = SPI_MEMS_PIN_SCK | SPI_MEMS_PIN_MOSI;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(SPI_MEMS_GPIO, &GPIO_InitStructure);

  /* Configure I/O for MEMS Chip select */
  GPIO_InitStructure.GPIO_Pin = SPI_MEMS_CS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(SPI_MEMS_CS_GPIO, &GPIO_InitStructure);

  /* Configure the MEMS Chip select #2 pin */
  if (spi_use_cs2) {
    GPIO_InitStructure.GPIO_Pin = CS_2_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CS_2_GPIO_PORT, &GPIO_InitStructure);
  }

  /* Deselect Mems Sensor: Chip Select high */
  SPI_MEMS_CS_HIGH();  
  if (spi_use_cs2)
    SPI_MEMS_CS_2_HIGH();

  /* SPI configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;  
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI_MEMS, &SPI_InitStructure);

  /* Enable the SPI  */
  //SPI_Cmd(SPI_MEMS, ENABLE);
}


/*******************************************************************************
* Function Name  : SPI_3W_Mems_Write_Reg
* Description    : Sends a byte through the SPI interface and return the byte
*                  received from the SPI bus.
* Input          : byte : byte to send.
* Output         : None
* Return         : The value of the received byte.
*******************************************************************************/
void SPI_3W_Mems_Write_Reg(uint8_t cs_num, uint8_t address, uint8_t data)
{
  
  // Enable SPI slave - set SS to LOW
  if (cs_num == 0)
    SPI_MEMS_CS_LOW();
  else
    SPI_MEMS_CS_2_LOW();
   
  // Enable SPI
  SPI_Cmd(SPI_MEMS, ENABLE);     
    
  // Loop while DR register in not empty - previous data transmited
  while (SPI_I2S_GetFlagStatus(SPI_MEMS, SPI_I2S_FLAG_TXE) == RESET);

  // Send byte through the SPI1 peripheral
  SPI_I2S_SendData(SPI_MEMS, address);
  
  // Loop while DR register in not emplty
  while (SPI_I2S_GetFlagStatus(SPI_MEMS, SPI_I2S_FLAG_TXE) == RESET);
  
  // Send byte through the SPI peripheral
  SPI_I2S_SendData(SPI_MEMS, data);
  
  // Loop while DR register in not empty and the SPI transfer is finished 
  while (SPI_I2S_GetFlagStatus(SPI_MEMS, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI_MEMS, SPI_I2S_FLAG_BSY) == SET);  
 
  // Disable SPI as the transfer is finished
  SPI_Cmd(SPI_MEMS, DISABLE);
  
  // Deselect the SPI slave
  if (cs_num == 0)
    SPI_MEMS_CS_HIGH();
  else
    SPI_MEMS_CS_2_HIGH();
}

/*******************************************************************************
* Function Name  : SPI_3W_Mems_Read_Reg
* Description    : Reads Mems Register
* Input          : Register 
* Output         : None
* Return         : Register Content
*******************************************************************************/
uint8_t SPI_3W_Mems_Read_Reg(uint8_t cs_num, uint8_t reg)
{
  uint8_t Temp;
  uint32_t pom;
    
  //reading procedure has to set the most significant bit
  reg |= 0x80;
  
  // Configure MOSI line as master output
  SPI_BiDirectionalLineConfig(SPI_MEMS, SPI_Direction_Tx); 
  
  // Enable the SPI slave device - set SS to "0"
  if (cs_num == 0)
    SPI_MEMS_CS_LOW();
  else
    SPI_MEMS_CS_2_LOW();
  
  // Enable SPI
  SPI_Cmd(SPI_MEMS, ENABLE);    
    
  // Send byte through the SPI peripheral
  SPI_I2S_SendData(SPI_MEMS, reg);
  
  // Loop while DR register in not empty and the SPI transfer is finished 
  while (SPI_I2S_GetFlagStatus(SPI_MEMS, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI_MEMS, SPI_I2S_FLAG_BSY) == SET);  
  
  // Disable SPI as the transfer is finished
  SPI_Cmd(SPI_MEMS, DISABLE);
   
  // Change the MOSI line direction to slave input
  SPI_BiDirectionalLineConfig(SPI_MEMS, SPI_Direction_Rx);  
  
  // Disable interupts during this critical phase of communication
  __disable_irq();
  
  // Enable SPI for reception - CLK generation starts
  SPI_Cmd(SPI_MEMS, ENABLE);
  
  // Wait at least one SPI CLK cycle
  for(pom=0; pom < SPI_TIME_DELAY; pom++);
  
  // Disable SPI
  SPI_Cmd(SPI_MEMS, DISABLE);
  
  // Enable interupts - critical phase of communication passed
  __enable_irq();
  
  // Wait till the data is received
  while (SPI_I2S_GetFlagStatus(SPI_MEMS, SPI_I2S_FLAG_RXNE) == RESET);
  
  // Save received data
  Temp = SPI_I2S_ReceiveData(SPI_MEMS); 
  
  // Reconfigure the MOSI line as master output
  SPI_BiDirectionalLineConfig(SPI_MEMS, SPI_Direction_Tx);
  
  // Deselect the SPI slave
  if (cs_num == 0)
    SPI_MEMS_CS_HIGH();
  else
    SPI_MEMS_CS_2_HIGH();
 
  return Temp;
}

/*******************************************************************************
* Function Name  : SPI_3W_Read_multiData
* Description    : Reads Mems Registers in auto increment mode. Functiona read 
*                  (length-1) bytes using the DMA. Then the SPI is disabled and
*                  the last byte is read using RXNE flag pooling.
* Input          : Start Address, number of register to read, Buffer
* Output         : None
* Return         : Register Content
*******************************************************************************/
void SPI_3W_Read_MultiData(uint8_t cs_num, uint8_t regAddr, uint8_t length, uint8_t* buffer)
{  
  uint8_t i = 0;
//  uint8_t pom=0;

  /* reading procedure has to set two most significant bits
   * Bit 8 is set - reading from slave
   * Bit 7 is not set - register address autoincrement should be enebled by caller
   *                    if device require it.
   */
  regAddr |= 0x80;
  
  /* Configure the DMA transfer of (length-1) bytes */
  DMA_SetCurrDataCounter(SPI_MEMS_Rx_DMA_Channel,length-1);
  
  /* Enable SPI_SLAVE Rx request */
  SPI_I2S_DMACmd(SPI_MEMS, SPI_I2S_DMAReq_Rx, ENABLE);
    
  // Configure MOSI line as master output
  SPI_BiDirectionalLineConfig(SPI_MEMS, SPI_Direction_Tx); 
  
  // Enable the SPI slave device - set SS to "0"
  if (cs_num == 0)
    SPI_MEMS_CS_LOW();
  else
    SPI_MEMS_CS_2_LOW();
  
  // Enable SPI
  SPI_Cmd(SPI_MEMS, ENABLE);    
    
  // Send byte through the SPI peripheral
  SPI_I2S_SendData(SPI_MEMS, regAddr);
  
  // Loop while DR register in not empty and the SPI transfer is finished 
  while (SPI_I2S_GetFlagStatus(SPI_MEMS, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI_MEMS, SPI_I2S_FLAG_BSY) == SET);
  
  // Disable SPI to change the data flow direction
  SPI_Cmd(SPI_MEMS, DISABLE);
   
  // Change the MOSI line direction to slave input
  SPI_BiDirectionalLineConfig(SPI_MEMS, SPI_Direction_Rx);
  
  // Read Rx data if awailable to have the data buffer empty
  SPI_I2S_ReceiveData(SPI_MEMS);
  
  // Clear data transfered flag
  DMA_ClearFlag(SPI_MEMS_Rx_DMA_FLAG);
    
  /* Enable DMA1 Channel */
  DMA_Cmd(SPI_MEMS_Rx_DMA_Channel, ENABLE);
    
  // Enable SPI and disable interrupts till the transfer is finished
  __disable_irq();
  SPI_Cmd(SPI_MEMS, ENABLE);
  
  /* Wait till the DMA1 RX transfer is completed */
  while (!((SPI_MEMS_Rx_DMA_FLAG)&(DMA1->ISR)));
  
  // Disable SPI and enable interrupts again
  SPI_Cmd(SPI_MEMS, DISABLE);
  __enable_irq();
  
  /* Disable DMA for SPI_SLAVE Rx request */
  DMA_Cmd(SPI_MEMS_Rx_DMA_Channel, DISABLE);
  
  // wait till last byte transfered
  while (SPI_I2S_GetFlagStatus(SPI_MEMS, SPI_I2S_FLAG_RXNE) == RESET);
   
  // Copy received data to data buffer
  for(i=0;i<length-1;i++)
    buffer[i] = SPI_RX_Buffer[i];

  // Save last byte - received separately
  buffer[i] = SPI_I2S_ReceiveData(SPI_MEMS);   
  
  // Reconfigure the MOSI line as master output
  SPI_BiDirectionalLineConfig(SPI_MEMS, SPI_Direction_Tx);
  
  /* Deselect Mems Sensor: Chip Select high */
  if (cs_num == 0)
    SPI_MEMS_CS_HIGH();
  else
    SPI_MEMS_CS_2_HIGH();
}
#endif
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
