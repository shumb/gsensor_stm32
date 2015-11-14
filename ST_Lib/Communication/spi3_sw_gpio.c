/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
* File Name          : spi3_sw_gpio.c
* Author             : AMS Application Team
* Version            : $Revision: 1.0 $
* Date               : $Date: 2013/02/12 10:37:09 $
* Description        : This file provides a set of functions needed to use SPI 3 wire GPIO software
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

/* Includes ------------------------------------------------------------------*/
#include "spi3_sw_gpio.h"

/*------global variable definition-------*/
uint8_t scl_state = 0; 		
uint8_t SPI3_CS_FLAG = 1;

static SPI3_STRUCT spi3;


/*******************************************************************************
* Function Name  : SPI3_sw_DeInit
* Description    : Disable designed SPI3wire pins 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI3_sw_DeInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  //deinit cs
  RCC_APB2PeriphClockCmd(CS_CLK, ENABLE);   
  GPIO_InitStructure.GPIO_Pin = CS_MASK;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(CS_PORT, &GPIO_InitStructure);
  
  //deinit SCL
  RCC_APB2PeriphClockCmd(SCL_CLK, ENABLE);   
  GPIO_InitStructure.GPIO_Pin = SCL_MASK;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(SCL_PORT, &GPIO_InitStructure);
  
  //deinit SDIO
  RCC_APB2PeriphClockCmd(SDIO_CLK, ENABLE);   
  GPIO_InitStructure.GPIO_Pin = SDIO_MASK;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(SDIO_PORT, &GPIO_InitStructure);  
  
}


/*******************************************************************************
* Function Name  : SPI3_sw_Init
* Description    : Enable designed SPI3wire pins and initializing register values 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI3_sw_Init(void)
{
  spi3.mode = SPI3_SW_MASTER;
  spi3.bus_state = SPI3_BUS_STATE_IDLE;
  spi3.status = SPI3_BUS_STATE_IDLE;
  spi3.address = 0x00;
  spi3.buffer = (u8*)0;
  spi3.length = 1;
  spi3.byte_cntr = 0;
  spi3.byte = 0;
  spi3.byte_bit_cntr = 8;
  if (spi3.mode==SPI3_SW_MASTER){
    cs_init();
    scl_transmitter();
    sdio_transmitter();	
    set_sdio();
    set_scl();
  }
  //init timer for interrupt
  //InitTimer2SPI();    uncomment if use timer interrupt
}


/*******************************************************************************
* Function Name  : InitTimer2SPI
* Description    : example to how sets interrupt timer fot SPI bitbanging function called
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void InitTimer2SPI(void) {  
  NVIC_InitTypeDef NVIC_InitStructure;
  //Set Timer2 Clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);   
  //Timer 2 configuration
  TIM_DeInit(TIM2);
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  
  //Desired Clock = TCK_INT/(Prescalar + 1)/(Period + 1)
  //72MHz/7200 = 10KHz
  //10KHz/10 = 1KHz (1ms)
  TIM_TimeBaseStructure.TIM_Prescaler = 0;  //setting to have interrupt frequency = 2 x SPI baud rate   
  TIM_TimeBaseStructure.TIM_Period = 1;              
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM2, DISABLE); 
  
  //set Timer2 interrupt
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);                
} 


/*******************************************************************************
* Function Name  : SPI3_sw_Action
* Description    : change SPI register value
* Input          : 8bit Address, buffer data to R/W, buffer length
* Output         : None
* Return         : None
*******************************************************************************/
u8 SPI3_sw_Action(u8 addr, u8 *data, u8 length)
{
  spi3.mode          = SPI3_SW_MASTER;
  spi3.buffer        = data;
  spi3.length        = length;
  spi3.address       = addr;
  if(addr & 0x80){  
    spi3.rw_operation  = READ_BYTE;
    spi3.byte_bit_cntr = 9;         
  }
  else{   	   
    spi3.rw_operation  = WRITE_BYTE;
    spi3.byte_bit_cntr = 8;         
  }
  spi3.byte_cntr     = 0;
  spi3.status        = SPI3_BUS_STATE_BUSY;
  spi3.bus_state     = SPI3_BUS_STATE_TX_0;
  return 1;
}


/*******************************************************************************
* Function Name  : cs_init
* Description    : Enable designed  CS pin and sets
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/ 
void cs_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  if(spi3.mode==SPI3_SW_MASTER){  
    RCC_APB2PeriphClockCmd(CS_CLK, ENABLE);   
    GPIO_InitStructure.GPIO_Pin = CS_MASK;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CS_PORT, &GPIO_InitStructure);
    high_cs();
  }
}

/*******************************************************************************
* Function Name  : get_sdio
* Description    : Read SDIO line state (low or high)
* Input          : None
* Output         : None
* Return         : SDIO line value 
*******************************************************************************/ 
u8 get_sdio(void)
{
  return GPIO_ReadInputDataBit(SDIO_PORT, SDIO_MASK);
}


/*******************************************************************************
* Function Name  : high_cs
* Description    : Set CS line (high to disable SPI device)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/ 
void high_cs(void){
  GPIO_SetBits(CS_PORT, CS_MASK);
  SPI3_CS_FLAG = 1;
}


/*******************************************************************************
* Function Name  : low_cs
* Description    : Reset CS line (low to enable SPI device)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/ 
void low_cs(void){
  GPIO_ResetBits(CS_PORT, CS_MASK);
  SPI3_CS_FLAG = 0;
}


/*******************************************************************************
* Function Name  : get_scl
* Description    : Read SCL line state (low or high)
* Input          : None
* Output         : None
* Return         : SCL line value 
*******************************************************************************/ 
u8 get_scl(void){ 
  return GPIO_ReadOutputDataBit(SCL_PORT, SCL_MASK);
}


/*******************************************************************************
* Function Name  : set_sdio
* Description    : Set SDIO line (high to transmit bit 1)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/ 
void set_sdio(void){
  GPIO_SetBits(SDIO_PORT, SDIO_MASK); 
}


/*******************************************************************************
* Function Name  : clr_sdio
* Description    : Reset SDIO line (low to transmit bit 0)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/ 
void clr_sdio(void){
  GPIO_ResetBits(SDIO_PORT, SDIO_MASK); 
}


/*******************************************************************************
* Function Name  : set_scl
* Description    : Set SCL line and global refer flag
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/ 
void set_scl(void){
  scl_state=1;
  GPIO_SetBits(SCL_PORT, SCL_MASK); 
}


/*******************************************************************************
* Function Name  : clr_scl
* Description    : Reset SCL line and global refer flag
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/ 
void clr_scl(void){
  scl_state=0;
  GPIO_ResetBits(SCL_PORT, SCL_MASK); 
}


/*******************************************************************************
* Function Name  : toggle_scl
* Description    : toggle SCL line (clock signal)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/ 
void toggle_scl(void){
  if(scl_state)	clr_scl();
  else	set_scl();
}


/*******************************************************************************
* Function Name  : sdio_receiver
* Description    : Change SDIO direction function (SDIO as input)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/ 
void sdio_receiver(void){
  
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  if(spi3.mode==SPI3_SW_MASTER){  
    RCC_APB2PeriphClockCmd(SDIO_CLK, ENABLE); 
    GPIO_InitStructure.GPIO_Pin = SDIO_MASK;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SDIO_PORT, &GPIO_InitStructure);
  }
}


/*******************************************************************************
* Function Name  : sdio_transmitter
* Description    : Change SDIO direction function (SDIO as output)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/ 
void sdio_transmitter(void){
  
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  if(spi3.mode==SPI3_SW_MASTER){  
    RCC_APB2PeriphClockCmd(SDIO_CLK, ENABLE);   
    GPIO_InitStructure.GPIO_Pin = SDIO_MASK;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SDIO_PORT, &GPIO_InitStructure);
  }
}


/*******************************************************************************
* Function Name  : scl_transmitter
* Description    : Change SCL direction function (SDIO as output)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/ 
void scl_transmitter(void){
  
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  if(spi3.mode==SPI3_SW_MASTER){  
    RCC_APB2PeriphClockCmd(SCL_CLK, ENABLE);  
    GPIO_InitStructure.GPIO_Pin = SCL_MASK;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SCL_PORT, &GPIO_InitStructure);
  }
}


/*******************************************************************************
* Function Name  : SPI3_sw_read_register
* Description    : Read register function
* Input          : Start address, buffer data to empty, buffer lengh
* Output         : None
* Return         : SUCCESSFULL / ERROR
*******************************************************************************/
u8 SPI3_sw_read_register(u8 register_add, u8 *buffer, u8 lengh)
{
  u8 ret_val=0;
  register_add = register_add | 0x80;
  if(lengh>1) {
    register_add = register_add | 0x40;
  }   //if >1 byte, read by multiread
  else {
    register_add = register_add & 0xBF;
  }   //if <1 byte, read by singleread 
  ret_val = SPI3_sw_Action(register_add, buffer, lengh);     //set spi3w struct
  if(!ret_val) 
    return ret_val;        //error
  sdio_transmitter();
  set_sdio();
  set_scl();
  low_cs();
  //  While the bus is busy   
  while(spi3.status!=SPI3_BUS_STATE_IDLE){
    //TIM_Cmd(TIM2, ENABLE); //uncomment if use timer interrupt
    toggle_scl();            //comment if use timer interrupt
    SPI3_sw_BitBanging();    //comment if use timer interrupt
  }
  //TIM_Cmd(TIM2, DISABLE);              //uncomment if use timer interrupt
  high_cs();  
  set_sdio();
  set_scl();
  return 1;
}


/*******************************************************************************
* Function Name  : SPI3_sw_write_register
* Description    : Write register function
* Input          : Start address, buffer data to write, buffer lengh
* Output         : None
* Return         : SUCCESSFULL / ERROR
*******************************************************************************/
u8 SPI3_sw_write_register(u8 register_add, u8 *buffer, u8 lengh)
{
  u8 ret_val=0;
  register_add = register_add & 0x7F;
  if(lengh>1) {
    register_add = register_add | 0x40;
  }   //if >1 byte, multiwrite
  else {
    register_add = register_add & 0xBF;
  }   //if <1 byte, singlewrite 
  ret_val = SPI3_sw_Action(register_add, buffer, lengh);  
  if(!ret_val) 
    return ret_val;        //error
  
  sdio_transmitter();
  set_sdio();
  set_scl(); 
  low_cs();
  //  While the bus is busy    
  while(spi3.status!=SPI3_BUS_STATE_IDLE){
    //TIM_Cmd(TIM2, ENABLE);  //uncomment if use timer interrupt
    toggle_scl();             //comment if use timer interrupt
    SPI3_sw_BitBanging();     //comment if use timer interrupt    
  }
  
  //TIM_Cmd(TIM2, DISABLE);               //uncomment if use timer interrupt
  high_cs();
  set_sdio();
  set_scl();  
  return 1;
}


/*******************************************************************************
* Function Name  : SPI3_sw_BitBanging
* Description    : bit banging fnction (to be called 2xSPI_BaudRate only if using timer interrupt)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI3_sw_BitBanging(){
  //----------------MASTER MODE----------------------//
  //only if scl is low 
  if(!scl_state){  
    switch(spi3.bus_state){ 
      //------------- Transmit address --------
    case SPI3_BUS_STATE_TX_0:     //trasmit first byte
      if(spi3.address & 0x80) 
        set_sdio();
      else 
        clr_sdio();
      spi3.address <<= 1;
      spi3.byte_bit_cntr--;
      if(spi3.byte_bit_cntr==0) {   //end byte
        if(spi3.rw_operation == READ_BYTE){  //read mode
          spi3.bus_state = SPI3_BUS_STATE_RX_0;
          spi3.byte_bit_cntr = 8; //8
          sdio_receiver(); //invert sdio direction
        }
        //write mode
        else {
          spi3.bus_state = SPI3_BUS_STATE_TX_1;
          spi3.byte_bit_cntr = 9;
        }
      }
      break;
      
      //--------------Transmit byte------------//
    case SPI3_BUS_STATE_TX_1:     //trasmit bytes
      if(spi3.buffer[spi3.byte_cntr] & 0x80) 
        set_sdio();
      else 
        clr_sdio();
      spi3.buffer[spi3.byte_cntr] <<= 1;
      spi3.byte_bit_cntr--;
      if(spi3.byte_bit_cntr==0){     //end byte
        spi3.byte_cntr++;
        spi3.byte_bit_cntr = 9;
        if (spi3.byte_cntr<spi3.length) spi3.bus_state = SPI3_BUS_STATE_TX_1;
        //stop SPI transmission, reset couter
        else 
          spi3.status = SPI3_BUS_STATE_IDLE;  
      }      
      break;
      
    }//end switch    
  }//end if SCL low
  
  //scl high and rx mode state
  if( (scl_state) && (spi3.bus_state == SPI3_BUS_STATE_RX_0)){
    spi3.buffer[spi3.byte_cntr] = ((spi3.buffer[spi3.byte_cntr] <<= 1) | get_sdio() ) & 0xFF;    
    spi3.byte_bit_cntr--;
    //multibyte
    if(!spi3.byte_bit_cntr){
      spi3.byte_bit_cntr = 8;
      spi3.byte_cntr++;
      if (spi3.byte_cntr<spi3.length)
        spi3.bus_state = SPI3_BUS_STATE_RX_0;
      //stop SPI transmission, reset couter
      else
        spi3.status = SPI3_BUS_STATE_IDLE; 
    }
    else
      spi3.bus_state = SPI3_BUS_STATE_RX_0;			  
  }//end  scl high
}//end bitbanging    



/*---------example interrupt hendler if using timer interrupt------------

void TIM2_IRQHandler(void) {
if ((TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET))
{      
if(  SPI3_CS_FLAG ==0 ){
toggle_scl();
SPI3_sw_BitBanging();
    }
TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
}
----------------------*/


/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/