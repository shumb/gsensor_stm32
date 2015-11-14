/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : adc_mems.h
* Author             : MSH Application Team
* Version            : V1.1
* Date               : 04/05/2010
* Description        : Descriptor Header for adc mems file
* HISTORY:
  * Date        | Modification        | Author
  04/05/2010    | Initial Revision    |Andrea Labombarda
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
#ifndef __ADC_MEMS_H
#define __ADC_MEMS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_adc.h"
/* Exported typedef -----------------------------------------------------------*/
/* Exported define ------------------------------------------------------------*/
/* Exported macro -------------------------------------------------------------*/
/* Exported variables ---------------------------------------------------------*/
/* Exported variables ----------------------------------------------------------*/
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
/*----- High layer function -----*/
void ADC_Mems_Init(void);
u16 readADC1(u8 channel);

#endif /* __ADC_MEMS_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/