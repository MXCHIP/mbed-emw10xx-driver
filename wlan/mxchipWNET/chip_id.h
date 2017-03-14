/**
  ******************************************************************************
  * @file    IAP/inc/chip_id.h 
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    07/27/2009
  * @brief   
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _CHIP_ID_H
#define _CHIP_ID_H

/* Includes ------------------------------------------------------------------*/
#ifdef MXCHIP_3163
#include "stm32f4xx.h"
#else
#include "stm32f2xx.h"
#endif
/** @addtogroup Exported_types
  * @{
  */  

/*!< STM32F10x Standard Peripheral Library old types (maintained for legacy purpose) */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Constants used by Serial Command Line Mode */
/* Exported macro ------------------------------------------------------------*/

//#define OLD_CHIPID

#define  UNIQUE_CODE_ADDR   0x8001000
#ifdef OLD_CHIPID

#define  UNIQUE_ID_ADDR_0	0x1FFFF7E8
#define  UNIQUE_ID_ADDR_1	0x1FFFF7EC
#define  UNIQUE_ID_ADDR_2	0x1FFFF7F0
#else
// below is the real chip id address, use it when bootloader updated.
#define  UNIQUE_ID_ADDR_0	0x1FFF7A10
#define  UNIQUE_ID_ADDR_1	0x1FFF7A14
#define  UNIQUE_ID_ADDR_2	0x1FFF7A18
#endif
#define	 ENCRYPT_KEY	  	0x52655026
//#define  BOOT_CHIPID		


/* Exported functions ------------------------------------------------------- */

void Verify_Program_Code(void);
void APP_Verify_Program_Code(void);

#endif  /* _CHIP_ID_H */

/*******************(C)COPYRIGHT 2009 STMicroelectronics *****END OF FILE******/
