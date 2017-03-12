/**
 ******************************************************************************
 * @file    mico_platform.h
 * @author  William Xu
 * @version V1.0.0
 * @date    16-Sep-2014
 * @brief   This file provides all the headers of MICO peripheral operations.
 ******************************************************************************
 *
 *  The MIT License
 *  Copyright (c) 2014 MXCHIP Inc.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is furnished
 *  to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 *  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ******************************************************************************
 */


#pragma once

#ifndef __MICOPLATFORM_H__
#define __MICOPLATFORM_H__

#include "mico_common.h"

#include "platform.h" /* This file is unique for each platform */
#include "platform_peripheral.h"

#ifdef __cplusplus
extern "C" {
#endif

// typedef platform_spi_slave_config_t             mico_spi_slave_config_t;

// typedef platform_spi_slave_transfer_direction_t mico_spi_slave_transfer_direction_t;

// typedef platform_spi_slave_transfer_status_t    mico_spi_slave_transfer_status_t;

// typedef platform_spi_slave_command_t            mico_spi_slave_command_t;

// typedef platform_spi_slave_data_buffer_t        mico_spi_slave_data_buffer_t;


#include "mico_driver_gpio.h"
// #include "MiCODrivers/MiCODriverI2c.h"
// #include "MiCODrivers/MiCODriverSpi.h"
// #include "MiCODrivers/MiCODriverUart.h"
// #include "MiCODrivers/MiCODriverGpio.h"
// #include "MiCODrivers/MiCODriverPwm.h"
// #include "MiCODrivers/MiCODriverRtc.h"
// #include "MiCODrivers/MiCODriverWdg.h"
// #include "MiCODrivers/MiCODriverAdc.h"
// #include "MiCODrivers/MiCODriverRng.h"
#include "mico_driver_flash.h"
// #include "MiCODrivers/MiCODriverMFiAuth.h"

// #define mico_mcu_powersave_config MicoMcuPowerSaveConfig

// #ifdef USE_MICO_SPI_FLASH
// extern const mico_spi_device_t mico_spi_flash;
// #endif

/** @defgroup MICO_PLATFORM  MICO Hardware Abstract Layer APIs
*   @brief Control hardware peripherals on different platfroms using standard HAL API functions
* 
*/
/** @addtogroup MICO_PLATFORM
  * @{
  */

/** @defgroup platform_misc Task switching, reboot, and standby
  * @brief Provide task switching,reboot and standby functions
  * @{
  */

#define ENABLE_INTERRUPTS   __asm("CPSIE i")  /**< Enable interrupts to start task switching in MICO RTOS. */
#define DISABLE_INTERRUPTS  __asm("CPSID i")  /**< Disable interrupts to stop task switching in MICO RTOS. */


/** @brief    Software reboot the MICO hardware
  *
  * @param    none
  * @return   none
  */
void mico_system_reboot(void);


/** @brief    Software reboot the MICO hardware
  *
  * @param    timeToWakeup: MICO will wakeup after secondsToWakeup (seconds)
  * @return   none
  */
void MicoSystemStandBy(uint32_t secondsToWakeup);


/** @brief    Enables the MCU to enter deep sleep mode when all threads are suspended.
  *
  * @note:    When all threads are suspended, mcu can shut down some peripherals to 
  *           save power. For example, STM32 enters STOP mode in this condition, 
  *           its peripherals are not working and needs to be wake up by an external
  *           interrupt or MICO core's internal timer. So if you are using a peripherals,  
  *           you should disable this function temporarily.
  *           To make this function works, you should not disable the macro in MicoDefault.h: 
  *           MICO_DISABLE_MCU_POWERSAVE
  *
  * @param    enable : 1 = enable MCU powersave, 0 = disable MCU powersave
  * @return   none
  */
void MicoMcuPowerSaveConfig( int enable );


/** @brief    Set MiCO system led on/off state
  *
  * @param    onff: State of syetem LED,0: OFF,1:ON
  * @return   none
  */
void MicoSysLed(bool onoff);

/** @brief     Set MiCO RF led on/off state
  *
  * @param    onff: State of RF LED,0: OFF,1:ON
  * @return   none
  */
void MicoRfLed(bool onoff);

/** @brief    add
  *
  * @param    add
  * @return   none
  */
bool MicoShouldEnterMFGMode(void);


/** @brief    add
  *
  * @param    add
  * @return   none
  */
bool MicoShouldEnterATEMode(void);


/** @brief    add
  *
  * @param    add
  * @return   none
  */
bool MicoShouldEnterBootloader(void);


/** @brief    add
  *
  * @param    add
  * @return   none
  */
char *mico_get_bootloader_ver(void);


#ifdef BOOTLOADER 
void mico_set_bootload_ver(void);
#endif
/**
  * @}
  */


/**
  * @}
  */
#ifdef __cplusplus
} /*extern "C" */
#endif


#endif

