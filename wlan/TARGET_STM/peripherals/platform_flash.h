/**
 ******************************************************************************
 * @file    platform_flash.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This file provides flash operation functions.
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "mico_board.h"
#include "mico_board_conf.h"

#ifdef USE_MICO_SPI_FLASH
#include "spi_flash.h"
#endif

#ifdef USE_QUAD_SPI_FLASH
#include "spi_flash.h"
#include "spi_flash_internal.h"
#endif

#ifdef USE_QUAD_SPI_FLASH
extern const platform_qspi_t platform_qspi_peripherals[];
#endif


int iflash_init( void );
int iflash_erase( uint32_t device_start_addr, uint32_t device_end_addr );
int iflash_write( volatile uint32_t* device_address, uint32_t* data_addr, uint32_t size );
int iflash_read( unsigned long device_address, void* const data_addr, uint32_t size );

#ifdef MCU_ENABLE_FLASH_PROTECT
int iflash_protect(uint32_t StartAddress, uint32_t EndAddress, bool enable);
#endif



#ifdef USE_MICO_SPI_FLASH
static OSStatus spiFlashErase(uint32_t StartAddress, uint32_t EndAddress);
#endif



#ifdef USE_QUAD_SPI_FLASH

int qsflash_erase( uint32_t StartAddress, uint32_t EndAddress );
int qsflash_init( /*@out@*/ const platform_qspi_t* qspi, const platform_flash_t* flash, /*@out@*/ sflash_handle_t* const handle, sflash_write_allowed_t write_allowed_in );
int qsflash_read_ID( sflash_handle_t* const handle, void* const data_addr );
int qsflash_write( const sflash_handle_t* const handle, unsigned long device_address, const void* const data_addr, unsigned int size );
int qsflash_read( const sflash_handle_t* const handle, unsigned long device_address, void* const data_addr, unsigned int size );
#endif
