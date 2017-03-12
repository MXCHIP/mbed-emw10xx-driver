/*
 * Copyright 2013, Broadcom Corporation
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */

#include "internal/wwd_internal.h"
#include "internal/Bus_protocols/wwd_bus_protocol_interface.h"
#include "internal/wwd_sdpcm.h"
#include "wwd_management.h"
#include <string.h>
#include "RTOS/wwd_rtos_interface.h"
#include "chip_constants.h"
#include "bcmendian.h"
#include <stdlib.h>

/******************************************************
 *             Constants
 ******************************************************/

#define BACKPLANE_ADDRESS_MASK  0x7FFF
#define AI_IOCTRL_OFFSET      0x408
#define SICF_FGC              0x0002
#define SICF_CLOCK_EN         0x0001
#define AI_RESETCTRL_OFFSET   0x800
#define AIRC_RESET            1

#define WRAPPER_REGISTER_OFFSET    0x100000

/******************************************************
 *             Structures
 ******************************************************/

/******************************************************
 *             Variables
 ******************************************************/

static const uint32_t core_base_address[] =
{
#ifdef WLAN_ARMCM3_BASE_ADDRESS
    (uint32_t) ( WLAN_ARMCM3_BASE_ADDRESS + WRAPPER_REGISTER_OFFSET  ),
#endif /* ifdef WLAN_ARMCM3_BASE_ADDRESS */
#ifdef WLAN_ARMCR4_BASE_ADDRESS
    (uint32_t) ( WLAN_ARMCR4_BASE_ADDRESS + WRAPPER_REGISTER_OFFSET  ),
#endif /* ifdef WLAN_ARMCR4_BASE_ADDRESS */
    (uint32_t) ( SOCSRAM_BASE_ADDRESS + WRAPPER_REGISTER_OFFSET ),
    (uint32_t) ( SDIO_BASE_ADDRESS )
};

wiced_wlan_status_t wiced_wlan_status =
{
    .state             = WLAN_DOWN,
    .country_code      = WICED_COUNTRY_AUSTRALIA,
    .keep_wlan_awake = 0,
};

/******************************************************
 *             Function declarations
 ******************************************************/
wiced_result_t wiced_abort_read( wiced_bool_t retry );

static uint32_t wiced_get_core_address( device_core_t core_id );

/******************************************************
 *             Function definitions
 ******************************************************/

/*
 * Update the backplane window registers
 */
wiced_result_t wiced_set_backplane_window( uint32_t addr )
{
    wiced_result_t result = WICED_ERROR;
    static uint32_t current_base_address = 0;
    uint32_t base = addr & ( (uint32_t) ~BACKPLANE_ADDRESS_MASK );

    if ( base == current_base_address )
    {
        return WICED_SUCCESS;
    }
    if ( ( base & 0xFF000000 ) != ( current_base_address & 0xFF000000 ) )
    {
        if ( WICED_SUCCESS != ( result = wiced_write_register_value( BACKPLANE_FUNCTION, SDIO_BACKPLANE_ADDRESS_HIGH, (uint8_t) 1, ( base >> 24 ) ) ) )
        {
            return result;
        }
    }
    if ( ( base & 0x0FF0000 ) != ( current_base_address & 0x00FF0000 ) )
    {
        if ( WICED_SUCCESS != ( result = wiced_write_register_value( BACKPLANE_FUNCTION, SDIO_BACKPLANE_ADDRESS_MID, (uint8_t) 1, ( base >> 16 ) ) ) )
        {
            return result;
        }
    }
    if ( ( base & 0x0000FF00 ) != ( current_base_address & 0x0000FF00 ) )
    {
        if ( WICED_SUCCESS != ( result = wiced_write_register_value( BACKPLANE_FUNCTION, SDIO_BACKPLANE_ADDRESS_LOW, (uint8_t) 1, ( base >> 8 ) ) ) )
        {
            return result;
        }
    }

    current_base_address = base;
    return WICED_SUCCESS;
}

/*
 * Returns the base address of the core identified by the provided coreId
 */
uint32_t wiced_get_core_address( device_core_t core_id )
{
    return core_base_address[(int) core_id];
}

/*
 * Returns WICED_SUCCESS is the core identified by the provided coreId is up, otherwise WICED_ERROR
 */
wiced_result_t wiced_device_core_is_up( device_core_t core_id )
{
    uint8_t regdata;
    uint32_t base;
    wiced_result_t result;

    base = wiced_get_core_address( core_id );

    /* Read the IO control register */
    result = wiced_read_backplane_value( base + AI_IOCTRL_OFFSET, (uint8_t) 1, &regdata );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    /* Verify that the clock is enabled and something else is not on */
    if ( ( regdata & ( SICF_FGC | SICF_CLOCK_EN ) ) != (uint8_t) SICF_CLOCK_EN )
    {
        return WICED_ERROR;
    }

    /* Read the reset control and verify it is not in reset */
    result = wiced_read_backplane_value( base + AI_RESETCTRL_OFFSET, (uint8_t) 1, &regdata );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }
    if ( ( regdata & AIRC_RESET ) != 0 )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

/*
 * Disables the core identified by the provided coreId
 */
wiced_result_t wiced_disable_device_core( device_core_t core_id )
{
    uint32_t base = wiced_get_core_address( core_id );
    wiced_result_t result;
    uint8_t junk;

    /* Read the reset control */
    result = wiced_read_backplane_value( base + AI_RESETCTRL_OFFSET, (uint8_t) 1, &junk );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    result = wiced_read_backplane_value( base + AI_RESETCTRL_OFFSET, (uint8_t) 1, &junk );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    /* Write 0 to the IO control and read it back */
    result = wiced_write_backplane_value( base + AI_IOCTRL_OFFSET, (uint8_t) 1, 0 );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    result = wiced_read_backplane_value( base + AI_IOCTRL_OFFSET, (uint8_t) 1, &junk );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    host_rtos_delay_milliseconds( (uint32_t) 1 );

    result = wiced_write_backplane_value( base + AI_RESETCTRL_OFFSET, (uint8_t) 1, (uint32_t) AIRC_RESET );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    host_rtos_delay_milliseconds( (uint32_t) 1 );

    return result;
}

/*
 * Resets the core identified by the provided coreId
 */
wiced_result_t wiced_reset_device_core( device_core_t core_id )
{
    uint32_t base = wiced_get_core_address( core_id );
    wiced_result_t result;
    uint8_t junk;

    result = wiced_write_backplane_value( base + AI_IOCTRL_OFFSET, (uint8_t) 1, (uint32_t) ( SICF_FGC | SICF_CLOCK_EN ) );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    result = wiced_read_backplane_value( base + AI_IOCTRL_OFFSET, (uint8_t) 1, &junk );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    result = wiced_write_backplane_value( base + AI_RESETCTRL_OFFSET, (uint8_t) 1, 0 );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    host_rtos_delay_milliseconds( (uint32_t) 1 );

    result = wiced_write_backplane_value( base + AI_IOCTRL_OFFSET, (uint8_t) 1, (uint32_t) SICF_CLOCK_EN );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    result = wiced_read_backplane_value( base + AI_IOCTRL_OFFSET, (uint8_t) 1, &junk );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    host_rtos_delay_milliseconds( (uint32_t) 1 );

    return result;
}




inline uint16_t bcmswap16( uint16_t val )
{
    return BCMSWAP16(val);
}

inline uint32_t bcmswap32( uint32_t val )
{
    return BCMSWAP32(val);
}

inline uint32_t bcmswap32by16( uint32_t val )
{
    return BCMSWAP32BY16(val);
}

/* Reverse pairs of bytes in a buffer (not for high-performance use) */
/* buf  - start of buffer of shorts to swap */
/* len  - byte length of buffer */
inline void bcmswap16_buf( uint16_t* buf, uint32_t len )
{
    len = len / 2;

    while ( ( len-- ) != 0 )
    {
        *buf = bcmswap16( *buf );
        buf++;
    }
}

/*
 * Store 16-bit value to unaligned little-endian byte array.
 */
inline void htol16_ua_store( uint16_t val, uint8_t* bytes )
{
    bytes[0] = (uint8_t) ( val & 0xff );
    bytes[1] = (uint8_t) ( val >> 8 );
}

/*
 * Store 32-bit value to unaligned little-endian byte array.
 */
inline void htol32_ua_store( uint32_t val, uint8_t* bytes )
{
    bytes[0] = (uint8_t) ( val & 0xff );
    bytes[1] = (uint8_t) ( ( val >> 8 ) & 0xff );
    bytes[2] = (uint8_t) ( ( val >> 16 ) & 0xff );
    bytes[3] = (uint8_t) ( val >> 24 );
}

/*
 * Store 16-bit value to unaligned network-(big-)endian byte array.
 */
inline void hton16_ua_store( uint16_t val, uint8_t* bytes )
{
    bytes[0] = (uint8_t) ( val >> 8 );
    bytes[1] = (uint8_t) ( val & 0xff );
}

/*
 * Store 32-bit value to unaligned network-(big-)endian byte array.
 */
inline void hton32_ua_store( uint32_t val, uint8_t* bytes )
{
    bytes[0] = (uint8_t) ( val >> 24 );
    bytes[1] = (uint8_t) ( ( val >> 16 ) & 0xff );
    bytes[2] = (uint8_t) ( ( val >> 8 ) & 0xff );
    bytes[3] = (uint8_t) ( val & 0xff );
}

/*
 * Load 16-bit value from unaligned little-endian byte array.
 */
inline uint16_t ltoh16_ua( const void* bytes )
{
    return (uint16_t) _LTOH16_UA((const uint8_t*)bytes);
}

/*
 * Load 32-bit value from unaligned little-endian byte array.
 */
inline uint32_t ltoh32_ua( const void* bytes )
{
    return (uint32_t) _LTOH32_UA((const uint8_t*)bytes);
}

/*
 * Load 16-bit value from unaligned big-(network-)endian byte array.
 */
inline uint16_t ntoh16_ua( const void* bytes )
{
    return (uint16_t) _NTOH16_UA((const uint8_t*)bytes);
}

/*
 * Load 32-bit value from unaligned big-(network-)endian byte array.
 */
inline uint32_t ntoh32_ua( const void* bytes )
{
    return (uint32_t) _NTOH32_UA((const uint8_t*)bytes);
}


static wiced_result_t read_data_frame(int address, uint8_t * data, uint16_t len)
{
#define addr_mask_low     	0x80
#define addr_mask_mid     	0xff
#define addr_mask_hig     	0xff
#define start_addr_low		0x1000A
#define start_addr_mid		0x1000B
#define start_addr_hig		0x1000C

	wiced_result_t result = 0;
	uint8_t add = 0;
    wiced_transfer_bytes_packet_t* p;

    p = (wiced_transfer_bytes_packet_t*)malloc(sizeof(wiced_transfer_bytes_packet_t) + len);
    if (p == NULL)
        return WICED_ERROR;
    
	add = (uint8_t)((address >> 8 ) & addr_mask_low);
    wiced_write_register_value( BACKPLANE_FUNCTION, start_addr_low, (uint8_t) 1, add );
	add = (uint8_t)((address >> 16 ) & addr_mask_mid);
    wiced_write_register_value( BACKPLANE_FUNCTION, start_addr_mid, (uint8_t) 1, add );
	add = (uint8_t)((address >> 24 ) & addr_mask_hig);
    wiced_write_register_value( BACKPLANE_FUNCTION, start_addr_hig, (uint8_t) 1, add );

    memcpy( p->data, data, len );
    result = wiced_bus_transfer_bytes( BUS_READ, BACKPLANE_FUNCTION, address&0xffff, (uint16_t)(len), p );
    memcpy( data, p->data, len );
    //result = wiced_bus_transfer_bytes(BUS_READ, BACKPLANE_FUNCTION, address & 0xffff, (uint16_t)len, (wiced_transfer_bytes_packet_t*)(data));
    
	add = 0x0;
    wiced_write_register_value( BACKPLANE_FUNCTION, start_addr_low, (uint8_t) 1, add );
	add = 0x0;
    wiced_write_register_value( BACKPLANE_FUNCTION, start_addr_mid, (uint8_t) 1, add );
	add = 0x18;
    wiced_write_register_value( BACKPLANE_FUNCTION, start_addr_hig, (uint8_t) 1, add );

    free(p);
	return result;

}

typedef struct {
	uint32_t	flags;
	uint32_t  	trap_addr;
	uint32_t  	assert_exp_addr;
	uint32_t  	assert_file_addr;
	uint32_t  	assert_line;
	uint32_t	console_addr;
	uint32_t  	msgtrace_addr;
	uint32_t  	brpt_addr;
} sdpcm_tt;

int log_idx = 0;

/*
 * From internal documentation: hwnbu-twiki/SdioMessageEncapsulation
 * When data is available on the device, the device will issue an interrupt:
 * - the device should signal the interrupt as a hint that one or more data frames may be available on the device for reading
 * - the host may issue reads of the 4 byte length tag at any time -- that is, whether an interupt has been issued or not
 * - if a frame is available, the tag read should return a nonzero length (>= 4) and the host can then read the remainder of the frame by issuing one or more CMD53 reads
 * - if a frame is not available, the 4byte tag read should return zero
 */
/*@only@*//*@null@*/wiced_result_t wiced_read_console(char *console_log, int len)
{
#define console_header_add 		0x0003a6f8
#define console_buffer_add 		0x0000a2e8
#define console_header_add1 	0x0003a6f8
#define shared_ram_add			0x0003bffc
#define shared_header_add	    0x000209e8

	sdpcm_tt shared_header;
	int console_header[4];
	int console_address = 0;
	int console_len = 0;
	static int console_header_address = 0;
	int ram_data = 0;
	
    /* Ensure the wlan backplane bus is up */
    if ( wwd_ensure_wlan_bus_is_up() != WWD_SUCCESS )
    {
        return WICED_ERROR;
    }
	
    memset((void *)(&shared_header), 0, sizeof(shared_header) );
	memset((void *)(console_log), 0, (size_t)len );

/////////////////////get real console_header_address
	if(console_header_address == 0) {
		
	    if ( WICED_SUCCESS != (read_data_frame(shared_ram_add, (uint8_t*)(&ram_data), (uint16_t)sizeof(ram_data))))
	    {
	        wiced_abort_read( WICED_FALSE );

	        return WICED_ERROR;
	    }

	    if ( WICED_SUCCESS != (read_data_frame(ram_data, (uint8_t*)(&shared_header), (uint16_t)sizeof(sdpcm_tt))))
	    {
	        wiced_abort_read( WICED_FALSE );

	        return WICED_ERROR;
	    }
		console_header_address = (int)(shared_header.console_addr + 8);
	} 
	

//////////////get real console header
    if ( WICED_SUCCESS != (read_data_frame(console_header_address, (uint8_t*)(console_header), (uint16_t)sizeof(console_header)) ))
    {
        wiced_abort_read( WICED_FALSE );

        return WICED_ERROR;
    }

	console_address = console_header[3];
	if(log_idx == console_header[2])
		return WICED_SUCCESS;

    if(console_header[2] > log_idx) {
	    console_len = console_header[2] - log_idx;
        if (console_len > len) {
            console_len = len;
        }
        
        if ( WICED_SUCCESS != (read_data_frame(console_address + log_idx, (uint8_t*)(console_log), (uint16_t)console_len) ))
        {
            wiced_abort_read( WICED_FALSE );
                return WICED_ERROR;
        }
        log_idx += console_len;
    } else {
        console_len = console_header[1] - log_idx;
        if (console_len > len) {
            console_len = len;
        }
        
        if ( WICED_SUCCESS != (read_data_frame(console_address + log_idx, (uint8_t*)(console_log), (uint16_t)console_len) ))
        {
            wiced_abort_read( WICED_FALSE );               
            return WICED_ERROR;
        }
        
        log_idx += console_len;
        if (log_idx == console_header[1])
            log_idx = 0;
        if (len > console_len) {
            len = len - console_len;
            
            if ( WICED_SUCCESS != (read_data_frame(console_address, (uint8_t*)(&console_log[console_len]), (uint16_t)len) ))
            {
                wiced_abort_read( WICED_FALSE );
                return WICED_ERROR;
            }
            log_idx = len;
        }
        
    }

    return WICED_SUCCESS;
}

