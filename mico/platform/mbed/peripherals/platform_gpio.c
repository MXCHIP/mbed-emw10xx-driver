/**
 ******************************************************************************
 * @file    platform_gpio.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This file provide GPIO driver functions.
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

//#include "platform.h"
#include "platform_peripheral.h"
#include "pinmap.h"
#include "platform/critical.h"
//#include "platform_logging.h"

/******************************************************
*                    Constants
******************************************************/

/******************************************************
*                   Enumerations
******************************************************/

/******************************************************
*                 Type Definitions
******************************************************/

/******************************************************
*                    Structures
******************************************************/

/******************************************************
*               Variables Definitions
******************************************************/

/******************************************************
*               Function Declarations
******************************************************/

/******************************************************
*               Function Definitions
******************************************************/

OSStatus platform_gpio_init( platform_gpio_driver_t* driver, const platform_gpio_t* gpio, platform_pin_config_t config )
{
    platform_mcu_powersave_disable( );

    gpio_init( &driver->gpio, gpio->mbed_pin );

    if ( config == INPUT_PULL_UP ) {
        gpio_dir( &driver->gpio, PIN_INPUT );
        gpio_mode( &driver->gpio, PullUp );
    }
    else if ( config == INPUT_PULL_DOWN ) {
        gpio_dir( &driver->gpio, PIN_INPUT );
        gpio_mode( &driver->gpio, PullDown );
    }
    else if ( config == INPUT_HIGH_IMPEDANCE ) {
        gpio_dir( &driver->gpio, PIN_INPUT );
        gpio_mode( &driver->gpio, PullNone );
    }
    else if ( config == OUTPUT_PUSH_PULL ) {
        gpio_dir( &driver->gpio, PIN_OUTPUT );
        gpio_mode( &driver->gpio, PullNone );
    }
    else if ( config == OUTPUT_OPEN_DRAIN_NO_PULL || OUTPUT_OPEN_DRAIN_PULL_UP ) {
        gpio_dir( &driver->gpio, PIN_OUTPUT );
        gpio_mode( &driver->gpio, OpenDrain );
    }

    platform_mcu_powersave_enable( );
    return kNoErr;
}

OSStatus platform_gpio_deinit( platform_gpio_driver_t* driver )
{
    gpio_dir( &driver->gpio, PIN_INPUT );
    gpio_mode( &driver->gpio, PullNone );
    return kNoErr;
}

OSStatus platform_gpio_output_high( platform_gpio_driver_t* driver )
{
    platform_mcu_powersave_disable( );

    gpio_write( &driver->gpio, 1 );

    platform_mcu_powersave_enable( );
    return kNoErr;
}

OSStatus platform_gpio_output_low( platform_gpio_driver_t* driver )
{
    platform_mcu_powersave_disable( );

    gpio_write( &driver->gpio, 0 );

    platform_mcu_powersave_enable( );
    return kNoErr;
}

OSStatus platform_gpio_output_trigger( platform_gpio_driver_t* driver )
{
    platform_mcu_powersave_disable( );

    gpio_write( &driver->gpio, 0 == gpio_read( &driver->gpio ) ? 1 : 0 );

    platform_mcu_powersave_enable( );
    return kNoErr;
}

bool platform_gpio_input_get( platform_gpio_driver_t* driver )
{
    bool result = false;

    platform_mcu_powersave_disable( );
    if ( gpio_read( &driver->gpio ) ) result = true;
    platform_mcu_powersave_enable( );

    return result;
}

static void _irq_handler( uint32_t id, gpio_irq_event event )
{
    platform_gpio_irq_driver_t* irq_driver = (platform_gpio_irq_driver_t*) id;
    if( irq_driver->fun ) irq_driver->fun( irq_driver->arg );
}

OSStatus platform_gpio_irq_enable( platform_gpio_irq_driver_t* irq_driver,  const platform_gpio_t* gpio, platform_gpio_irq_trigger_t trigger, platform_gpio_irq_callback_t handler, void* arg )
{
    core_util_critical_section_enter();
    gpio_irq_init( &irq_driver->gpio_irq, gpio->mbed_pin, _irq_handler, (uint32_t) irq_driver );
    irq_driver->fun = handler;
    irq_driver->arg = arg;

    switch ( trigger )
    {
        case IRQ_TRIGGER_RISING_EDGE: {
            gpio_irq_set( &irq_driver->gpio_irq, IRQ_RISE, 1 );
            gpio_irq_set( &irq_driver->gpio_irq, IRQ_FALL, 0 );
            break;
        }
        case IRQ_TRIGGER_FALLING_EDGE: {
            gpio_irq_set( &irq_driver->gpio_irq, IRQ_RISE, 0 );
            gpio_irq_set( &irq_driver->gpio_irq, IRQ_FALL, 1 );
            break;
        }
        case IRQ_TRIGGER_BOTH_EDGES: {
            gpio_irq_set( &irq_driver->gpio_irq, IRQ_RISE, 1 );
            gpio_irq_set( &irq_driver->gpio_irq, IRQ_FALL, 1 );
            break;
        }
        default: {
            gpio_irq_set( &irq_driver->gpio_irq, IRQ_RISE, 0 );
            gpio_irq_set( &irq_driver->gpio_irq, IRQ_FALL, 0 );
            break;
        }
    }

    gpio_irq_enable( &irq_driver->gpio_irq );
    core_util_critical_section_exit();
    return kNoErr;
}


OSStatus platform_gpio_irq_disable( platform_gpio_irq_driver_t* irq_driver )
{
    platform_mcu_powersave_disable( );

    irq_driver->fun = NULL;
    irq_driver->arg = NULL;
    gpio_irq_free( &irq_driver->gpio_irq );

    platform_mcu_powersave_enable( );
    return kNoErr;
}



