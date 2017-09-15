/**
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 *
 */

/*******************************************************************************
 **  Name:       userial_bby.c
 **
 **  Description:
 **
 **  This file contains the universal driver wrapper for the BTE-QC serial
 **  drivers
 *******************************************************************************/

#include "mico.h"

#include "platform_bluetooth.h"


/* Macro for checking of bus is initialised */
#define IS_BUS_INITIALISED() \
do \
{ \
    if ( bus_initialised == false ) \
    { \
        printf( "bus uninitialised" ); \
        return -1; \
    } \
}while ( 0 )

/* Macro for checking if bus is ready */
#define BT_BUS_IS_READY() \
do \
{ \
    if ( bt_bus_is_ready( ) == false ) \
    { \
        printf( "bus not ready" ) \
        return kGeneralErr; \
    } \
}while ( 0 )

/* Macro for waiting until bus is ready */
#define BT_BUS_WAIT_UNTIL_READY() \
do \
{ \
    while ( bt_bus_is_ready( ) == false ) \
    { \
        mico_rtos_thread_msleep( 10 ); \
    } \
} while ( 0 )

/* TODO: bring in bt_bus code to remove BTE dependency on MiCO bluetooth library */
extern int bt_bus_init(void);

extern int bt_bus_deinit(void);

extern int bt_bus_transmit(const uint8_t *data_out, uint32_t size);

extern int bt_bus_receive(uint8_t *data_in, uint32_t size, uint32_t timeout_ms);

extern int bt_bus_uart_reset(void);

extern int bt_bus_uart_reconifig_baud(uint32_t baud);

extern bool bt_bus_is_ready(void);

/******************************************************
 *                   Enumerations
 ******************************************************/


/*****************************************************************************
 * Platform UART interface, taken from
 * ../Library/bluetooth/internal/bus/UART/bt_bus.c
 * (audio/2.4.x-bluetooth branch)
 *****************************************************************************/

#ifndef USERIAL_RX_FIFO_SIZE
#define USERIAL_RX_FIFO_SIZE (3000)
#endif

static volatile bool bus_initialised = false;
static volatile bool device_powered = false;

/* RX ring buffer. Bluetooth chip UART receive can be asynchronous, therefore a ring buffer is required */
static ring_buffer_t rx_ring_buffer;
static uint8_t rx_data[USERIAL_RX_FIFO_SIZE];

int bt_bus_init(void)
{
    if (bus_initialised == false) {
        if (bt_control_pins[BT_PIN_HOST_WAKE] != NULL) {
            require_noerr(platform_gpio_init(&bt_control_pin_drivers[BT_PIN_HOST_WAKE],
                                             bt_control_pins[BT_PIN_HOST_WAKE],
                                             INPUT_HIGH_IMPEDANCE),
                          exit);
        }

        if (bt_control_pins[BT_PIN_DEVICE_WAKE] != NULL) {
            require_noerr(platform_gpio_init(&bt_control_pin_drivers[BT_PIN_DEVICE_WAKE],
                                             bt_control_pins[BT_PIN_DEVICE_WAKE],
                                             OUTPUT_OPEN_DRAIN_PULL_UP),
                          exit);
            require_noerr(platform_gpio_output_high(&bt_control_pin_drivers[BT_PIN_DEVICE_WAKE]), exit);
            mico_rtos_thread_msleep(100);
        }

        /* Configure Reg Enable pin to output. Set to HIGH */
        if (bt_control_pins[BT_PIN_POWER] != NULL) {
            require_noerr(platform_gpio_init(&bt_control_pin_drivers[BT_PIN_POWER], bt_control_pins[BT_PIN_POWER],
                                             OUTPUT_OPEN_DRAIN_PULL_UP), exit);
            require_noerr(platform_gpio_output_high(&bt_control_pin_drivers[BT_PIN_POWER]), exit);
        }
        device_powered = true;

        if (bt_uart_config.flow_control == FLOW_CONTROL_DISABLED) {
            /* Configure RTS pin to output. Set to HIGH */
            require_noerr(platform_gpio_init(&bt_uart_pin_drivers[BT_PIN_UART_RTS], bt_uart_pins[BT_PIN_UART_RTS],
                                             OUTPUT_OPEN_DRAIN_PULL_UP), exit);

            // William, working wrong if set high, so donot use FLOW_CONTROL_DISABLED in 43438A1
            require_noerr(platform_gpio_output_high(&bt_uart_pin_drivers[BT_PIN_UART_RTS]),
                          exit);

            /* Configure CTS pin to input pull-up */
            require_noerr(platform_gpio_init(&bt_uart_pin_drivers[BT_PIN_UART_CTS], bt_uart_pins[BT_PIN_UART_CTS],
                                             INPUT_PULL_UP), exit);
        }

        /* Configure Reset pin to output. Set to HIGH */
        if (bt_control_pins[BT_PIN_RESET] != NULL) {
            require_noerr(platform_gpio_init(&bt_control_pin_drivers[BT_PIN_RESET], bt_control_pins[BT_PIN_RESET],
                                             OUTPUT_OPEN_DRAIN_PULL_UP), exit);
            require_noerr(platform_gpio_output_high(&bt_control_pin_drivers[BT_PIN_RESET]), exit);
        }

        /* Initialise RX ring buffer */
        ring_buffer_init(&rx_ring_buffer, (uint8_t *) rx_data, sizeof(rx_data));

        /* Configure USART comms */
        require_noerr(platform_uart_init(bt_uart_driver, bt_uart_peripheral, &bt_uart_config, &rx_ring_buffer), exit);
//        require_noerr(platform_uart_init(bt_uart_driver, bt_uart_peripheral, &bt_uart_config, NULL), exit);

#ifdef  MICO_USE_BT_RESET_PIN
        /* Reset bluetooth chip. Delay momentarily. */
        require_noerr( platform_gpio_output_low( &bt_control_pins[BT_PIN_RESET] ), exit );
        mico_rtos_thread_msleep( 10 );
        require_noerr( platform_gpio_output_high( &bt_control_pins[BT_PIN_RESET] ), exit );
#endif

        /* Wait until the Bluetooth chip stabilizes.  */
        mico_rtos_thread_msleep(100);

        /* Bluetooth chip is ready. Pull host's RTS low */
        if (bt_uart_config.flow_control == FLOW_CONTROL_DISABLED) {
            /* Bluetooth chip is ready. Pull host's RTS low */
            require_noerr(platform_gpio_output_low(&bt_uart_pin_drivers[BT_PIN_UART_RTS]), exit);
        }

        bus_initialised = true;

        /* Wait for bluetooth chip to pull its RTS (host's CTS) low. From observation using CRO,
         * it takes the bluetooth chip > 170ms to pull its RTS low after CTS low */
        BT_BUS_WAIT_UNTIL_READY();

    }

exit:
    return kNoErr;
}

int bt_bus_deinit(void)
{
    require(bus_initialised, exit);

    if (bt_control_pins[BT_PIN_RESET] != NULL) {
        require_noerr(platform_gpio_output_low(&bt_uart_pin_drivers[BT_PIN_RESET]), exit);
    }

    require_noerr(platform_gpio_output_high(&bt_uart_pin_drivers[BT_PIN_UART_RTS]), exit); // RTS deasserted

    if (bt_control_pins[BT_PIN_POWER] != NULL) {
        require_noerr(platform_gpio_output_low(&bt_uart_pin_drivers[BT_PIN_POWER]),
                      exit); // Bluetooth chip regulator off
    }

    device_powered = false;

    /* Deinitialise UART */
    require_noerr(platform_uart_deinit(bt_uart_driver), exit);
    bus_initialised = false;

    return kNoErr;
exit:
    return kGeneralErr;
}

int bt_bus_transmit(const uint8_t *data_out, uint32_t size)
{
    IS_BUS_INITIALISED();

    BT_BUS_WAIT_UNTIL_READY();

    require_noerr(platform_uart_transmit_bytes(bt_uart_driver, data_out, size), exit);

exit:
    return kNoErr;
}

int bt_bus_receive(uint8_t *data_in, uint32_t size, uint32_t timeout_ms)
{
    IS_BUS_INITIALISED();
    return platform_uart_receive_bytes(bt_uart_driver, (void *) data_in, size, timeout_ms);
}

bool bt_bus_is_ready(void)
{
    return (bus_initialised == false) ? false : ((platform_gpio_input_get(&bt_uart_pin_drivers[BT_PIN_UART_CTS]) ==
                                                  true)
                                                 ? false : true);
}

int bt_bus_uart_reconifig_baud(uint32_t newBaudRate)
{
    OSStatus               err;
    platform_uart_config_t config;

    uint32_t               last_rx_size;

    if (!bus_initialised) {
        return kGeneralErr;
    }

    memcpy(&config, &bt_uart_config, sizeof(bt_uart_config));
    config.baud_rate = newBaudRate;
    last_rx_size = bt_uart_driver->rx_size;

    /* De-initialise UART */
    platform_uart_deinit(bt_uart_driver);

    /* Initialise RX ring buffer. */
    ring_buffer_init(&rx_ring_buffer, (uint8_t *)rx_data, sizeof(rx_data));

    /* Configure UART */
    err = platform_uart_init(bt_uart_driver, bt_uart_peripheral, &config, &rx_ring_buffer);
    if (err == kNoErr) {
        /* new baud rate value. */
        bt_uart_config.baud_rate = config.baud_rate;

        /* UART receive function may on the pending, but init will clear the rx size, recover here */
        bt_uart_driver->rx_size = last_rx_size;
    }

    return err;
}






