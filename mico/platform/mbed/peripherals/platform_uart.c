/**
 ******************************************************************************
 * @file    platform_uart.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This file provide UART driver functions.
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#include "platform_peripheral.h"

/******************************************************
*                    Constants
******************************************************/

#define DMA_INTERRUPT_FLAGS  ( DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE )

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
static platform_uart_driver_t *uart_async_driver = NULL;

/******************************************************
*        Static Function Declarations
******************************************************/
/* Interrupt service functions - called from interrupt vector table */
void platform_uart_irq(uint32_t id, SerialIrq event);

static OSStatus transmit_bytes(platform_uart_driver_t *driver, const void *data, uint32_t size, uint32_t timeout);

static OSStatus receive_bytes(platform_uart_driver_t *driver, void *data, uint32_t size, uint32_t timeout);

/******************************************************
*               Function Definitions
******************************************************/

OSStatus platform_uart_init(platform_uart_driver_t *driver, const platform_uart_t *peripheral,
                            const platform_uart_config_t *config, ring_buffer_t *optional_ring_buffer)
{
    OSStatus err = kNoErr;
    int wordlen, stopbit;
    SerialParity parity;

    platform_mcu_powersave_disable();

    require_action_quiet((driver != NULL) && (peripheral != NULL) && (config != NULL), exit, err = kParamErr);
    require_action_quiet((optional_ring_buffer == NULL) ||
                         ((optional_ring_buffer->buffer != NULL) && (optional_ring_buffer->size != 0)), exit,
                         err = kParamErr);
    driver->rx_size = 0;
    driver->tx_size = 0;
    driver->last_transmit_result = kNoErr;
    driver->last_receive_result = kNoErr;
    driver->is_recv_over_flow = false;
    driver->is_receving = false;
    driver->FlowControl = FlowControlNone;

    mico_rtos_init_semaphore(&driver->rx_complete, 1);
    mico_rtos_init_mutex(&driver->tx_mutex);

    switch (config->data_width) {
        case DATA_WIDTH_7BIT  :
            wordlen = 7;
            break;
        case DATA_WIDTH_8BIT:
            wordlen = 8;
            break;
        default:
            err = kParamErr;
            goto exit;
    }

    switch (config->parity) {
        case NO_PARITY:
            parity = ParityNone;
            break;

        case EVEN_PARITY:
            parity = ParityEven;
            break;

        case ODD_PARITY:
            parity = ParityOdd;
            break;
        default:
            err = kParamErr;
            goto exit;
    }

    switch (config->stop_bits) {
        case STOP_BITS_1:
            stopbit = 1;
            break;
        case STOP_BITS_2:
            stopbit = 2;
            break;
        default:
            err = kParamErr;
            goto exit;
    }

    switch (config->flow_control) {
        case FLOW_CONTROL_DISABLED:
            driver->FlowControl = FlowControlNone;
            break;
        case FLOW_CONTROL_RTS:
            driver->FlowControl = FlowControlRTS;
            break;
        case FLOW_CONTROL_CTS:
            driver->FlowControl = FlowControlCTS;
            break;
        case FLOW_CONTROL_CTS_RTS:
            driver->FlowControl = FlowControlRTSCTS;
            break;
        default:
            err = kParamErr;
            goto exit;
    }
    serial_init(&(driver->serial_obj), peripheral->mbed_tx_pin, peripheral->mbed_rx_pin);
    serial_baud(&driver->serial_obj, config->baud_rate);
    serial_format(&driver->serial_obj, wordlen, parity, stopbit);
#if DEVICE_SERIAL_FC
    if (driver->FlowControl != FlowControlNone) {
        serial_set_flow_control(&driver->serial_obj,
                                (FlowControl) driver->FlowControl,
                                peripheral->mbed_rts_pin,
                                peripheral->mbed_cts_pin);
    }
#endif

    if (optional_ring_buffer != NULL) {
        /* Note that the ring_buffer should've been initialised first */
        driver->rx_buffer = optional_ring_buffer;
        driver->rx_size = 0;
        /* Enable and set IRQ Handler to start receiving data from UART immediately. */
        serial_irq_handler(&driver->serial_obj, platform_uart_irq, (uint32_t) driver);
        serial_irq_set(&driver->serial_obj, RxIrq, MICO_TRUE);
    } else {
        /* Receiving data from UART by calling receive_bytes() */
        return kNoErr;
    }
exit:
    platform_mcu_powersave_enable();
    return err;
}

OSStatus platform_uart_deinit(platform_uart_driver_t *driver)
{
    OSStatus err = kNoErr;

    platform_mcu_powersave_disable();

    require_action_quiet((driver != NULL), exit, err = kParamErr);

    serial_free(&driver->serial_obj);

    mico_rtos_deinit_semaphore(&driver->rx_complete);
    mico_rtos_deinit_mutex(&driver->tx_mutex);

    driver->rx_size = 0;
    driver->tx_size = 0;
    driver->last_transmit_result = kNoErr;
    driver->last_receive_result = kNoErr;
    driver->FlowControl = FlowControlNone;
    driver->is_recv_over_flow = false;
    driver->is_receving = false;

exit:
    platform_mcu_powersave_enable();
    return err;
}

OSStatus platform_uart_transmit_bytes(platform_uart_driver_t *driver, const uint8_t *data_out, uint32_t size)
{
    OSStatus err = kNoErr;

    platform_mcu_powersave_disable();

    mico_rtos_lock_mutex(&driver->tx_mutex);

    /* Send data to UART (blocking call...) */
    err = transmit_bytes(driver, data_out, size, MICO_WAIT_FOREVER);

    mico_rtos_unlock_mutex(&driver->tx_mutex);
    platform_mcu_powersave_enable();
    return err;
}

OSStatus platform_uart_receive_bytes(platform_uart_driver_t *driver, uint8_t *data_in, uint32_t expected_data_size,
                                     uint32_t timeout_ms)
{
    OSStatus err = kNoErr;

    require_action_quiet((driver != NULL)
                         && (data_in != NULL)
                         && (expected_data_size != 0)
                         && (expected_data_size <= driver->rx_buffer->size),
                         exit,
                         err = kParamErr);

    if (driver->is_receving) {
        /* There is a procedure which is receiving data. */
        return kInProgressErr;
    } else {
        driver->is_receving = true;
    }

    mico_rtos_get_semaphore(&driver->rx_complete, 0);

    if (driver->rx_buffer != NULL) {
        mico_rtos_enter_critical();
        if (ring_buffer_used_space(driver->rx_buffer) < expected_data_size) {
            /* Ring buffer does not have enough data to meet the needs of users. */
            if (timeout_ms > 0) {
                driver->rx_size = expected_data_size;
                mico_rtos_exit_critical();

                err = mico_rtos_get_semaphore(&driver->rx_complete, timeout_ms);

                mico_rtos_enter_critical();
                require_action_quiet(err == kNoErr, exit_with_critical, driver->rx_size = 0);
            } else {
                require_action_quiet(0, exit_with_critical, err = kGeneralErr);
            }
        }

        /* Copy enough data to user space */
        ring_buffer_read(driver->rx_buffer, data_in, expected_data_size, &expected_data_size);

        /* RX Ring buffer overrun ? */
        if ((driver->FlowControl == FlowControlRTSCTS || driver->FlowControl == FlowControlRTS)
                && (driver->is_recv_over_flow == true)) {
            /* Re-enable RX Interrupt for incomming data. */
            driver->is_recv_over_flow = false;
            serial_irq_set(&driver->serial_obj, RxIrq, MICO_TRUE);
        }

exit_with_critical:
        mico_rtos_exit_critical();
    } else {
        err = receive_bytes(driver, data_in, expected_data_size, timeout_ms);
    }

exit:
    driver->is_receving = false;
    return err;
}

uint32_t platform_uart_get_length_in_buffer(platform_uart_driver_t *driver)
{
    return ring_buffer_used_space(driver->rx_buffer);
}

/******************************************************
*            Interrupt Service Routines
******************************************************/

void platform_uart_irq(uint32_t id, SerialIrq event)
{
    uint8_t recv_byte;
    platform_uart_driver_t *driver = (platform_uart_driver_t *) id;

    if (event == RxIrq) {
        if (ring_buffer_is_full(driver->rx_buffer) == 0) {
            recv_byte = (uint8_t) serial_getc(&driver->serial_obj);
            ring_buffer_write(driver->rx_buffer, &recv_byte, 1);
            if ((driver->rx_size > 0) && (ring_buffer_used_space(driver->rx_buffer) >= driver->rx_size)) {
                mico_rtos_set_semaphore(&driver->rx_complete);
                driver->rx_size = 0;
            }
        } else {
            if (driver->FlowControl == FlowControlRTSCTS || driver->FlowControl == FlowControlRTS) {
                serial_irq_set(&driver->serial_obj, RxIrq, MICO_FALSE);
                driver->is_recv_over_flow = true;
            } else {
                serial_getc(&driver->serial_obj);
            }
        }
    }
}

/* Handle for asynchronous RX transfer event and used with @receive_bytes.  */
static void platform_uart_event_handler(void)
{
    if (uart_async_driver == NULL) return;

    int event = serial_irq_handler_asynch(&uart_async_driver->serial_obj);
    int rx_event = event & SERIAL_EVENT_RX_MASK;

    if (rx_event) {
        if (rx_event & SERIAL_EVENT_RX_COMPLETE) {
            // printf("%s: SERIAL_EVENT_RX_COMPLETE\r\n", __FUNCTION__);
            uart_async_driver->last_receive_result = kNoErr;
        } else {
            if (rx_event & SERIAL_EVENT_RX_OVERRUN_ERROR) {
                printf("rx_event: RX_OVERRUN_ERROR\r\n");
            }
            if (rx_event & SERIAL_EVENT_RX_FRAMING_ERROR) {
                printf("rx_event: RX_FRAMING_ERROR\r\n");
            }
            if (rx_event & SERIAL_EVENT_RX_PARITY_ERROR) {
                printf("rx_event: RX_PARITY_ERROR\r\n");
            }
            if (rx_event & SERIAL_EVENT_RX_OVERFLOW) {
                printf("rx_event: RX_OVERFLOW\r\n");
            }
            uart_async_driver->last_receive_result = kOverrunErr;
        }
        mico_rtos_set_semaphore(&uart_async_driver->rx_complete);
    }
}

/* Receive data form UART without local ring buffer */
static OSStatus receive_bytes(platform_uart_driver_t *driver, void *data, uint32_t size, uint32_t timeout)
{
    uint32_t rx_event = SERIAL_EVENT_RX_ALL & (~SERIAL_EVENT_RX_CHARACTER_MATCH);

    if (serial_rx_active(&driver->serial_obj)) {
        return kInProgressErr;
    }

    /* Start to receive data. */
    uart_async_driver = driver;
    serial_rx_asynch(&driver->serial_obj, data, size, 8,
                     (uint32_t) platform_uart_event_handler,
                     rx_event,
                     SERIAL_RESERVED_CHAR_MATCH,
                     DMA_USAGE_NEVER);

    OSStatus err = mico_rtos_get_semaphore(&driver->rx_complete, timeout);
    if (err != kNoErr) {
        printf("%s: get semaphore failed: %d\r\n", __FUNCTION__, err);
        serial_rx_abort_asynch(&driver->serial_obj);
    } else {
        err = driver->last_receive_result;
    }
    uart_async_driver = NULL;
    return err;
}

/* Send a stream packet to UART by blocking call. */
static OSStatus transmit_bytes(platform_uart_driver_t *driver, const void *data, uint32_t size, uint32_t timeout)
{
    for (uint32_t i = 0; i < size; i++) {
        serial_putc(&driver->serial_obj, *((const uint8_t *)data + i));
    }

    return kNoErr;
}
