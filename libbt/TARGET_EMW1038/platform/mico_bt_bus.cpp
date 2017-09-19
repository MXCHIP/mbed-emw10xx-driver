//
// Created by zhangjian-mico on 2017/9/19 0019.
//

#include "mico.h"
#include "mbed.h"

#include "platform_bluetooth.h"

/**
 * BT Serial Impletation
 */
static OSStatus platform_serial_init();
static OSStatus platform_serial_deinit();
static OSStatus platform_serial_send_bytes(const uint8_t *data, uint32_t size);
static OSStatus platform_serial_read_bytes(uint8_t *data, uint32_t size, uint32_t timeout);
static OSStatus platform_serial_set_baudrate(uint32_t baudrate);

extern "C" int bt_bus_init(void);
extern "C" int bt_bus_deinit(void);
extern "C" int bt_bus_transmit(const uint8_t *data_out, uint32_t size);
extern "C" int bt_bus_receive(uint8_t *data_in, uint32_t size, uint32_t timeout_ms);
extern "C" int bt_bus_uart_reset(void);
extern "C" int bt_bus_uart_reconifig_baud(uint32_t baud);
extern "C" bool bt_bus_is_ready(void);

/* Macro for checking of bus is initialised */
#define IS_BUS_INITIALISED() \
do \
{ \
    if ( !bus_initialised ) \
    { \
        printf( "bus uninitialised" ); \
        return kGeneralErr; \
    } \
}while ( 0 )

/* Macro for checking if bus is ready */
#define BT_BUS_IS_READY() \
do \
{ \
    if ( !bt_bus_is_ready( ) ) \
    { \
        printf( "bus not ready" ) \
        return kGeneralErr; \
    } \
}while ( 0 )

/* Macro for waiting until bus is ready */
#define BT_BUS_WAIT_UNTIL_READY() \
do \
{ \
    while ( !bt_bus_is_ready( ) ) \
    { \
        mico_rtos_thread_msleep( 10 ); \
    } \
} while ( 0 )


static volatile bool bus_initialised = false;
static volatile bool device_powered = false;

extern "C" int bt_bus_init()
{
    OSStatus err = kNoErr;

    if (!bus_initialised) {
        if (bt_control_pins[BT_PIN_HOST_WAKE] != NULL) {
            err = platform_gpio_init(&bt_control_pin_drivers[BT_PIN_HOST_WAKE],
                                     bt_control_pins[BT_PIN_HOST_WAKE],
                                     INPUT_HIGH_IMPEDANCE);
            require_noerr(err, exit);
        }

        if (bt_control_pins[BT_PIN_DEVICE_WAKE] != NULL) {
            err = platform_gpio_init(&bt_control_pin_drivers[BT_PIN_DEVICE_WAKE],
                                     bt_control_pins[BT_PIN_DEVICE_WAKE],
                                     OUTPUT_OPEN_DRAIN_PULL_UP);
            require_noerr(err, exit);

            err = platform_gpio_output_high(&bt_control_pin_drivers[BT_PIN_DEVICE_WAKE]);
            require_noerr(err, exit);


            mico_rtos_thread_msleep(100);
        }

        /* Configure Reg Enable pin to output. Set to HIGH */
        if (bt_control_pins[BT_PIN_POWER] != NULL) {
            err = platform_gpio_init(&bt_control_pin_drivers[BT_PIN_POWER], bt_control_pins[BT_PIN_POWER],
                                     OUTPUT_OPEN_DRAIN_PULL_UP);
            require_noerr(err, exit);

            err = platform_gpio_output_high(&bt_control_pin_drivers[BT_PIN_POWER]);
            require_noerr(err, exit);
        }
        device_powered = true;

        if (bt_uart_config.flow_control == FLOW_CONTROL_DISABLED) {
            /* Configure RTS pin to output. Set to HIGH */
            err = platform_gpio_init(&bt_uart_pin_drivers[BT_PIN_UART_RTS], bt_uart_pins[BT_PIN_UART_RTS],
                                     OUTPUT_OPEN_DRAIN_PULL_UP);
            require_noerr(err, exit);

            // William, working wrong if set high, so donot use FLOW_CONTROL_DISABLED in 43438A1
            err = platform_gpio_output_high(&bt_uart_pin_drivers[BT_PIN_UART_RTS]);
            require_noerr(err, exit);

            /* Configure CTS pin to input pull-up */
            err = platform_gpio_init(&bt_uart_pin_drivers[BT_PIN_UART_CTS], bt_uart_pins[BT_PIN_UART_CTS],
                                     INPUT_PULL_UP);
            require_noerr(err, exit);
        }

        /* Configure Reset pin to output. Set to HIGH */
        if (bt_control_pins[BT_PIN_RESET] != NULL) {
            err = platform_gpio_init(&bt_control_pin_drivers[BT_PIN_RESET], bt_control_pins[BT_PIN_RESET],
                                     OUTPUT_OPEN_DRAIN_PULL_UP);
            require_noerr(err, exit);

            err = platform_gpio_output_high(&bt_control_pin_drivers[BT_PIN_RESET]);
            require_noerr(err, exit);
        }

        /* Configure UART */
        err = platform_serial_init();
        require_noerr(err, exit);

        /* Wait until the Bluetooth chip stabilizes. */
        mico_rtos_thread_msleep(100);

        /* Bluetoot chip is ready. Pull host's RTS low */
        if (bt_uart_config.flow_control == FLOW_CONTROL_DISABLED) {
            /* Bluetooth chip is ready, pull host's RTS low */
            require_noerr(platform_gpio_output_low(&bt_uart_pin_drivers[BT_PIN_UART_RTS]), exit);
        }

        bus_initialised = true;

        /*
         * Wait for bluetoot chip to pull its RTS (host's CTS) low. From observation using CRO,
         * It takes the bluetooth chip > 170ms to pull its RTS low after CTS low.
         */
        BT_BUS_WAIT_UNTIL_READY();
    }

exit:
    return err;
}

extern "C" int bt_bus_deinit()
{
    require(bus_initialised, exit);

    if (bt_control_pins[BT_PIN_RESET] != NULL) {
        require_noerr(platform_gpio_output_low(&bt_uart_pin_drivers[BT_PIN_RESET]), exit);
    }

    require_noerr(platform_gpio_output_high(&bt_uart_pin_drivers[BT_PIN_UART_RTS]), exit);

    if (bt_control_pins[BT_PIN_POWER] != NULL) {
        require_noerr(platform_gpio_output_low(&bt_uart_pin_drivers[BT_PIN_POWER]), exit);
    }

    device_powered = false;

    /* De-initialise UART */
    platform_serial_deinit();

    bus_initialised = false;
    return kNoErr;

exit:
    return kGeneralErr;
}

extern "C" int bt_bus_transmit(const uint8_t *data_out, uint32_t size)
{
    IS_BUS_INITIALISED();

    BT_BUS_WAIT_UNTIL_READY();

    require_noerr(platform_serial_send_bytes(data_out, size), exit);

exit:
    return kNoErr;
}

extern "C" int bt_bus_receive(uint8_t *data_in, uint32_t size, uint32_t timeout_ms)
{
    IS_BUS_INITIALISED();

    return platform_serial_read_bytes(data_in, size, timeout_ms);
}

extern "C" bool bt_bus_is_ready()
{
    return bus_initialised != 0 && !platform_gpio_input_get(&bt_uart_pin_drivers[BT_PIN_UART_CTS]);
}

extern "C" int bt_bus_uart_reconifig_baud(uint32_t newBaudRate)
{
    if (!bus_initialised) {
        return kGeneralErr;
    }

    return platform_serial_set_baudrate(newBaudRate);
}

/**
 * BT Serial implementation
 */

static Serial *btSerial = NULL;
static volatile bool btInitialised = false;

static OSStatus platform_serial_init()
{
    OSStatus err = kNoErr;

    if (btInitialised) {
        return kAlreadyInitializedErr;
    }

    /* Configure UART */
    bt_uart_driver->last_receive_result = kNoErr;
    bt_uart_driver->last_transmit_result = kNoErr;
    mico_rtos_init_semaphore(&bt_uart_driver->rx_complete, 1);
    mico_rtos_init_semaphore(&bt_uart_driver->tx_complete, 1);

    btSerial = new Serial(bt_uart_peripheral->mbed_tx_pin,
                          bt_uart_peripheral->mbed_rx_pin,
                          bt_uart_config.baud_rate);
    require_action(btSerial, exit, err = kNoResourcesErr);
    btSerial->format();
    btSerial->set_flow_control(Serial::RTSCTS,
                               bt_uart_peripheral->mbed_rts_pin,
                               bt_uart_peripheral->mbed_cts_pin);

    btInitialised = true;

exit:
    return err;
}

static OSStatus platform_serial_deinit()
{
    if (btInitialised) {
        mico_rtos_deinit_semaphore(&bt_uart_driver->rx_complete);
        mico_rtos_deinit_semaphore(&bt_uart_driver->tx_complete);

        delete btSerial;
        btSerial = NULL;
        btInitialised = false;
    }
    return kNoErr;
}

static void platform_serial_tx_complete_event_handler(int id)
{
    bt_uart_driver->last_transmit_result = kNoErr;
    mico_rtos_set_semaphore(&bt_uart_driver->tx_complete);
}

static void platform_serial_rx_complete_event_handler(int id)
{
    if (id & SERIAL_EVENT_RX_COMPLETE) {
        bt_uart_driver->last_receive_result = kNoErr;
    } else {
        bt_uart_driver->last_receive_result = kOverrunErr;
    }
    mico_rtos_set_semaphore(&bt_uart_driver->rx_complete);
}

static OSStatus platform_serial_send_bytes(const uint8_t *data, uint32_t size)
{
    if (!btInitialised) return kNotInitializedErr;

    mico_rtos_get_semaphore(&bt_uart_driver->tx_complete, 0);

    while (btSerial->write(data, size,
                           platform_serial_tx_complete_event_handler,
                           SERIAL_EVENT_TX_COMPLETE) != 0) {
        mico_rtos_thread_msleep(10);
    }

    mico_rtos_get_semaphore(&bt_uart_driver->tx_complete, MICO_NEVER_TIMEOUT);
    return bt_uart_driver->last_transmit_result;
}

static OSStatus platform_serial_read_bytes(uint8_t *data, uint32_t size, uint32_t timeout)
{
    uint32_t tmpTimeout = 0;

    if (!btInitialised) return kNotInitializedErr;

    mico_rtos_get_semaphore(&bt_uart_driver->rx_complete, 0);

    while (btSerial->read(data, size,
                          platform_serial_rx_complete_event_handler,
                          SERIAL_EVENT_RX_ALL & (~SERIAL_EVENT_RX_CHARACTER_MATCH)) != 0) {
        if (tmpTimeout >= timeout) {
            return kTimeoutErr;
        }
        mico_rtos_thread_msleep(10);
        tmpTimeout += 10;
    }

    mico_rtos_get_semaphore(&bt_uart_driver->rx_complete, timeout);
    return bt_uart_driver->last_receive_result;
}

static OSStatus platform_serial_set_baudrate(uint32_t baudrate)
{
    if (!btInitialised) return kNotInitializedErr;

    btSerial->baud(baudrate);
    bt_uart_config.baud_rate = baudrate;
    return kNoErr;
}

