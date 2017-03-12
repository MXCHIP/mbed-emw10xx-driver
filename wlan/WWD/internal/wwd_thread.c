/*
 * Copyright 2013, Broadcom Corporation
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */

/** @file
 *  Allows thread safe access to the Wiced hardware bus
 *
 *  This file provides functions which allow multiple threads to use the Wiced hardware bus (SDIO or SPI)
 *  This is achieved by having a single thread (the "Wiced Thread") which queues messages to be sent, sending
 *  them sequentially, as well as receiving messages as they arrive.
 *
 *  Messages to be sent come from the wiced_send_sdpcm_common function in SDPCM.c .  The messages already
 *  contain SDPCM headers, but not any bus headers (GSPI), and are passed to the wiced_thread_send_data function.
 *  This function can be called from any thread.
 *
 *  Messages are received by way of a callback supplied by in SDPCM.c - wiced_process_sdpcm
 *  Received messages are delivered in the context of the Wiced Thread, so the callback function needs to avoid blocking.
 *
 *  It is also possible to use these functions without any operating system, by periodically calling the wiced_send_one_packet,
 *  wiced_receive_one_packet or wiced_poll_all functions
 *
 */

#include <string.h>
#include "wwd_assert.h"
#include "wwd_logging.h"
#include "wwd_poll.h"
#include "RTOS/wwd_rtos_interface.h"
#include "Network/wwd_buffer_interface.h"
#include "Network/wwd_network_interface.h"
#include "Platform/wwd_bus_interface.h"
#include "internal/wwd_thread.h"
#include "internal/wwd_sdpcm.h"
#include "internal/wwd_internal.h"
#include "internal/Bus_protocols/wwd_bus_protocol_interface.h"
//#include "rtos.h"

void wlan_notify_irq( void );


#define WICED_THREAD_POLL_TIMEOUT      (NEVER_TIMEOUT)

#ifdef RTOS_USE_STATIC_THREAD_STACK
static uint8_t wiced_thread_stack[WICED_THREAD_STACK_SIZE];
#define WICED_THREAD_STACK     wiced_thread_stack
#else
#ifdef RTOS_USE_DYNAMIC_THREAD_STACK
#define WICED_THREAD_STACK     NULL
#else
#error RTOS_USE_STATIC_THREAD_STACK or RTOS_USE_DYNAMIC_THREAD_STACK must be defined
#endif
#endif

#define MAX_POKE_TIMES 10
int wwd_thread_line=0, sdio_irq_=0;

#ifdef MXCHIP                
extern void wifi_reboot(void);
#endif

/******************************************************
 *             Static Variables
 ******************************************************/

static char                  wiced_thread_quit_flag = (char) 0;
static char                  wiced_inited           = (char) 0;
static host_thread_type_t    wiced_thread;
static host_semaphore_type_t wiced_transceive_semaphore;

static wiced_bool_t wiced_bus_interrupt = WICED_FALSE;
static wiced_bool_t wiced_need_poll = WICED_FALSE;

/******************************************************
 *             Static Function Prototypes
 ******************************************************/

static void wiced_thread_func( /*@unused@*/ uint32_t thread_input )   /*@globals wiced_thread, wiced_packet_send_queue_mutex, wiced_transceive_semaphore@*/ /*@modifies wiced_thread_quit_flag, wiced_inited@*/;

/******************************************************
 *             Global Functions
 ******************************************************/
#ifdef MXCHIP_LIBRARY
wiced_bool_t host_platform_is_sdio_int_asserted(void);

WEAK wiced_bool_t host_platform_is_sdio_int_asserted(void)
{
    return WICED_FALSE;
}
#endif
/** Initialises the Wiced Thread
 *
 * Initialises the Wiced thread, and its flags/semaphores,
 * then starts it running
 *
 * @return    WICED_SUCCESS : if initialisation succeeds
 *            WICED_ERROR   : otherwise
 */
wiced_result_t wiced_thread_init( void ) /*@globals undef wiced_thread, undef wiced_packet_send_queue_mutex, undef wiced_transceive_semaphore@*/ /*@modifies wiced_inited@*/
{
    if ( wiced_init_sdpcm( ) != WICED_SUCCESS )
    {
        WPRINT_WWD_ERROR(("Could not initialize SDPCM codec\r\n"));
        /*@-globstate@*/
        return WICED_ERROR;
        /*@+globstate@*/
    }

    /* Create the event flag which signals the Wiced thread needs to wake up */
    if ( host_rtos_init_semaphore( &wiced_transceive_semaphore ) != WICED_SUCCESS )
    {
        WPRINT_WWD_ERROR(("Could not initialize WICED thread semaphore\r\n"));
        /*@-globstate@*/
        return WICED_ERROR;
        /*@+globstate@*/
    }

    if ( WICED_SUCCESS != host_rtos_create_thread( &wiced_thread, wiced_thread_func, "wifiRXTX", WICED_THREAD_STACK, (uint32_t) WICED_THREAD_STACK_SIZE, (uint32_t) WICED_THREAD_PRIORITY ) )
    {
        /* could not start wiced main thread */
        WPRINT_WWD_ERROR(("Could not start WICED thread\r\n"));
        return WICED_ERROR;
    }

    wiced_inited = (char) 1;
    return WICED_SUCCESS;
}

/** Sends the first queued packet
 *
 * Checks the queue to determine if there is any packets waiting
 * to be sent. If there are, then it sends the first one.
 *
 * This function is normally used by the Wiced Thread, but can be
 * called periodically by systems which have no RTOS to ensure
 * packets get sent.
 *
 * @return    1 : packet was sent
 *            0 : no packet sent
 */
int8_t wiced_send_one_packet( void ) /*@modifies internalState, wiced_packet_send_queue_head, wiced_packet_send_queue_tail@*/
{
    wiced_buffer_t tmp_buf_hnd = NULL;
    int8_t ret = 0, get_pkt_ret;
#ifdef FAST_ROAM  	
#define STUCK_REPORT_TIME (2*1000)

	static int wifi_tx_stuck = 0;
	static int last_stuck_time = 0;
#endif

	get_pkt_ret = wiced_get_packet_to_send(&tmp_buf_hnd);
    if (get_pkt_ret == WICED_SUCCESS )
    {
#ifdef FAST_ROAM   
		last_stuck_time = 0;
    	if (wifi_tx_stuck == 1) {
			wifi_tx_stuck = 0;
			wifi_tx_state(0);
		}
#endif		
        /* Ensure the wlan backplane bus is up */
        if ( WWD_SUCCESS == wwd_ensure_wlan_bus_is_up() )
        {
            WPRINT_WWD_DEBUG(("Wcd:> Sending pkt 0x%08X\n\r", (unsigned int)tmp_buf_hnd ));
            if ( WICED_SUCCESS == wiced_bus_transfer_buffer( BUS_WRITE, WLAN_FUNCTION, 0, tmp_buf_hnd ) )
            {
                ret = (int8_t) 1;
            }
        }
        else
        {
            wiced_assert("Could not bring bus back up", 0 != 0 );
        }

        host_buffer_release( tmp_buf_hnd, WICED_NETWORK_TX );
    } else if (get_pkt_ret == WICED_ERROR) {
#ifdef FAST_ROAM  
		if (wifi_tx_stuck == 0) {
			if (last_stuck_time == 0) {
				last_stuck_time = host_rtos_get_time();
			} else if (last_stuck_time + STUCK_REPORT_TIME < host_rtos_get_time()) {
				wifi_tx_state(1);
				wifi_tx_stuck = 1;
				last_stuck_time = host_rtos_get_time();
			}
		}
#endif		
	}

    return ret;
}

/** Receives a packet if one is waiting
 *
 * Checks the wifi chip fifo to determine if there is any packets waiting
 * to be received. If there are, then it receives the first one, and calls
 * the callback wiced_process_sdpcm (in SDPCM.c).
 *
 * This function is normally used by the Wiced Thread, but can be
 * called periodically by systems which have no RTOS to ensure
 * packets get received properly.
 *
 * @return    1 : packet was received
 *            0 : no packet waiting
 */
int8_t wiced_receive_one_packet( void )
{
    /* Check if there is a packet ready to be received */
    wiced_buffer_t recv_buffer;
    wwd_thread_line = __LINE__;
    if ( wiced_read_frame( &recv_buffer ) == WICED_SUCCESS)
    {
        wwd_thread_line = __LINE__;
        if ( recv_buffer != NULL )
        {
            wwd_thread_line = __LINE__;
            WICED_LOG(("Wcd:< Rcvd pkt 0x%08X\n", (unsigned int)recv_buffer ));
            /* Send received buffer up to SDPCM layer */
            wiced_process_sdpcm( recv_buffer );
            wwd_thread_line = __LINE__;
        }
        return (int8_t) 1;
    }
    return 0;
}


#ifdef DHD_DEBUG
static int8_t wiced_receive_console( int time )
{
    char log[128];
    /* Check if there is a packet ready to be received */
    (void)time;
    if ( wiced_read_console(log, 128) == WICED_SUCCESS) {
        if (strlen(log) > 0)
            printf("%s\r\n", log);
        return (int8_t) 1;
    }
    
    return 0;
}
#endif

/** Sends and Receives all waiting packets
 *
 * Repeatedly calls wiced_send_one_packet and wiced_receive_one_packet
 * to send and receive packets, until there are no more packets waiting to
 * be transferred.
 *
 * This function is normally used by the Wiced Thread, but can be
 * called periodically by systems which have no RTOS to ensure
 * packets get send and received properly.
 *
 */
void wiced_poll_all( void ) /*@modifies internalState@*/
{
    do
    {
        /* Send queued outgoing packets */
        while ( wiced_send_one_packet( ) != 0 )
        {
            /* loop whist packets still queued */
        }
    } while ( wiced_receive_one_packet( ) != 0 );
}

/** Terminates the Wiced Thread
 *
 * Sets a flag then wakes the Wiced Thread to force it to terminate.
 *
 */
void wiced_thread_quit( void )
{
    /* signal main thread and wake it */
    wiced_thread_quit_flag = (char) 1;
    host_rtos_set_semaphore( &wiced_transceive_semaphore, WICED_FALSE );

    /* Wait for the Wiced thread to end */
    while ( wiced_inited != 0 )
    {
        host_rtos_delay_milliseconds( 1 );
    }
    host_rtos_delete_terminated_thread( &wiced_thread );
}

/**
 * Informs Wiced of an interrupt
 *
 * This function should be called from the SDIO/SPI interrupt function
 * and usually indicates newly received data is available.
 * It wakes the Wiced Thread, forcing it to check the send/receive
 *
 */
void wiced_platform_notify_irq( void )
{
    wiced_bus_interrupt = WICED_TRUE;
    /* just wake up the main thread and let it deal with the data */
    if ( wiced_inited == (char) 1 )
    {
        sdio_irq_ = __LINE__;
        host_rtos_set_semaphore( &wiced_transceive_semaphore, WICED_TRUE );
        
    }
}

void wlan_notify_irq( void )
{
    wiced_bus_interrupt = WICED_TRUE;
    /* just wake up the main thread and let it deal with the data */
    if ( wiced_inited == (char) 1 )
    {
        sdio_irq_ = __LINE__;
        host_rtos_set_semaphore( &wiced_transceive_semaphore, WICED_TRUE );
        
    }
}

void wiced_thread_notify( void )
{
    /* just wake up the main thread and let it deal with the data */
    if ( wiced_inited == (char) 1 )
    {
        host_rtos_set_semaphore( &wiced_transceive_semaphore, WICED_FALSE );
    }
}

/******************************************************
 *             Static Functions
 ******************************************************/

/** The Wiced Thread function
 *
 *  This is the main loop of the Wiced Thread.
 *  It simply calls wiced_poll_all to send/receive all waiting packets, then goes
 *  to sleep.  The sleep has a 100ms timeout, causing the send/receive queues to be
 *  checked 10 times per second in case an interrupt is missed.
 *  Once the quit flag has been set, flags/mutexes are cleaned up, and the function exits.
 *
 * @param thread_input  : unused parameter needed to match thread prototype.
 *
 */
static void wiced_thread_func( uint32_t /*@unused@*/thread_input )   /*@globals wiced_thread, wiced_packet_send_queue_mutex, wiced_transceive_semaphore, wiced_wlan_status@*/ /*@modifies wiced_thread_quit_flag, wiced_inited@*/
{
    uint32_t       int_status;
    int8_t         rx_status;
    int8_t         tx_status;
//    int            poke_num = 0;
    wiced_result_t result;
#ifdef DHD_DEBUG    
    static wiced_time_t console_time;
	wiced_time_t cur_console_time;
	wiced_time_t console_read_interval;

	(void)console_read_interval;
	console_read_interval = 1000;
	console_time = host_rtos_get_time();
#endif
    /*@-noeffect@*/
    UNUSED_PARAMETER(thread_input);
    /*@+noeffect@*/

    //wiced_bus_interrupt = WICED_FALSE;

    while ( wiced_thread_quit_flag != (char) 1 )
    {
#ifdef DHD_DEBUG    
        cur_console_time = host_rtos_get_time();
		
		if(cur_console_time > (console_time + console_read_interval)){
			console_time = cur_console_time;
			
			if(console_time > 10000)
				wiced_receive_console((int)console_time);
		}
#endif		
        wwd_thread_line = __LINE__;
        /* Check if we were woken by interrupt */
        if ( ( wiced_bus_interrupt == WICED_TRUE ) || WICED_BUS_USE_STATUS_REPORT_SCHEME )
        {
            wiced_bus_interrupt = WICED_FALSE;
            int_status = wiced_bus_process_interrupt();
            wwd_thread_line = __LINE__;
            /* Check if the interrupt indicated there is a packet to read */
            if ( WICED_BUS_PACKET_AVAILABLE_TO_READ(int_status) != 0)
            {
                /* Receive all available packets */
                do
                {
                    rx_status = wiced_receive_one_packet( );
                } while ( rx_status != 0 );
            }
        }
        wwd_thread_line = __LINE__;
        /* Send all queued packets */
        do
        {
            tx_status = wiced_send_one_packet( );
        }
        while (tx_status != 0);
        wwd_thread_line = __LINE__;
        /* Check if we have run out of bus credits */
        if ( wiced_get_available_bus_credits( ) == 0 )
        {
            
//            poke_num++;
//            if (poke_num > MAX_POKE_TIMES) {
//                poke_num = 0;
//#ifdef MXCHIP                
//                wifi_reboot();
//#endif
//            }
            /* Keep poking the WLAN until it gives us more credits */
            wiced_bus_poke_wlan( );
            result = host_rtos_get_semaphore( &wiced_transceive_semaphore, (uint32_t) 100, WICED_FALSE );
        }
        else
        {
            wwd_thread_line = __LINE__;
//            poke_num = 0;
            /* Put the bus to sleep and wait for something else to do */
            if ( wiced_wlan_status.keep_wlan_awake == 0 )
            {
                wwd_thread_line = __LINE__;
                result = wwd_allow_wlan_bus_to_sleep( );
                wwd_thread_line = __LINE__;
                wiced_assert( "Error setting wlan sleep", result == WICED_SUCCESS );
            }

#ifdef MXCHIP_LIBRARY   
            sdio_irq_ = __LINE__;
            wwd_thread_line = __LINE__;
            
            while ( host_platform_is_sdio_int_asserted() == WICED_FALSE ) {
                result = host_rtos_get_semaphore( &wiced_transceive_semaphore, (uint32_t) 100, WICED_FALSE );
                if (result == WICED_SUCCESS)
                    break;
            }
            wwd_thread_line = __LINE__;
            if( host_platform_is_sdio_int_asserted() == WICED_TRUE ){
                wwd_thread_line = __LINE__;
                result = WICED_SUCCESS;
                wiced_bus_interrupt = WICED_TRUE;
            }
#else
            wwd_thread_line = __LINE__;
            result = host_rtos_get_semaphore( &wiced_transceive_semaphore, (uint32_t) WICED_THREAD_POLL_TIMEOUT, WICED_FALSE );
            wwd_thread_line = __LINE__;
#endif 
            REFERENCE_DEBUG_ONLY_VARIABLE(result);
        } wiced_assert("Could not get wiced sleep semaphore\r\n", (result == WICED_SUCCESS)||(result == WICED_TIMEOUT) );
    }

    /* Reset the quit flag */
    wiced_thread_quit_flag = (char) 0;

    wwd_thread_line = __LINE__;
    wiced_quit_sdpcm( );
    wiced_inited = (char) 0;
    /* Delete the semaphore */
    host_rtos_deinit_semaphore( &wiced_transceive_semaphore );
    
    if ( WICED_SUCCESS != host_rtos_finish_thread( &wiced_thread ) )
    {
        WPRINT_WWD_DEBUG(("Could not close WICED thread\r\n"));
    }

    mico_rtos_delete_thread(NULL);
}

void wiced_setting_poll_flag(int flag)
{
    wiced_need_poll = flag;
}
