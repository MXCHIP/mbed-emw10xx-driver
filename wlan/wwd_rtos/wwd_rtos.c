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
 *  Implementation of wiced_rtos.c for FreeRTOS
 *
 *  This is the FreeRTOS implementation of the Wiced RTOS
 *  abstraction layer.
 *  It provides Wiced with standard ways of using threads,
 *  semaphores and time functions
 *
 */

#include "wwd_rtos.h"
#include <stdint.h>
#include "mico_common.h"
#include "internal_mico_rtos.h"
#include "RTOS/wwd_rtos_interface.h"

/******************************************************
 *             Constants
 ******************************************************/

/******************************************************
 *             Function definitions
 ******************************************************/

/**
 * Creates a new thread
 *
 * @param thread         : pointer to variable which will receive handle of created thread
 * @param entry_function : main thread function
 * @param name           : a string thread name used for a debugger
 *
 * @returns WICED_SUCCESS on success, WICED_ERROR otherwise
 */
OSStatus host_rtos_create_thread( /*@out@*/ host_thread_type_t* thread, void(*entry_function)( uint32_t ), const char* name, /*@null@*/ void* stack, uint32_t stack_size, uint32_t priority )
{
    UNUSED_PARAMETER( stack );   /* Unused in release mode */
    return mico_rtos_create_thread( thread, WICED_PRIORITY_TO_NATIVE_PRIORITY(priority), name, entry_function, stack_size, 0 );
}

OSStatus host_rtos_create_thread_with_arg( /*@out@*/ host_thread_type_t* thread, void(*entry_function)( uint32_t ), const char* name, /*@null@*/ void* stack, uint32_t stack_size, uint32_t priority, uint32_t arg )
{
    /*@-noeffect@*/
    UNUSED_PARAMETER( stack );   /* Unused in release mode */
    /*@+noeffect@*/
    return mico_rtos_create_thread( thread, WICED_PRIORITY_TO_NATIVE_PRIORITY(priority), name, entry_function, stack_size, (mico_thread_arg_t)arg );
}

/**
 * Terminates the current thread
 *
 * @param thread         : handle of the thread to terminate
 *
 * @returns WICED_SUCCESS on success, WICED_ERROR otherwise
 */
OSStatus host_rtos_finish_thread( host_thread_type_t* thread )
{
    return mico_rtos_delete_thread( thread );
}


/**
 * Blocks the current thread until the indicated thread is complete
 *
 * @param thread         : handle of the thread to terminate
 *
 * @returns WICED_SUCCESS on success, WICED_ERROR otherwise
 */
OSStatus host_rtos_join_thread( host_thread_type_t* thread )
{
    return mico_rtos_thread_join( thread );
}

/**
 * Deletes a terminated thread
 *
 * FreeRTOS does not require that another thread deletes any terminated thread
 *
 * @param thread         : handle of the terminated thread to delete
 *
 * @returns WICED_SUCCESS on success, WICED_ERROR otherwise
 */
OSStatus host_rtos_delete_terminated_thread( host_thread_type_t* thread )
{
    /*@-noeffect@*/
    UNUSED_PARAMETER( thread );
    /*@+noeffect@*/
    return kNoErr;
}


/**
 * Creates a semaphore
 *
 * In FreeRTOS a semaphore is represented with a counting semaphore
 *
 * @param semaphore         : pointer to variable which will receive handle of created semaphore
 *
 * @returns WICED_SUCCESS on success, WICED_ERROR otherwise
 */
OSStatus host_rtos_init_semaphore( /*@out@*/ host_semaphore_type_t* semaphore )
{
    return mico_rtos_init_semaphore( semaphore,  0x7fffffff );
}


/**
 * Gets a semaphore
 *
 * If value of semaphore is larger than zero, then the semaphore is decremented and function returns
 * Else If value of semaphore is zero, then current thread is suspended until semaphore is set.
 * Value of semaphore should never be below zero
 *
 * Must not be called from interrupt context, since it could block, and since an interrupt is not a
 * normal thread, so could cause RTOS problems if it tries to suspend it.
 *
 * @param semaphore       : Pointer to variable which will receive handle of created semaphore
 * @param timeout_ms      : Maximum period to block for. Can be passed NEVER_TIMEOUT to request no timeout
 * @param will_set_in_isr : True if the semaphore will be set in an ISR. Currently only used for NoOS/NoNS
 *
 */

OSStatus host_rtos_get_semaphore( host_semaphore_type_t* semaphore, uint32_t timeout_ms, /*@unused@*/ wiced_bool_t will_set_in_isr )
{
    /*@-noeffect@*/
    UNUSED_PARAMETER( will_set_in_isr );
    /*@+noeffect@*/

    return mico_rtos_get_semaphore( semaphore, timeout_ms );
}


/**
 * Sets a semaphore
 *
 * If any threads are waiting on the semaphore, the first thread is resumed
 * Else increment semaphore.
 *
 * Can be called from interrupt context, so must be able to handle resuming other
 * threads from interrupt context.
 *
 * @param semaphore       : Pointer to variable which will receive handle of created semaphore
 * @param called_from_ISR : Value of WICED_TRUE indicates calling from interrupt context
 *                          Value of WICED_FALSE indicates calling from normal thread context
 *
 * @return wiced_result_t : WICED_SUCCESS if semaphore was successfully set
 *                        : WICED_ERROR if an error occurred
 *
 */

OSStatus host_rtos_set_semaphore( host_semaphore_type_t* semaphore, wiced_bool_t called_from_ISR )
{
    /*@-noeffect@*/
    UNUSED_PARAMETER( called_from_ISR );
    /*@+noeffect@*/

    return mico_rtos_set_semaphore( semaphore );
}


/**
 * Deletes a semaphore
 *
 * WICED uses this function to delete a semaphore.
 *
 * @param semaphore         : Pointer to the semaphore handle
 *
 * @return wiced_result_t : WICED_SUCCESS if semaphore was successfully deleted
 *                        : WICED_ERROR if an error occurred
 *
 */

OSStatus host_rtos_deinit_semaphore( host_semaphore_type_t* semaphore )
{
    return mico_rtos_deinit_semaphore( semaphore );
}


/**
 * Gets time in milliseconds since RTOS start
 *
 * @Note: since this is only 32 bits, it will roll over every 49 days, 17 hours.
 *
 * @returns Time in milliseconds since RTOS started.
 */
mico_time_t host_rtos_get_time( void )  /*@modifies internalState@*/
{
    return mico_rtos_get_time();
}


/**
 * Delay for a number of milliseconds
 *
 * Processing of this function depends on the minimum sleep
 * time resolution of the RTOS.
 * The current thread sleeps for the longest period possible which
 * is less than the delay required, then makes up the difference
 * with a tight loop
 *
 * @return wiced_result_t : WICED_SUCCESS if delay was successful
 *                        : WICED_ERROR if an error occurred
 *
 */
OSStatus host_rtos_delay_milliseconds( uint32_t num_ms )
{
   return mico_rtos_delay_milliseconds( num_ms );
}


OSStatus host_rtos_init_queue( host_queue_type_t* queue, void* buffer, uint32_t buffer_size, uint32_t message_size )
{
    UNUSED_PARAMETER(buffer);
    return mico_rtos_init_queue( queue, NULL, message_size, buffer_size / message_size );
}


OSStatus host_rtos_push_to_queue( host_queue_type_t* queue, void* message, uint32_t timeout_ms )
{
    return mico_rtos_push_to_queue( queue, message, timeout_ms );
}


OSStatus host_rtos_pop_from_queue( host_queue_type_t* queue, void* message, uint32_t timeout_ms )
{
    return mico_rtos_pop_from_queue( queue, message, timeout_ms );
}

OSStatus host_rtos_deinit_queue( host_queue_type_t* queue )
{
    return mico_rtos_deinit_queue( queue );
}
