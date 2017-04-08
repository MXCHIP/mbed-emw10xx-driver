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
 *  Definitions for the FreeRTOS implementation of the Wiced RTOS
 *  abstraction layer.
 *
 */

#ifndef INCLUDED_WICED_RTOS_H_
#define INCLUDED_WICED_RTOS_H_

#include "internal_mico_rtos.h"


#define WICED_PRIORITY_TO_NATIVE_PRIORITY(priority)     (uint8_t)(priority)

#define WICED_END_OF_THREAD( thread )       mico_rtos_delete_thread(NULL)

#define malloc_get_current_thread( ) mico_rtos_get_current_thread() //xTaskGetCurrentTaskHandle()
typedef mico_thread_t malloc_thread_handle;

#define RTOS_USE_DYNAMIC_THREAD_STACK

#define TCP_CALLBACK_SIZE 800
#ifdef DEBUG
#define WICED_THREAD_STACK_SIZE        (632+TCP_CALLBACK_SIZE)   /* Stack checking requires a larger stack */
#else /* ifdef DEBUG */
#define WICED_THREAD_STACK_SIZE        (544+TCP_CALLBACK_SIZE)
#endif
/******************************************************
 *             Structures
 ******************************************************/

typedef mico_semaphore_t    host_semaphore_type_t;  /** MiCORTOS definition of a semaphore */
typedef mico_thread_t       host_thread_type_t;     /** MiCORTOS definition of a thread handle */
typedef mico_queue_t        host_queue_type_t;      /** MiCORTOS definition of a message queue */

/*@external@*/ extern void vApplicationMallocFailedHook( void );
/*@external@*/ extern void vApplicationIdleSleepHook( void );

#endif /* ifndef INCLUDED_WICED_RTOS_H_ */

