/*
 * Copyright 2013, Broadcom Corporation
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */
#pragma once

#include "wwd_constants.h"
#include "wwd_structures.h"
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 * @cond                       Macros
 ******************************************************/

static inline uint16_t htobe16(uint16_t v)
{
    return (uint16_t)(((v&0x00FF) << 8) | ((v&0xFF00)>>8));
}

static inline uint32_t htobe32(uint32_t v)
{
    return (uint32_t)(((v&0x000000FF) << 24) | ((v&0x0000FF00) << 8) | ((v&0x00FF0000) >> 8) | ((v&0xFF000000) >> 24));
}

#ifndef MIN
#define MIN(x,y)  ((x) < (y) ? (x) : (y))
#endif /* ifndef MIN */

#ifndef MAX
#define MAX(x,y)  ((x) > (y) ? (x) : (y))
#endif /* ifndef MAX */
//#define ROUND_UP(x,y)   (((x) + ((y)-1)) & ~((y)-1))
#define ROUND_UP(x,y)    ((x) % (y) ? (x) + (y)-((x)%(y)) : (x))
#define DIV_ROUND_UP(m, n)    (((m) + (n) - 1) / (n))

#define WICED_VERIFY(x)                               {wiced_result_t res = (x); if (res != WICED_SUCCESS){return res;}}

#define MEMCAT(destination, source, source_length)    (void*)((uint8_t*)memcpy((destination),(source),(source_length)) + (source_length))

#define MALLOC_OBJECT(name,object_type)               ((object_type*)malloc_named(name,sizeof(object_type)))

#define OFFSET(type, member)                          ((uint32_t)&((type *)0)->member)

#ifdef MALLOC_DEBUG
#include <stddef.h>
#include "wwd_rtos.h"
extern void* calloc_named                  ( const char* name, size_t nelems, size_t elemsize );
extern void * calloc_named_hideleak        ( const char* name, size_t nelem, size_t elsize );
extern void* malloc_named                  ( const char* name, size_t size );
extern void* malloc_named_hideleak         ( const char* name, size_t size );
extern void  malloc_set_name               ( const char* name );
extern void  malloc_leak_set_ignored       ( leak_check_scope_t global_flag );
extern void  malloc_leak_set_base          ( leak_check_scope_t global_flag );
extern void  malloc_leak_check             ( malloc_thread_handle thread, leak_check_scope_t global_flag );
extern void  malloc_transfer_to_curr_thread( void* block );
extern void  malloc_transfer_to_thread     ( void* block, malloc_thread_handle thread );
#else
#define calloc_named( name, nelems, elemsize) calloc ( nelems, elemsize )
#define calloc_named_hideleak( name, nelems, elemsize )  calloc ( nelems, elemsize )
#define realloc_named( name, ptr, size )      realloc( ptr, size )
#define malloc_named( name, size )            malloc ( size )
#define malloc_named_hideleak( name, size )   malloc ( size )
#define malloc_set_name( name )
#define malloc_leak_set_ignored( global_flag )
#define malloc_leak_set_base( global_flag )
#define malloc_leak_check( thread, global_flag )
#define malloc_transfer_to_curr_thread( block )
#define malloc_transfer_to_thread( block, thread )
#endif /* ifdef WICED_ENABLE_MALLOC_DEBUG */

/******************************************************
 *                    Constants
 ******************************************************/

#define WICED_NEVER_TIMEOUT   (0xFFFFFFFF)
#define WICED_WAIT_FOREVER    (0xFFFFFFFF)
#define WICED_NO_WAIT         (0)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    uint8_t*  buffer;
    uint32_t  size;
    uint32_t  head;
    uint32_t  tail;
} wiced_ring_buffer_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *                 Function Declarations
 * @endcond
 ******************************************************/
#if 0
extern uint32_t utoa(uint32_t value, char* output, uint8_t min_length, uint8_t max_length);
extern char     nibble_to_hexchar( uint8_t nibble );

/* Ring Buffer API */
wiced_result_t ring_buffer_init      ( wiced_ring_buffer_t* ring_buffer, uint8_t* buffer, uint32_t size );
wiced_result_t ring_buffer_deinit    ( wiced_ring_buffer_t* ring_buffer );
uint32_t       ring_buffer_write     ( wiced_ring_buffer_t* ring_buffer, const uint8_t* data, uint32_t data_length );
uint32_t       ring_buffer_used_space( wiced_ring_buffer_t* ring_buffer );
uint32_t       ring_buffer_free_space( wiced_ring_buffer_t* ring_buffer );
wiced_result_t ring_buffer_get_data  ( wiced_ring_buffer_t* ring_buffer, uint8_t** data, uint32_t* contiguous_bytes );
wiced_result_t ring_buffer_consume   ( wiced_ring_buffer_t* ring_buffer, uint32_t bytes_consumed );

/* Printing utilities */
extern void print_scan_result( wiced_scan_result_t* record );
#endif
#ifdef __cplusplus
} /*extern "C" */
#endif
