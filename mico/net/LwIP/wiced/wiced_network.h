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

//#include "FreeRTOS.h"
//#include "task.h"
#include "mico_rtos.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
//#include "tls_types.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/sockets.h"

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define MAX_TCP_PAYLOAD_SIZE    ( WICED_PAYLOAD_MTU - TCP_HLEN - IP_HLEN - WICED_PHYSICAL_HEADER )
#define MAX_UDP_PAYLOAD_SIZE    ( WICED_PAYLOAD_MTU - UDP_HLEN - IP_HLEN - WICED_PHYSICAL_HEADER )
#define MAX_IP_PAYLOAD_SIZE     ( WICED_PAYLOAD_MTU - IP_HLEN - WICED_PHYSICAL_HEADER )


#define IP_STACK_SIZE               (1024*4)
#define ARP_CACHE_SIZE              (3*52)
#define DHCP_STACK_SIZE             (1024)

#define WICED_ANY_PORT              (0)

#define wiced_packet_pools          (NULL)


#define IP_HANDLE(x)   (wiced_ip_handle[(x)&1])


/******************************************************
 *                   Enumerations
 ******************************************************/




/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct netbuf       wiced_packet_t;
typedef wiced_result_t (*wiced_socket_callback_t)( void* socket );


/******************************************************
 *                    Structures
 ******************************************************/
#if 0
typedef struct
{
    wiced_tls_context_type_t context_type;
    wiced_tls_context_t      context;
    wiced_tls_session_t      session;
    wiced_packet_t*          temp_packet;
} wiced_tls_simple_context_t;


typedef struct
{
    wiced_tls_context_type_t context_type;
    wiced_tls_context_t      context;
    wiced_tls_session_t      session;
    wiced_packet_t*          temp_packet;
    wiced_tls_certificate_t  certificate;
    wiced_tls_key_t          key;
} wiced_tls_advanced_context_t;
#endif

typedef struct
{
    int                         socket;
    struct netconn*             conn_handler;
    struct netconn*             accept_handler;
    ip_addr_t                   local_ip_addr;
    wiced_bool_t                is_bound;
    int                         interface;
    //wiced_tls_simple_context_t* tls_context;
    wiced_bool_t                context_malloced;
    wiced_socket_callback_t     callbacks[3];

} wiced_tcp_socket_t;

typedef wiced_tcp_socket_t  wiced_udp_socket_t;

#if 0
typedef struct
{
    wiced_tcp_socket_type_t  type;
    int                      socket;
    wiced_tls_context_t      context;
    wiced_tls_session_t      session;
    wiced_tls_certificate_t* certificate;
    wiced_tls_key_t*         key;
} wiced_tls_socket_t;
#endif
/******************************************************
 *                 Global Variables
 ******************************************************/

/* WiHi objects.
 * Note: These objects are for internal use only!
 */
extern mico_thread_t   wiced_thread_handle;
extern struct netif    wiced_ip_handle[2];
extern struct dhcp     wiced_dhcp_handle;

/******************************************************
 *               Function Declarations
 ******************************************************/
char *gethostname( char *name, int len );

char *sethostname( char *name );
struct netif *get_netif_by_interface(wiced_interface_t interface);

#ifdef __cplusplus
} /*extern "C" */
#endif
