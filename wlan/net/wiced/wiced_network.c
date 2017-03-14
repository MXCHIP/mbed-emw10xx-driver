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
 */

#include <string.h>
#include "wiced.h"
#include "wiced_network.h"
//#include "wiced_utilities.h"
#include "wwd_debug.h"
#include "wwd_assert.h"
#include "lwip/init.h"
#include "lwip/tcpip.h"
#include "netif/etharp.h"
#include "lwip/dhcp.h"
#include "wwd_management.h"
#include "wwd_network.h"
#include "dhcp_server.h"
#include "wifimgr.h"
//#include "internal/wiced_internal_api.h"
//#include "bootloader_app.h"
#include "lwip/dns.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define TIME_WAIT_TCP_SOCKET_DELAY (400)

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
 *                 Static Variables
 ******************************************************/

/* Network objects */
struct netif        wiced_ip_handle[2];
struct dhcp         wiced_dhcp_handle;
static wiced_bool_t        wiced_using_dhcp;
static wiced_dhcp_server_t internal_dhcp_server;

wiced_mutex_t send_interface_mutex;
static wiced_mutex_t link_subscribe_mutex;

const wiced_ip_address_t INITIALISER_IPV4_ADDRESS( wiced_ip_broadcast, 0xFFFFFFFF );
static wiced_network_link_callback_t link_up_callbacks[WICED_MAXIMUM_LINK_CALLBACK_SUBSCRIPTIONS];
static wiced_network_link_callback_t link_down_callbacks[WICED_MAXIMUM_LINK_CALLBACK_SUBSCRIPTIONS];

static wiced_bool_t ip_networking_up[2];
static char hostname[32]="MXCHIP";
static wiced_bool_t dhcp_interrupted; // yhb added, use may push wps/easylink to stop dhcp

/******************************************************
 *               Function Declarations
 ******************************************************/

extern int ReallocIP(void);
#ifdef MXCHIPAPP
void delay_reload(void );
extern void send_mdns_up(void);
#endif
static void tcpip_init_done( void* arg );

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t wiced_network_init( void )
{
    wiced_semaphore_t lwip_done_sema = NULL;

    /* Initialize the LwIP system.  */
    WPRINT_NETWORK_INFO(("Initialising LwIP " "\r\n"));

    ip_networking_up[0] = WICED_FALSE;
    ip_networking_up[1] = WICED_FALSE;

    /* Create a semaphore to signal when LwIP has finished initialising */\
    mico_rtos_init_semaphore( &lwip_done_sema , 1 );
    if ( lwip_done_sema == NULL )
    {
        /* Could not create semaphore */
        WPRINT_NETWORK_ERROR(("Could not create LwIP init semaphore"));
        return WICED_ERROR;
    }

    /* Initialise LwIP, providing the callback function and callback semaphore */
    tcpip_init( tcpip_init_done, (void*) &lwip_done_sema );
    wiced_rtos_get_semaphore( &lwip_done_sema, MICO_NEVER_TIMEOUT );
    wiced_rtos_deinit_semaphore( &lwip_done_sema );

    /* Create a mutex for UDP and TCP sending with ability to swap a default interface */
    wiced_rtos_init_mutex( &send_interface_mutex );
    /* create a mutex for link up and down registrations */
    wiced_rtos_init_mutex( &link_subscribe_mutex );

    memset(&internal_dhcp_server, 0, sizeof(internal_dhcp_server));

    memset(link_up_callbacks,   0, sizeof(link_up_callbacks));
    memset(link_down_callbacks, 0, sizeof(link_down_callbacks));
    wiced_using_dhcp = WICED_FALSE;
    return WICED_SUCCESS;
}

/**
 *  LwIP init complete callback
 *
 *  This function is called by LwIP when initialisation has finished.
 *  A semaphore is posted to allow the startup thread to resume, and to run the app_main function
 *
 * @param arg : the handle for the semaphore to post (cast to a void pointer)
 */

static void tcpip_init_done( void* arg )
{
    wiced_semaphore_t* LwIP_done_sema = (wiced_semaphore_t *) arg;
    wiced_rtos_set_semaphore( LwIP_done_sema );
}



wiced_result_t wiced_network_deinit( void )
{
    wiced_assert("LwIP cannot be shut down", 0!=0 );
    return WICED_ERROR;
}

wiced_bool_t wiced_network_is_up( wiced_interface_t interface )
{
    return (wiced_wifi_is_ready_to_transceive(interface) == WICED_SUCCESS) ? WICED_TRUE : WICED_FALSE;
}

wiced_result_t wiced_network_suspend(void)
{
    /* Stop all registered TCP/IP timers */
    if( tcpip_deactivate_tcpip_timeouts() != ERR_OK )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

wiced_result_t wiced_network_resume(void)
{
    /* Resume all TCP/IP timers again */
    if( tcpip_activate_tcpip_timeouts() != ERR_OK )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

int ReallocIP(void)
{
    dhcp_renew( &IP_HANDLE(WICED_STA_INTERFACE) );

    return WICED_SUCCESS;
}

wiced_result_t wiced_ip_up( wiced_interface_t interface, wiced_network_config_t config, const wiced_ip_setting_t* ip_settings )
{
    struct ip_addr ipaddr;
    struct ip_addr netmask;
    struct ip_addr gw;
    wiced_bool_t static_ip;

    dhcp_interrupted = WICED_FALSE;
    
    if ( ip_networking_up[interface & 1] == WICED_TRUE )
    {
        return WICED_SUCCESS;
    }

    static_ip = ( ( config == WICED_USE_STATIC_IP || config == WICED_USE_INTERNAL_DHCP_SERVER) && ip_settings != NULL )? WICED_TRUE : WICED_FALSE;

    /* Enable the network interface */
    if ( static_ip == WICED_TRUE )
    {
        ipaddr.addr  = htonl(ip_settings->ip_address.ip.v4);
        gw.addr      = htonl(ip_settings->gateway.ip.v4);
        netmask.addr = htonl(ip_settings->netmask.ip.v4);
    }
    else
    {
        /* make sure that ip address is zero in order to start dhcp client */
        ip_addr_set_zero( &gw );
        ip_addr_set_zero( &ipaddr );
        ip_addr_set_zero( &netmask );
    }

    if ( NULL == netif_add( &IP_HANDLE(interface), &ipaddr, &netmask, &gw, (void*) (interface&1), ethernetif_init, ethernet_input ) )
    {
        WPRINT_NETWORK_ERROR(( "Could not add network interface\r\n" ));
        return WICED_ERROR;
    }

    IP_HANDLE(interface).hostname = hostname;
    
    if ( ( static_ip == WICED_FALSE ) && ( config == WICED_USE_EXTERNAL_DHCP_SERVER ))
    {
        wiced_ip_address_t dns_server_address;
        uint32_t     address_resolution_timeout = WICED_DHCP_IP_ADDRESS_RESOLUTION_TIMEOUT;
        wiced_bool_t timeout_occured            = WICED_FALSE;

        /* Bring up the network interface */
        netif_set_up( &IP_HANDLE(interface) );
        netif_set_default( &IP_HANDLE(interface) );

        //igmp_start(&IP_HANDLE(interface));

        WPRINT_NETWORK_INFO(("Obtaining IP address via DHCP\r\n"));
        dhcp_set_struct( &IP_HANDLE(interface), &wiced_dhcp_handle );
        dhcp_start( &IP_HANDLE(interface) );
        while ( wiced_dhcp_handle.state != DHCP_BOUND  && timeout_occured == WICED_FALSE )
        {
            sys_msleep( 10 );
            address_resolution_timeout -= 10;

            if ( address_resolution_timeout <= 0 )
            {
                /* timeout has occured */
                timeout_occured = WICED_TRUE;
            }
            if (dhcp_interrupted == WICED_TRUE)
            {
                timeout_occured = WICED_TRUE;
            }
        }

        if ( timeout_occured == WICED_TRUE )
        {
            dhcp_stop( &IP_HANDLE(interface) );
            netif_remove( &IP_HANDLE(interface) );
			printf("dhcp timeout\r\n");
            return WICED_ERROR;
        }

        wiced_using_dhcp = WICED_TRUE;

        /* Check if DNS servers were supplied by the DHCP client */
#if LWIP_DHCP && LWIP_DNS
        if ( (dns_getserver( 0 )).addr != IP_ADDR_ANY->addr )
        {
            u8_t i;
            ip_addr_t lwip_dns_server_addr;

            dns_server_address.version = WICED_IPV4;
            for (i = 0; i < DNS_MAX_SERVERS; i++)
            {
                lwip_dns_server_addr = dns_getserver(i);
                if (lwip_dns_server_addr.addr != IP_ADDR_ANY->addr)
                {
                    dns_server_address.ip.v4 = ntohl(lwip_dns_server_addr.addr);
                    dns_client_add_server_address(dns_server_address);
                }
            }
        }
        else
#endif /* if LWIP_DHCP && LWIP_DNS */
        {
            /* DNS servers were not supplied by DHCP client... */
            /* Add gateway DNS server and Google public DNS server */
            wiced_ip_get_gateway_address( interface, &dns_server_address );
            dns_client_add_server_address( dns_server_address );

            /* Google DNS server is 8.8.8.8 */
            memset( &dns_server_address.ip.v4, 8, sizeof( dns_server_address.ip.v4 ) );
            dns_client_add_server_address( dns_server_address );
        }

        /* Register for IP address change notification */
        /* TODO: Add support for IP address change notification */
    }
    else
    {
        netif_set_up( &IP_HANDLE(interface) );
        if (interface == WICED_STA_INTERFACE) // yhb added to always set STA interface as default interface. Fix static IP can't do broadcast/multicast
            netif_set_default( &IP_HANDLE(interface) );
        
        igmp_start(&IP_HANDLE(interface));

        /* Check if we should start the DHCP server */
        if ( config == WICED_USE_INTERNAL_DHCP_SERVER )
        {
            wiced_start_dhcp_server(&internal_dhcp_server, interface);
        }
    }

    ip_networking_up[interface&1] = WICED_TRUE;

#ifdef MXCHIPAPP 
    if (interface == WICED_STA_INTERFACE)
        send_mdns_up();
#endif

    WPRINT_NETWORK_INFO( ( "Network ready IP: %u.%u.%u.%u\r\n", (unsigned char) ( ( htonl( IP_HANDLE(interface).ip_addr.addr ) >> 24 ) & 0xff ),
        (unsigned char) ( ( htonl( IP_HANDLE(interface).ip_addr.addr ) >> 16 ) & 0xff ),
        (unsigned char) ( ( htonl( IP_HANDLE(interface).ip_addr.addr ) >>  8 ) & 0xff ),
        (unsigned char) ( ( htonl( IP_HANDLE(interface).ip_addr.addr ) >>  0 ) & 0xff ) ) );

    return WICED_SUCCESS;
}

wiced_result_t wiced_ip_down ( wiced_interface_t interface )
{
    if (interface == WICED_STA_INTERFACE)
        wiced_stop_dhcp();
    if ( ip_networking_up[interface&1] == WICED_TRUE )
    {
        /* Cleanup DNS client and DHCP server/client depending on interface */
        if ( ( interface == WICED_AP_INTERFACE ) || ( interface == WICED_CONFIG_INTERFACE ) )
        {
            wiced_stop_dhcp_server( &internal_dhcp_server );

            /* Wait till the time wait sockets get closed */
            sys_msleep(TIME_WAIT_TCP_SOCKET_DELAY);
        }
        else /* STA interface */
        {
            if ( wiced_using_dhcp == WICED_TRUE )
            {
                dhcp_stop( &IP_HANDLE(interface) );
                wiced_using_dhcp = WICED_FALSE;
            }
            dns_client_remove_all_server_addresses();
        }

        /* Delete the network interface */
        netif_remove( &IP_HANDLE(interface) );

        ip_networking_up[interface&1] = WICED_FALSE;
    }

    return WICED_SUCCESS;
}

/* Bring down the network interface
 *
 * @param interface       : wiced_interface_t, either WICED_AP_INTERFACE or WICED_STA_INTERFACE
 *
 * @return  WICED_SUCCESS : completed successfully
 *
 *  XXX : Needs testing
 *
 */
wiced_result_t wiced_network_down( wiced_interface_t interface )
{
    wiced_ip_down(interface);

    /* Stop Wi-Fi */
    if (wiced_network_is_up(interface) == WICED_TRUE)
    {
        if ( ( interface == WICED_AP_INTERFACE ) || ( interface == WICED_CONFIG_INTERFACE ) )
        {
            wiced_stop_ap( );
        }
        else
        {
            wiced_leave_ap();
        }
    }

    return WICED_SUCCESS;
}

void wiced_network_notify_link_up( void )
{
    /* Notify LwIP that the link is up */
    netif_set_up( &IP_HANDLE(WICED_STA_INTERFACE) );
}

void wiced_network_notify_link_down( void )
{
    /* Notify LwIP that the link is down*/
    netif_set_down( &IP_HANDLE(WICED_STA_INTERFACE) );
}

wiced_result_t wiced_network_link_down_handler( void* arg )
{
    int i = 0;
    UNUSED_PARAMETER( arg );

    WPRINT_NETWORK_DEBUG( ("Wireless link DOWN!\n\r") );

    if ( wiced_using_dhcp == WICED_TRUE )
    {
        dhcp_stop( &IP_HANDLE(WICED_STA_INTERFACE) );
    }
    /* Wait for a while before the time wait sockets get closed */
    sys_msleep( TIME_WAIT_TCP_SOCKET_DELAY );

    /* TODO: Add clearing of ARP cache */

    /* Inform all subscribers about the link down event */
    for ( i = 0; i < WICED_MAXIMUM_LINK_CALLBACK_SUBSCRIPTIONS; i++ )
    {
        if ( link_down_callbacks[i] != NULL )
        {
            link_down_callbacks[i]( );
        }
    }

    return WICED_SUCCESS;
}

wiced_result_t wiced_network_link_up_handler( void* arg )
{
    int i = 0;
    ip_addr_t ip_addr;

    UNUSED_PARAMETER( arg );

    WPRINT_NETWORK_DEBUG(("Wireless link UP!\n\r"));

    if ( wiced_using_dhcp == WICED_TRUE )
    {
        /* For DHCP only, we should reset netif IP address. We don't want to re-use previous netif IP address given from previous DHCP session */
        ip_addr_set_zero( &ip_addr );
        netif_set_ipaddr( &IP_HANDLE(WICED_STA_INTERFACE), &ip_addr);

        /* Restart DHCP */
        if ( dhcp_start( &IP_HANDLE(WICED_STA_INTERFACE)) != ERR_OK )
        {
            WPRINT_NETWORK_ERROR( ( "Failed to initiate DHCP transaction\r\n" ) );
            return WICED_ERROR;
        }
    }
    /* Inform all subscribers about an event */
    for ( i = 0; i < WICED_MAXIMUM_LINK_CALLBACK_SUBSCRIPTIONS; i++ )
    {
        if ( link_up_callbacks[i] != NULL )
        {
            link_up_callbacks[i]( );
        }
    }

    return WICED_SUCCESS;
}

wiced_result_t wiced_network_link_renew_handler( void )
{
    /* TODO: Do a DHCP renew if interface is using external DHCP server */

    if ( wiced_dhcp_handle.state == DHCP_BOUND )
    {
        netifapi_netif_common( &IP_HANDLE(WICED_STA_INTERFACE), (netifapi_void_fn)dhcp_renew, NULL);
    }

    return WICED_SUCCESS;
}

wiced_result_t wiced_network_register_link_callback( wiced_network_link_callback_t link_up_callback, wiced_network_link_callback_t link_down_callback )
{
    int i = 0;

    wiced_rtos_lock_mutex( &link_subscribe_mutex );

    /* Find next empty slot among the list of currently subscribed */
    for ( i = 0; i < WICED_MAXIMUM_LINK_CALLBACK_SUBSCRIPTIONS; ++i )
    {
        if ( link_up_callback != NULL && link_up_callbacks[i] == NULL )
        {
            link_up_callbacks[i] = link_up_callback;
            link_up_callback = NULL;
        }

        if ( link_down_callback != NULL && link_down_callbacks[i] == NULL )
        {
            link_down_callbacks[i] = link_down_callback;
            link_down_callback = NULL;
        }
    }

    wiced_rtos_unlock_mutex( &link_subscribe_mutex );

    /* Check if we didn't find a place of either of the callbacks */
    if ( (link_up_callback != NULL) || (link_down_callback != NULL) )
    {
        return WICED_ERROR;
    }
    else
    {
        return WICED_SUCCESS;
    }
}

wiced_result_t wiced_network_deregister_link_callback( wiced_network_link_callback_t link_up_callback, wiced_network_link_callback_t link_down_callback )
{
    int i=0;

    wiced_rtos_lock_mutex( &link_subscribe_mutex );

    /* Find matching callbacks */
    for ( i = 0; i < WICED_MAXIMUM_LINK_CALLBACK_SUBSCRIPTIONS; ++i )
    {
        if ( link_up_callback != NULL && link_up_callbacks[i] == link_up_callback )
        {
            link_up_callbacks[i] = NULL;
        }

        if ( link_down_callback != NULL && link_down_callbacks[i] == link_down_callback )
        {
            link_down_callbacks[i] = NULL;
        }
    }

    wiced_rtos_unlock_mutex( &link_subscribe_mutex );

    return WICED_SUCCESS;
}


wiced_result_t wiced_ip_register_address_change_callback( wiced_ip_address_change_callback_t callback, void* arg )
{
    /* Unimplemented */
    UNUSED_PARAMETER( callback );
    UNUSED_PARAMETER( arg );

    return WICED_ERROR;
}

wiced_result_t wiced_ip_deregister_address_change_callback( wiced_ip_address_change_callback_t callback )
{
    /* Unimplemented */
    UNUSED_PARAMETER( callback );

    return WICED_ERROR;
}

/* Below functions are added by yhb */
void wiced_set_hostname(char *name)
{
    strncpy(hostname, name, 32);
}

char *gethostname( char *name, int len )
{
   if( len ) {
      if(len < ((int)strlen( hostname ) + 1) )
         *name = 0;
      else
         strcpy( name, hostname );
      return( name );
   }
   return *hostname ?  hostname  : (char*)NULL;
}

char *sethostname( char *name )
{
	if (strlen(name) < 32) {
		strcpy(hostname, name);
		return (hostname);
	} else {
		return ((char*)NULL);
	}
}

void wiced_stop_dhcp(void)
{
    dhcp_interrupted = WICED_TRUE;
}

