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
 *  Defines functions to communicate over the IP network
 */

#pragma once

#include "wiced_utilities.h"
#include "wwd_network_interface.h"
#include "wiced_network.h"
#include <limits.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 * @cond                Macros
 ******************************************************/

/* For wiced_tcp_bind - if the local port number is irrelevant */
#define WICED_ANY_PORT    (0)

#define INITIALISER_IPV4_ADDRESS( addr_var, addr_val )  addr_var = { WICED_IPV4, { .v4 = (uint32_t)(addr_val) } }
#define SET_IPV4_ADDRESS( addr_var, addr_val )          (((addr_var).version = WICED_IPV4),((addr_var).ip.v4 = (uint32_t)(addr_val)))
#define GET_IPV4_ADDRESS( addr_var )                    ((addr_var).ip.v4)
#define WICED_IP_BROADCAST                              (&wiced_ip_broadcast)
#define MAKE_IPV4_ADDRESS(a, b, c, d)                   ((((uint32_t) a) << 24) | (((uint32_t) b) << 16) | (((uint32_t) c) << 8) | ((uint32_t) d))
#define GET_IPV6_ADDRESS( addr_var )                    ((uint32_t*)((addr_var).ip.v6))
#define SET_IPV6_ADDRESS( addr_var, addr_val )          { \
                                                            uint32_t _value[4] = addr_val; \
                                                            (addr_var).version = WICED_IPV6; \
                                                            (addr_var).ip.v6[0] = _value[0];  \
                                                            (addr_var).ip.v6[1] = _value[1];  \
                                                            (addr_var).ip.v6[2] = _value[2];  \
                                                            (addr_var).ip.v6[3] = _value[3];  \
                                                        }
#define MAKE_IPV6_ADDRESS(a, b, c, d, e, f, g, h)       { \
                                                            (((((uint32_t) (a)) << 16) & 0xFFFF0000UL) | ((uint32_t)(b) &0x0000FFFFUL)), \
                                                            (((((uint32_t) (c)) << 16) & 0xFFFF0000UL) | ((uint32_t)(d) &0x0000FFFFUL)), \
                                                            (((((uint32_t) (e)) << 16) & 0xFFFF0000UL) | ((uint32_t)(f) &0x0000FFFFUL)), \
                                                            (((((uint32_t) (g)) << 16) & 0xFFFF0000UL) | ((uint32_t)(h) &0x0000FFFFUL))  \
                                                        }


/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef void (*wiced_ip_address_change_callback_t)( void* arg );

/******************************************************
 *            Enumerations
 ******************************************************/
typedef struct
{
	wiced_tcp_socket_t* socket;
	wiced_packet_t* 	packet;
	uint8_t*			packet_data;
	uint16_t			packet_space_available;
} wiced_tcp_stream_t;

typedef enum
{
    WICED_IPV4 = 4,
    WICED_IPV6 = 6,
    WICED_INVALID_IP = INT_MAX
} wiced_ip_version_t;

typedef enum
{
    IPv6_LINK_LOCAL_ADDRESS,
    IPv6_GLOBAL_ADDRESS,
} wiced_ipv6_address_type_t;

/******************************************************
 *             Structures
 ******************************************************/


typedef struct
{
    wiced_ip_version_t version;

    union
    {
        uint32_t v4;
        uint32_t v6[4];
    } ip;
} wiced_ip_address_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

extern const wiced_ip_address_t wiced_ip_broadcast;

/******************************************************
 *               Function Declarations
 * @endcond
 ******************************************************/

/*****************************************************************************/
/** @defgroup ipcoms       IP Communication
 *
 *  WICED IP Communication Functions
 */
/*****************************************************************************/


/*****************************************************************************/
/** @addtogroup tcp       TCP
 *  @ingroup ipcoms
 *
 * Communication functions for TCP (Transmission Control Protocol)
 * Many of these are similar to the BSD-Sockets functions which are standard on POSIX
 *
 *  @{
 */
/*****************************************************************************/

/** Create a new TCP socket
 *
 *  Creates a new TCP socket.
 *  Additional steps required for the socket to become active:
 *
 *  Client socket:
 *   - bind - the socket needs to be bound to a local port ( usually WICED_ANY_PORT )
 *   - connect - connect to a specific remote IP & TCP port
 *
 *  Server socket:
 *   - listen - opens a specific local port and attaches socket to it.
 *   - accept - waits for a remote client to establish a connection
 *
 * @param[out] socket    : A pointer to a UDP socket structure which will receive the created socket handle
 * @param[in]  interface : The interface (AP or STA) for which the socket should be created
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_tcp_create_socket( wiced_tcp_socket_t* socket, wiced_interface_t interface );


/** Registers a callback function with the indicated TCP socket
 *
 * @param[in,out] socket              : A pointer to a TCP socket handle that has been previously created with @ref wiced_tcp_create_socket
 * @param[in]     connect_callback    :
 * @param[in]     receive_callback    :
 * @param[in]     disconnect_callback :
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_tcp_register_callbacks( wiced_tcp_socket_t* socket, wiced_socket_callback_t connect_callback, wiced_socket_callback_t receive_callback, wiced_socket_callback_t disconnect_callback);


/** Binds a TCP socket to a local TCP port
 *
 *  Binds a TCP socket to a local port.
 *
 * @param[in,out] socket : A pointer to a socket handle that has been previously created with @ref wiced_tcp_create_socket
 * @param[in]     port   : The TCP port number on the local device. Can be WICED_ANY_PORT if it is not important.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_tcp_bind( wiced_tcp_socket_t* socket, uint16_t port );


/** Connects a client TCP socket to a remote server
 *
 *  Connects an existing client TCP socket to a specific remote server TCP port
 *
 * @param[in,out] socket     : A pointer to a socket handle that has been previously created with @ref wiced_tcp_create_socket
 * @param[in]     address    : The IP address of the remote server to which the connection should be made
 * @param[in]     port       : The TCP port number on the remote server
 * @param[in]     timeout_ms : Timeout period in milliseconds
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_tcp_connect( wiced_tcp_socket_t* socket, const wiced_ip_address_t* address, uint16_t port, uint32_t timeout_ms );


/** Opens a specific local port and attaches a socket to listen on it.
 *
 *  Opens a specific local port and attaches a socket to listen on it.
 *
 * @param[in,out] socket : A pointer to a socket handle that has been previously created with @ref wiced_tcp_create_socket
 * @param[in]     port   : The TCP port number on the local device
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_tcp_listen( wiced_tcp_socket_t* socket, uint16_t port );

/** Returns the details( ip address and the source port) of the client  
 *  which is connected currently to a server
 *  
 *  @param[in] socket  : A pointer to a socket handle that has been previously created with @ref wiced_tcp_create_socket 
 *  @param[in/out] address: returned IP address of the connected client
 *  @param[in/out] address: returned source port of the connected client
 *  
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_tcp_server_peer( wiced_tcp_socket_t* socket, wiced_ip_address_t* address, uint16_t* port );


/** Wait for a remote client and establish TCP connection
 *
 *  Sleeps until a remote client to connects to the given socket.
 *
 * @param[in,out] socket : A pointer to a socket handle that has been previously listened with @ref wiced_tcp_listen
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_tcp_accept( wiced_tcp_socket_t* socket );

/** Disconnect a TCP connection
 *
 *  Disconnects a TCP connection from a remote host
 *
 * @param[in,out] socket : The open TCP socket to disconnect
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_tcp_disconnect( wiced_tcp_socket_t* socket );


/** Deletes a TCP socket
 *
 *  Deletes a TCP socket. Socket must be either never opened or disconnected.
 *
 * @param[in,out] socket : The open TCP socket to delete
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_tcp_delete_socket( wiced_tcp_socket_t* socket );


/** Enable TLS on a TCP server socket
 *
 * Enable Transport Layer Security (successor to SSL) on a TCP
 * socket with a pre-existing TLS context
 *
 * @note: if socket is not yet connected with @ref wiced_tcp_accept , then a
 *        call to @ref wiced_tcp_accept will cause TLS to start.
 *        Otherwise, if a connection is already established, you will need
 *        to call @ref wiced_tcp_start_tls to begin TLS communication.
 *
 * @param[in,out] socket  : The TCP socket to use for TLS
 * @param[in]     context : The TLS context to use for security. This must
 *                          have been initialised with @ref wiced_tls_init_simple_context
 *                          or @ref wiced_tls_init_advanced_context
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_tcp_enable_tls( wiced_tcp_socket_t* socket, void* context );



/*****************************************************************************/
/** @addtogroup tcppkt       TCP packet comms
 *  @ingroup tcp
 *
 * Functions for communication over TCP in packet mode
 *
 *  @{
 */
/*****************************************************************************/

/** Send a TCP data packet
 *
 *  Sends a TCP packet to the remote host.
 *  Once this function is called, the caller must not use the packet pointer
 *  again, since ownership has been transferred to the IP stack.
 *
 * @param[in,out] socket : A pointer to an open socket handle.
 * @param[in]     packet : A pointer to a packet to be sent.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_tcp_send_packet( wiced_tcp_socket_t* socket, wiced_packet_t* packet );


/** Receives a TCP data packet
 *
 *  Attempts to receive a TCP data packet from the remote host.
 *  If a packet is returned successfully, then ownership of it
 *  has been transferred to the caller, and it must be released
 *  with @ref wiced_packet_delete as soon as it is no longer needed.
 *
 * @param[in,out] socket  : A pointer to an open socket handle.
 * @param[in]     packet  : A pointer to a packet pointer which will be
 *                          filled with the received packet.
 * @param[in]     timeout : Timeout value in milliseconds or WICED_NEVER_TIMEOUT
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_tcp_receive( wiced_tcp_socket_t* socket, wiced_packet_t** packet, uint32_t timeout );


/** @} */
/*****************************************************************************/
/** @addtogroup tcpbfr       TCP buffer comms
 *  @ingroup tcp
 *
 * Functions for communication over TCP with C array buffers
 *
 *  @{
 */
/*****************************************************************************/

/** Send a memory buffer of TCP data
 *
 *  Sends a memory buffer containing TCP data to the remote host.
 *  This is not limited by packet sizes.
 *
 * @param[in,out] socket        : A pointer to an open socket handle.
 * @param[in]     buffer        : The memory buffer to send
 * @param[in]     buffer_length : The number of bytes in the buffer to send
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_tcp_send_buffer( wiced_tcp_socket_t* socket, const void* buffer, uint16_t buffer_length );


/** @} */
/*****************************************************************************/
/** @addtogroup tcpstream       TCP stream comms
 *  @ingroup tcp
 *
 * Functions for communication over TCP in stream mode
 * Users need not worry about splitting data into packets in this mode
 *
 *  @{
 */
/*****************************************************************************/


/** Creates a stream for a TCP connection
 *
 *  Creates a stream for a TCP connection.
 *  The stream allows the user to write successive small
 *  amounts data into the stream without worrying about packet boundaries
 *
 * @param[out]    tcp_stream : A pointer to a stream handle that will be initialised
 * @param[in,out] socket     : A pointer to an open socket handle.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_tcp_stream_init( wiced_tcp_stream_t* tcp_stream, wiced_tcp_socket_t* socket );


/** Deletes a TCP stream
 *
 *  Deletes a stream for a TCP connection.
 *
 * @param[in,out] tcp_stream : A pointer to a stream handle that will be de-initialised
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_tcp_stream_deinit( wiced_tcp_stream_t* tcp_stream );


/** Write data into a TCP stream
 *
 *  Write data into an open TCP stream.
 *  Data will only be sent if it causes the current internal packet
 *  to become full, or if @ref wiced_tcp_stream_flush is called.
 *
 * @param[in,out] tcp_stream  : A pointer to a stream handle where data will be written
 * @param[in]     data        : The memory buffer to send
 * @param[in]     data_length : The number of bytes in the buffer to send
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_tcp_stream_write( wiced_tcp_stream_t* tcp_stream, const void* data, uint16_t data_length );


/** Flush pending TCP stream data out to remote host
 *
 *  Flushes any pending data in the TCP stream out to the remote host
 *
 * @param[in,out] tcp_stream  : A pointer to a stream handle whose pending data will be flushed
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_tcp_stream_flush( wiced_tcp_stream_t* tcp_stream );

/* Enable TCP keepalive mechanism on a specified socket
 *
 * @param[in]     socket    : Pointer to a tcp_socket
 * @param[in]     interval  : The interval between subsequent keep-alive probes
 * @param[in]     probes    : The number of unacknowledged probes to send before considering \n
 *                            the connection dead and notifying the application layer
 * @param[in]     time      : The interval between the last data packet sent (simple ACKs are not \n
                             considered data) and the first keep-alive probe
*/
wiced_result_t wiced_tcp_enable_keepalive(wiced_tcp_socket_t* socket, uint16_t interval, uint16_t probes, uint16_t _time );


/** @} */
/** @} */
/*****************************************************************************/
/** @addtogroup udp       UDP
 *  @ingroup ipcoms
 *
 * Communication functions for UDP (User Datagram Protocol)
 *
 *  @{
 */
/*****************************************************************************/

/** Create a new UDP socket
 *
 *  Creates a new UDP socket.
 *  If successful, the socket is immediately ready to communicate
 *
 * @param[out] socket    : A pointer to a UDP socket structure which will receive the created socket handle
 * @param[in]  port      : The UDP port number on the local device to use. (must not be in use already)
 * @param[in]  interface : The interface (AP or STA) for which the socket should be created
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_udp_create_socket( wiced_udp_socket_t* socket, uint16_t port, wiced_interface_t interface );


/** Send a UDP data packet
 *
 *  Sends a UDP packet to a remote host.
 *  Once this function is called, the caller must not use the packet pointer
 *  again, since ownership has been transferred to the IP stack.
 *
 * @param[in,out] socket  : A pointer to an open UDP socket handle.
 * @param[in]     address : The IP address of the remote host
 * @param[in]     port    : The UDP port number on the remote host
 * @param[in]     packet  : A pointer to the packet to be sent.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_udp_send( wiced_udp_socket_t* socket, const wiced_ip_address_t* address, uint16_t port, wiced_packet_t* packet );


/** Receives a UDP data packet
 *
 *  Attempts to receive a UDP data packet from the remote host.
 *  If a packet is returned successfully, then ownership of it
 *  has been transferred to the caller, and it must be released
 *  with @ref wiced_packet_delete as soon as it is no longer needed.
 *
 * @param[in,out] socket  : A pointer to an open UDP socket handle.
 * @param[in]     packet  : A pointer to a packet pointer which will be
 *                          filled with the received packet.
 * @param[in]     timeout : Timeout value in milliseconds or WICED_NEVER_TIMEOUT
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_udp_receive( wiced_udp_socket_t* socket, wiced_packet_t** packet, uint32_t timeout );


/** Replies to a UDP received data packet
 *
 *  Sends a UDP packet to the host IP address and UDP
 *  port from which a previous packet was received.
 *  Ownership of the received packet does not change.
 *  Ownership of the packet being sent is transferred to the IP stack.
 *
 * @param[in,out] socket     : A pointer to an open UDP socket handle.
 * @param[in]     in_packet  : Pointer to a packet previously received with @ref wiced_udp_receive
 * @param[in]     out_packet : A packet pointer for the UDP packet to be sent
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_udp_reply( wiced_udp_socket_t* socket, wiced_packet_t* in_packet, wiced_packet_t* out_packet );



/** Deletes a UDP socket
 *
 *  Deletes a UDP socket that has been created with @ref wiced_udp_create_socket
 *
 * @param[in,out] socket : A pointer to an open UDP socket handle.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_udp_delete_socket( wiced_udp_socket_t* socket );


/** Get the remote IP address and UDP port of a received packet
 *
 * Get the IP address and UDP port number details of the remote
 * host for a received packet
 *
 * @param[in]  packet  : the packet handle
 * @param[out] address : a pointer to an address structure that will receive the remote IP address
 * @param[out] port    : a pointer to a variable that will receive the remote UDP port number
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_udp_packet_get_info( wiced_packet_t* packet, wiced_ip_address_t* address, uint16_t* port );


/** Registers a callback function with the indicated UDP socket
 *
 * @param[in,out] socket           : A pointer to a TCP socket handle that has been previously created with @ref wiced_tcp_create_socket
 * @param[in]     receive_callback : The callback function that will be called when a UDP packet is received
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_udp_register_callbacks( wiced_udp_socket_t* socket, wiced_socket_callback_t receive_callback );



/** @} */

/*****************************************************************************/
/** @addtogroup icmp       ICMP ping
 *  @ingroup ipcoms
 *
 * Functions for ICMP echo requests (Internet Control Message Protocol)
 * This is commonly known as ping
 *
 *  @{
 */
/*****************************************************************************/

/** Sends a ping (ICMP echo request)
 *
 *  Sends a ICMP echo request (a ping) and waits for the response.
 *  Supports both IPv4 and IPv6
 *
 * @param[in]  interface  : The interface (AP or STA) on which to send the ping
 * @param[in]  address    : The IP address to which the ping should be sent
 * @param[in]  timeout_ms : Timeout value in milliseconds
 * @param[out] elapsed_ms : Pointer to a uint32_t which will receive the elapsed response time in milliseconds
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_ping( wiced_interface_t interface, const wiced_ip_address_t* address, uint32_t timeout_ms, uint32_t* elapsed_ms );

/** @} */

/*****************************************************************************/
/** @addtogroup dns       DNS lookup
 *  @ingroup ipcoms
 *
 * Functions for DNS (Domain Name System) lookups
 *
 *  @{
 */
/*****************************************************************************/


/** Looks up a hostname via DNS
 *
 *  Sends a DNS query to find an IP address for a given hostname string.
 *
 *  @note :  hostname is permitted to be in dotted quad form
 *  @note :  The returned IP may be IPv4 or IPv6
 *
 * @param[in]  hostname   : A null-terminated string containing the hostname to be looked-up
 * @param[out] address    : A pointer to an IP address that will receive the resolved address
 * @param[in]  timeout_ms : Timeout value in milliseconds
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_hostname_lookup( const char* hostname, wiced_ip_address_t* address, uint32_t timeout_ms );


/** @} */

/*****************************************************************************/
/** @addtogroup igmp       IGMP multicast
 *  @ingroup ipcoms
 *
 * Functions for joining/leaving IGMP (Internet Group Management Protocol) groups
 *
 *  @{
 */
/*****************************************************************************/

/** Joins an IGMP group
 *
 *  Joins an IGMP multicast group, allowing reception of packets being sent to
 *  the group.
 *
 * @param[in] interface : The interface (AP or STA) which should be used to join the group
 * @param[in] address   : The IP address of the multicast group which should be joined.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_multicast_join( wiced_interface_t interface, const wiced_ip_address_t* address );


/** Leaves an IGMP group
 *
 *  Leaves an IGMP multicast group, stopping reception of packets being sent to
 *  the group.
 *
 * @param[in] interface : The interface (AP or STA) which should was used to join the group
 * @param[in] address   : The IP address of the multicast group which should be left.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_multicast_leave( wiced_interface_t interface, const wiced_ip_address_t* address );

/** @} */

/*****************************************************************************/
/** @addtogroup pktmgmt       Packet management
 *  @ingroup ipcoms
 *
 * Functions for allocating/releasing/processing packets from the WICED packet pool
 *
 *  @{
 */
/*****************************************************************************/


/** Allocates a TCP packet from the pool
 *
 *  Allocates a TCP packet from the main packet pool.
 *
 *  @note: Packets are fixed size. and applications must be very careful
 *         to avoid writing past the end of the packet buffer.
 *         The available_space parameter should be used for this.
 *
 * @param[in,out] socket          : An open TCP socket for which the packet should be created
 * @param[in]     content_length  : the intended length of TCP content if known.
 *                                  (This can be adjusted at a later point with @ref wiced_packet_set_data_end if not known)
 * @param[out]    packet          : Pointer to a packet handle which will receive the allocated packet
 * @param[out]    data            : Pointer pointer which will receive the data pointer for the packet. This is where
 *                                  TCP data should be written
 * @param[out]    available_space : pointer to a variable which will receive the space
 *                                  available for TCP data in the packet in bytes
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_packet_create_tcp( wiced_tcp_socket_t* socket, uint16_t content_length, wiced_packet_t** packet, uint8_t** data, uint16_t* available_space );


/** Allocates a UDP packet from the pool
 *
 *  Allocates a UDP packet from the main packet pool.
 *
 *  @note: Packets are fixed size. and applications must be very careful
 *         to avoid writing past the end of the packet buffer.
 *         The available_space parameter should be used for this.
 *
 * @param[in,out] socket          : An open UDP socket for which the packet should be created
 * @param[in]     content_length  : the intended length of UDP content if known.
 *                                  (This can be adjusted at a later point with @ref wiced_packet_set_data_end if not known)
 * @param[out]    packet          : Pointer to a packet handle which will receive the allocated packet
 * @param[out]    data            : Pointer pointer which will receive the data pointer for the packet. This is where
 *                                  UDP data should be written
 * @param[out]    available_space : pointer to a variable which will receive the space
 *                                  available for UDP data in the packet in bytes
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_packet_create_udp( wiced_udp_socket_t* socket, uint16_t content_length, wiced_packet_t** packet, uint8_t** data, uint16_t* available_space );


/** Allocates a general packet from the pool
 *
 *  Allocates a general packet from the main packet pool.
 *  Packet will not be usable for TCP/UDP as it will not
 *  have the required headers.
 *
 *  @note: Packets are fixed size. and applications must be very careful
 *         to avoid writing past the end of the packet buffer.
 *         Theavailable_space parameter should be used for this.
 *
 * @param[in]  content_length   : the intended length of content if known.
 *                                (This can be adjusted at a later point with @ref wiced_packet_set_data_end if not known)
 * @param[out] packet           : Pointer to a packet handle which will receive the allocated packet
 * @param[out] data             : Pointer pointer which will receive the data pointer for the packet. This is where
 *                                data should be written
 * @param[out] available_space  : pointer to a variable which will receive the space
 *                                available for data in the packet in bytes
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_packet_create( uint16_t content_length, wiced_packet_t** packet, uint8_t** data, uint16_t* available_space );


/** Releases a packet back to the pool
 *
 *  Releases a packet that is in use, back to the main packet pool,
 *  allowing re-use.
 *
 * @param[in,out] packet : the packet to be released
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_packet_delete( wiced_packet_t* packet );


/** Gets a data buffer pointer for a packet
 *
 * Retrieves a data buffer pointer for a given packet handle at a particular offset.
 * For fragmented packets, the offset input is used to traverse through the packet chain.
 *
 * @param[in,out] packet                : the packet handle for which to get a data pointer
 * @param[in]     offset                : the offset from the starting address.
 * @param[out]    data                  : a pointer which will receive the data pointer
 * @param[out]    data_length           : the current length of data in the packet in bytes.
 * @param[out]    available_data_length : for a TX packet: the current amount of extra data space that is available in bytes.
 *                                        for an RX packet: the amount of data available to be read from all fragments
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_packet_get_data( wiced_packet_t* packet, uint16_t offset, uint8_t** data, uint16_t* data_length, uint16_t *available_data_length );


/** Set the size of data in a packet
 *
 * If data has been added to a packet, this function should be
 * called to ensure the packet length is updated
 *
 * @param[in,out]  packet   : the packet handle
 * @param[in]      data_end : a pointer to the address immediately after the
 *                            last data byte in the packet buffer
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_packet_set_data_end( wiced_packet_t* packet, uint8_t* data_end );

/** Set the size of data in a packet
 *
 * If data has been processed in this packet, this function should be
 * called to ensure calls to wiced_packet_get_data() skip the processed
 * data.
 *
 * @param[in,out] packet     : the packet handle
 * @param[in]     data_start : a pointer to the address immediately after the
 *                             last processed byte in the packet buffer
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_packet_set_data_start( wiced_packet_t* packet, uint8_t* data_start );


/** Get the next fragment from a packet chain
 *
 * Retrieves the next fragment from a given packet handle
 *
 * @param[in]  packet               : the packet handle
 * @param[out] next_packet_fragment : the packet handle of the next fragment
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_packet_get_next_fragment( wiced_packet_t* packet, wiced_packet_t** next_packet_fragment);

/** @} */

/*****************************************************************************/
/** @addtogroup rawip       Raw IP
 *  @ingroup ipcoms
 *
 * Functions to access IP information from network interfaces
 *
 *  @{
 */
/*****************************************************************************/


/** Retrieves the IPv4 address for an interface
 *
 * Retrieves the IPv4 address for an interface (AP or STA) if it
 * exists.
 *
 * @param[in]  interface    : the interface (AP or STA)
 * @param[out] ipv4_address : the address structure to be filled
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_ip_get_ipv4_address( wiced_interface_t interface, wiced_ip_address_t* ipv4_address );


/** Retrieves the IPv6 address for an interface
 *
 * Retrieves the IPv6 address for an interface (AP or STA) if it
 * exists.
 *
 * @param[in]  interface    : the interface (AP or STA)
 * @param[out] ipv6_address : the address structure to be filled
 * @param[in]  address_type : the address type
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_ip_get_ipv6_address( wiced_interface_t interface, wiced_ip_address_t* ipv6_address, wiced_ipv6_address_type_t address_type );


/** Retrieves the IPv4 gateway address for an interface
 *
 * Retrieves the gateway IPv4 address for an interface (AP or STA) if it
 * exists.
 *
 * @param[in]   interface    : the interface (AP or STA)
 * @param[out]  ipv4_address : the address structure to be filled with the
 *                             gateway IP
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_ip_get_gateway_address( wiced_interface_t interface, wiced_ip_address_t* ipv4_address );


/** Retrieves the IPv4 netmask for an interface
 *
 * Retrieves the gateway IPv4 netmask for an interface (AP or STA) if it
 * exists.
 *
 * @param[in]  interface    : the interface (AP or STA)
 * @param[out] ipv4_address : the address structure to be filled with the
 *                            netmask
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_ip_get_netmask( wiced_interface_t interface, wiced_ip_address_t* ipv4_address );

wiced_result_t wiced_ip_get_ipaddr( wiced_interface_t interface, uint32_t *ip, uint32_t *mask, uint32_t *gw);


/** Registers a callback function that gets called when the IP address has changed
 *
 * Registers a callback function that gets called when the IP address has changed
 *
 * @param[in] callback : callback function to register
 * @param[in] arg      : pointer to the argument to pass to the callback
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_ip_register_address_change_callback( wiced_ip_address_change_callback_t callback, void* arg );


/** De-registers a callback function that gets called when the IP address has changed
 *
 * De-registers a callback function that gets called when the IP address has changed
 *
 * @param[in] callback : callback function to de-register
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_ip_deregister_address_change_callback( wiced_ip_address_change_callback_t callback );



#ifdef __cplusplus
} /*extern "C" */
#endif
