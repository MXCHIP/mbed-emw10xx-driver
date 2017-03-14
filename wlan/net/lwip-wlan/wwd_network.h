/*
 * Copyright 2013, Broadcom Corporation
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */

#ifndef INCLUDED_WICED_NETWORK_H_
#define INCLUDED_WICED_NETWORK_H_

#include "lwip/err.h"

#ifdef __cplusplus
extern "C"
{
#endif

struct pbuf;
struct netif;

/******************************************************
 *             Constants
 ******************************************************/

/******************************************************
 *             Structures
 ******************************************************/

/******************************************************
 *             Function declarations
 ******************************************************/

extern err_t ethernetif_init( /*@partial@*/ struct netif *netif );

/******************************************************
 *             Global variables
 ******************************************************/

#ifdef __cplusplus
}
#endif

#endif /* define INCLUDED_WICED_NETWORK_H_ */
