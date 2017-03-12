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
 *  Provides prototypes / declarations for chip-specific APSTA functionality
 */

#ifndef INCLUDED_WWD_AP_H_
#define INCLUDED_WWD_AP_H_

#include "wiced_utilities.h"

extern wiced_bool_t wiced_wifi_ap_is_up;

extern wiced_bool_t wiced_wifi_is_packet_from_ap(uint8_t flags2);
/* mode: 1=default mode, b,g,n mixed.
  * 2: b,g mixed;
  * 3: b only;
  */
void wiced_set_ap_mode(int mode);


#endif /* ifndef INCLUDED_WWD_AP_H_ */
