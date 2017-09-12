/*****************************************************************************
**
**  Name:          wiced_bt_sdp.c
**
**  Description:   wiced wrappers
**
**
**  Copyright (c) 2014, Broadcom Corp, All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
******************************************************************************/
#include "bt_target.h"
#include "gki.h"
#include "btu.h"
#include "sdpint.h"

BOOLEAN mico_bt_sdp_db_init (UINT8 *p_gatt_db, UINT16 size)
{
    BTU_MUTEX_LOCK();
#if SDP_SERVER_ENABLED == TRUE
    sdp_cb.p_server_db                    = p_gatt_db;
    sdp_cb.server_db_len                  = size;
#endif
    BTU_MUTEX_UNLOCK();
    return (TRUE);
}
