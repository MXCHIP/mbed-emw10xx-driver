/*****************************************************************************
**
**  Name:          mico_bt_gatt.c
**
**  Description:   wiced wrappers
**
**
**  Copyright (c) 2014, Broadcom Corp, All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
******************************************************************************/


#include "bt_target.h"
#include "gki.h"
#include "gatt_api.h"
#include "gatt_int.h"
#include "legattdb.h"
#include <stdlib.h>
#include "legattdb.h"
#include "mico_bt_int.h"



/*****************************************************************************
** Shim layer between mico_bt_gatt and BTE GATT
******************************************************************************/

/* Register application callback for mico_bt_gatt event notification */
tGATT_STATUS mico_bt_gatt_register(tGATT_IF gatt_if, tGATT_CBACK *p_gatt_cback)
{
    tGATT_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = GATT_Register (gatt_if, p_gatt_cback);
    BTU_MUTEX_UNLOCK();

    return (retval);
}

/* Deregister application callback for mico_bt_gatt event notification */
void mico_bt_gatt_deregister (tGATT_IF gatt_if)
{
    BTU_MUTEX_LOCK();
    GATT_Deregister(gatt_if);
    BTU_MUTEX_UNLOCK();
}


/* Initiate GATT over LE connection */
/* Translate mico_bt_gatt_connect to BTE GATT_Connect */
BOOLEAN mico_bt_gatt_le_connect (BD_ADDR bd_addr, tBLE_ADDR_TYPE bd_addr_type, tBLE_CONN_MODE conn_mode,  BOOLEAN is_direct)
{
    BOOLEAN retval;

    BTU_MUTEX_LOCK();

    retval = GATT_BLE_Connect(GATT_CLIENT_IF_APP, bd_addr, bd_addr_type, conn_mode, is_direct);

    BTU_MUTEX_UNLOCK();

    return(retval);
}

/* Initiate GATT over BR/EDR connection */
/* Translate mico_bt_gatt_connect to BTE GATT_Connect */
BOOLEAN mico_bt_gatt_bredr_connect (BD_ADDR bd_addr)
{
    BOOLEAN retval;

    BTU_MUTEX_LOCK();

    retval = GATT_BR_Connect (GATT_FIXED_DB_IF_APP, bd_addr);

    BTU_MUTEX_UNLOCK();

    return(retval);
}
/* Cancel GATT connection */
/* Translate mico_bt_gatt_cancel_connect to BTE GATT_CancelConnect */
BOOLEAN mico_bt_gatt_cancel_connect (BD_ADDR bd_addr, BOOLEAN is_direct)
{
    BOOLEAN retval;

    BTU_MUTEX_LOCK();
    retval = GATT_CancelConnect(GATT_CLIENT_IF_APP, bd_addr, is_direct);
    BTU_MUTEX_UNLOCK();

    return(retval);
}

/* Add/remove device from whitelist */
mico_bool_t mico_bt_gatt_listen (mico_bool_t start, mico_bt_device_address_t bd_addr)
{
    mico_bool_t retval;

    BTU_MUTEX_LOCK();
    retval = GATT_Listen (GATT_CLIENT_IF_APP, start, bd_addr);
    BTU_MUTEX_UNLOCK();

    return (retval);
}


/*****************************************************************************
** Utility functions used by LEGATTDB (pre-built GATT database parser)
******************************************************************************/


void blecm_log1(char *str,int val )
{
}

void ble_tracen (char *p_str, UINT16 len)
{
}

int  blecm_needToGATTDB16( void )
{
    return 0;
}

UINT32 legattdb_checkEncAuthRequirement(UINT8 conn_idx)
{
    tGATT_TCB *p_tcb = gatt_get_tcb_by_idx(conn_idx);
    UINT8       cur_secur;
    UINT8       cur_encr_key_size;

    if(p_tcb == NULL)
        return LEATT_ERR_CODE_UNLIKELY_ERROR;

    if(!BTM_GetLeSecurityState(p_tcb->peer_bda,&cur_secur,&cur_encr_key_size))
        return LEATT_ERR_CODE_UNLIKELY_ERROR;

    GATT_TRACE_DEBUG1("legattdb_checkEncAuthRequirement :%x", cur_secur);

    if(cur_secur & BTM_SEC_LE_LINK_ENCRYPTED)
        return 0;
    else
        return LEATT_ERR_CODE_INSUFFICIENT_AUTHENTICATION;
}

int emconninfo_linkEncrypted(UINT8 conn_idx)
{
    tGATT_TCB *p_tcb = gatt_get_tcb_by_idx(conn_idx);
    UINT8       cur_secur;
    UINT8       cur_encr_key_size;

    if(p_tcb == NULL)
        return LEATT_ERR_CODE_UNLIKELY_ERROR;

    if(!BTM_GetLeSecurityState(p_tcb->peer_bda,&cur_secur,&cur_encr_key_size))
        return LEATT_ERR_CODE_UNLIKELY_ERROR;

    GATT_TRACE_DEBUG1("emconninfo_linkEncrypted :%x", cur_secur);

    if(cur_secur & BTM_SEC_LE_LINK_ENCRYPTED)
        return 0;
    else
        return LEATT_ERR_CODE_INSUFFICIENT_AUTHENTICATION;
}

// This will send an ATT Error response.
void leatt_sendErrResponse(int errCode, int reqOpcode, int handleInError )
{
    GATT_TRACE_DEBUG0("STUB: leatt_sendErrResponse");
}

// This is a convenient function to map 16 bits UUID to 128 bits.
void leatt_mapUUID16ToUUID128(UINT16 uuid_16, UINT8 *uuid128Holder)
{
    gatt_convert_uuid16_to_uuid128(uuid128Holder, uuid_16);
}

