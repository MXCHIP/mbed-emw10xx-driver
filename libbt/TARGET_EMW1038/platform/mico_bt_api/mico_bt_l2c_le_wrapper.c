/*****************************************************************************
**
**  Name:          mico_bt_l2c_wrapper.c
**
**  Description:   mico_bt_* API wrappers
**
**  Copyright (c) 2014, Broadcom Corp, All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
******************************************************************************/
#include <string.h>
#include "bt_target.h"
#include "btu.h"
#include "l2c_api.h"
#include "gki.h"
#include "l2c_int.h"
#include "mico_bt_int.h"

UINT8 mico_bt_l2cap_le_data_write (UINT16 cid, UINT8 *p_buf, UINT16 buf_len, UINT16 flags)
{
    UINT8 retval = 0;
    BT_HDR *p_l2c_buf;

    BTU_MUTEX_LOCK();
    if ((p_l2c_buf = (BT_HDR *)GKI_getbuf(sizeof(BT_HDR) + L2CAP_MIN_OFFSET + buf_len)) != NULL)
    {
        p_l2c_buf->len = buf_len;
        p_l2c_buf->offset = L2CAP_MIN_OFFSET;
        p_l2c_buf->event = 0;
        p_l2c_buf->layer_specific = 0;
        memcpy((UINT8*) (p_l2c_buf + 1) + p_l2c_buf->offset, (UINT8*)p_buf, buf_len);
        retval = L2CA_LeDataWrite (cid, p_l2c_buf);
    }
    BTU_MUTEX_UNLOCK();

    return retval;
}

BOOLEAN mico_bt_l2cap_cancel_ble_connect_req(BD_ADDR rem_bda)
{
    BOOLEAN retval = FALSE;

    BTU_MUTEX_LOCK();
    retval = L2CA_CancelBleConnectReq (rem_bda);
    BTU_MUTEX_UNLOCK();

    return retval;
}

BOOLEAN mico_bt_l2cap_update_ble_conn_params (BD_ADDR rem_bda, UINT16 min_int, UINT16 max_int, UINT16 latency, UINT16 timeout)
{
    BOOLEAN retval = FALSE;

    BTU_MUTEX_LOCK();
    retval = L2CA_UpdateBleConnParams (rem_bda, min_int, max_int, latency, timeout);
    BTU_MUTEX_UNLOCK();

    return retval;
}

BOOLEAN mico_bt_l2cap_enable_update_ble_conn_params (BD_ADDR rem_bda, BOOLEAN enable)
{
    BOOLEAN retval = FALSE;

    BTU_MUTEX_LOCK();
    retval = L2CA_EnableUpdateBleConnParams (rem_bda, enable);
    BTU_MUTEX_UNLOCK();

    return retval;
}

UINT8 mico_bt_l2cap_get_ble_conn_role (BD_ADDR bd_addr)
{
    UINT8 retval = 0;

    BTU_MUTEX_LOCK();
    retval = L2CA_GetBleConnRole (bd_addr);
    BTU_MUTEX_UNLOCK();

    return retval;
}

UINT16 mico_bt_l2cap_le_register (UINT16 le_psm, mico_bt_l2cap_le_appl_information_t *p_cb_info, void *context)
{
    UINT16 retval;

    BTU_MUTEX_LOCK();
    retval = L2CA_LeRegister (le_psm, p_cb_info, L2CAP_REG_OPTS_NO_BT_HDR, context);
    BTU_MUTEX_UNLOCK();

    return (retval);
}

BOOLEAN mico_bt_l2cap_le_deregister (UINT16 le_psm)
{
    BOOLEAN retval = FALSE;

    BTU_MUTEX_LOCK();
    retval = L2CA_LeDeregister(le_psm);
    BTU_MUTEX_UNLOCK();

    return retval;
}

UINT16 mico_bt_l2cap_le_connect_req (UINT16 le_psm, BD_ADDR p_bd_addr,
                                mico_bt_ble_address_type_t bd_addr_type,
                                mico_bt_ble_conn_mode_t conn_mode,
                                UINT16 rx_mtu, UINT8 rx_sdu_pool_id,
                                UINT8 req_security, UINT8 req_encr_key_size)
{
    UINT16 retval = 0;

    BTU_MUTEX_LOCK();
    retval = L2CA_LeConnectReq (le_psm, p_bd_addr, bd_addr_type, conn_mode, rx_mtu, rx_sdu_pool_id, req_security,
                                req_encr_key_size);
    BTU_MUTEX_UNLOCK();

    return retval;
}

BOOLEAN mico_bt_l2cap_le_connect_rsp (BD_ADDR p_bd_addr, UINT8 id, UINT16 lcid, UINT16 result,
                                     UINT16 rx_mtu, UINT8 rx_sdu_pool_id)
{
    BOOLEAN retval = FALSE;

    BTU_MUTEX_LOCK();
    retval = L2CA_LeConnectRsp (p_bd_addr, id, lcid, result, rx_mtu, rx_sdu_pool_id);
    BTU_MUTEX_UNLOCK();

    return retval;
}

BOOLEAN mico_bt_l2cap_le_disconnect_req (UINT16 lcid)
{
    BOOLEAN retval = FALSE;

    BTU_MUTEX_LOCK();
    retval = L2CA_LeDisconnectReq (lcid);
    BTU_MUTEX_UNLOCK();

    return retval;
}

BOOLEAN mico_bt_l2cap_le_disconnect_rsp (UINT16 lcid)
{
    BOOLEAN retval = FALSE;

    BTU_MUTEX_LOCK();
    retval = L2CA_LeDisconnectRsp (lcid);
    BTU_MUTEX_UNLOCK();

    return retval;
}

BOOLEAN  mico_bt_l2cap_le_set_user_congestion (UINT16 lcid, BOOLEAN is_congested)
{
    BOOLEAN retval = FALSE;

    BTU_MUTEX_LOCK();
    retval = L2CA_LeSetUserCongestion (lcid, is_congested);
    BTU_MUTEX_UNLOCK();

    return retval;
}

UINT16 mico_bt_l2cap_le_get_peer_mtu (UINT16 lcid)
{
    UINT16 retval = 0;

    BTU_MUTEX_LOCK();
    retval = L2CA_LeGetPeerMTU (lcid);
    BTU_MUTEX_UNLOCK();

    return retval;
}

UINT16 mico_bt_l2cap_le_determ_secur_rsp (BD_ADDR bd_addr, UINT8 req_secur, UINT8 req_encr_key_size)
{
    UINT16 retval = 0;

    BTU_MUTEX_LOCK();
    retval = L2CA_LeDetermSecurRsp (bd_addr, req_secur, req_encr_key_size);
    BTU_MUTEX_UNLOCK();

    return retval;
}
