/*****************************************************************************
**
**  Name:          mico_bt_dev_wrapper.c
**
**  Description:   Auto-generated mico_bt_* API wrappers
**
**  Copyright (c) 2014, Broadcom Corp, All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
******************************************************************************/
#include <string.h>
#include "bt_target.h"
#include "btu.h"
#include "btm_api.h"
#include "btm_ble_api.h"
#include "btm_sec_nvram.h"
#include "mico_bt_int.h"

typedef void (mico_bt_dev_cmpl_cback_t)(void *pl);

void mico_bt_dev_read_local_addr (BD_ADDR bd_addr)
{
    BTU_MUTEX_LOCK();
    BTM_GetLocalDeviceAddr (bd_addr );
    BTU_MUTEX_UNLOCK();
    return;
}

mico_bt_result_t  mico_bt_start_inquiry (mico_bt_dev_inq_parms_t *p_inqparms, mico_bt_inquiry_result_cback_t *p_inquiry_result_cback)
{
    tBTM_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = BTM_StartInquiry (p_inqparms, p_inquiry_result_cback);
    BTU_MUTEX_UNLOCK();

    return (mico_bt_result_t)(retval);
}

tBTM_STATUS mico_bt_cancel_inquiry (void)
{
    tBTM_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = BTM_CancelInquiry ();
    BTU_MUTEX_UNLOCK();

    return (retval);
}

mico_bt_dev_status_t mico_bt_dev_set_advanced_connection_params (mico_bt_dev_inquiry_scan_result_t *p_inquiry_scan_result)
{
    mico_bt_dev_status_t retval = BTM_SUCCESS;

    BTU_MUTEX_LOCK();
    retval = BTM_SetAdvancedConnectionParams (p_inquiry_scan_result);
    BTU_MUTEX_UNLOCK();

    return (retval);
}

mico_bt_dev_status_t mico_bt_dev_set_local_device_address(mico_bt_device_address_t bdaddr, 
                                                          mico_bt_dev_cmpl_cback_t *p_cback)
{
    mico_bt_dev_status_t retval;

    BTU_MUTEX_LOCK();
    retval = BTM_SetLocalDeviceAddr(bdaddr, p_cback);
    BTU_MUTEX_UNLOCK();

    return (retval);
}

mico_bt_dev_status_t mico_bt_dev_vendor_specific_command (uint16_t opcode, uint8_t param_len, uint8_t *p_param_buf,
                                mico_bt_dev_vendor_specific_command_complete_cback_t *p_cback)
{
    mico_bt_dev_status_t retval;

    BTU_MUTEX_LOCK();
    retval = BTM_VendorSpecificCommand (opcode, param_len, p_param_buf, p_cback);
    BTU_MUTEX_UNLOCK();

    return (retval);
}

tBTM_STATUS mico_bt_dev_set_discoverability (UINT8 inq_mode, UINT16 window, UINT16 interval)
{
    tBTM_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = BTM_SetDiscoverability (inq_mode, window, interval );
    BTU_MUTEX_UNLOCK();

    return (retval);
}


tBTM_STATUS mico_bt_dev_set_connectability (UINT8 page_mode, UINT16 window, UINT16 interval)
{
    tBTM_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = BTM_SetConnectability (page_mode, window, interval );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

mico_bt_dev_status_t mico_bt_dev_register_connection_status_change(mico_bt_connection_status_change_cback_t *p_mico_bt_connection_status_change_cback)
{
    mico_bt_dev_status_t retval;

    BTU_MUTEX_LOCK();
    retval = BTM_AclRegisterForChanges(*p_mico_bt_connection_status_change_cback);
    BTU_MUTEX_UNLOCK();

    return (retval);
}


tBTM_STATUS mico_bt_dev_set_sniff_mode (BD_ADDR remote_bda, uint16_t min_period,
                                             uint16_t max_period, uint16_t attempt,
                                             uint16_t timeout)
{
    mico_bt_dev_status_t retval;

    BTU_MUTEX_LOCK();
    retval = BTM_SetSniffMode(remote_bda, min_period, max_period, attempt, timeout);
    BTU_MUTEX_UNLOCK();

    return (retval);
}

tBTM_STATUS mico_bt_dev_cancel_sniff_mode (BD_ADDR remote_bda)
{
    mico_bt_dev_status_t retval;

    BTU_MUTEX_LOCK();
    retval = BTM_CancelSniffMode(remote_bda);
    BTU_MUTEX_UNLOCK();

    return (retval);
}

mico_bt_result_t mico_bt_dev_set_sniff_subrating (mico_bt_device_address_t remote_bda, uint16_t max_latency,
                              uint16_t min_remote_timeout, uint16_t min_local_timeout)
{
    mico_bt_result_t retval = MICO_BT_ILLEGAL_ACTION;
    tACL_CONN   *p_acl;

    BTU_MUTEX_LOCK();

    /* Get HCI handle for requested remote_bda */
    if ((p_acl = btm_bda_to_acl(remote_bda, BT_TRANSPORT_BR_EDR)) != NULL)
    {
        if (btsnd_hcic_sniff_sub_rate(p_acl->hci_handle, max_latency, min_remote_timeout, min_local_timeout))
            retval = MICO_BT_SUCCESS;
    }

    BTU_MUTEX_UNLOCK();

    return (retval);
}

tBTM_STATUS mico_bt_dev_read_rssi (BD_ADDR remote_bda, tBT_TRANSPORT transport, tBTM_CMPL_CB *p_cb)
{
    tBTM_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = BTM_ReadRSSI (remote_bda, transport, p_cb);
    BTU_MUTEX_UNLOCK();

    return (retval);

}

tBTM_STATUS mico_bt_dev_read_tx_power (BD_ADDR remote_bda, tBT_TRANSPORT transport, tBTM_CMPL_CB *p_cb)
{
    tBTM_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = BTM_ReadTxPower (remote_bda,  transport,  p_cb);
    BTU_MUTEX_UNLOCK();

    return (retval);
}

mico_bt_result_t mico_bt_dev_write_eir (uint8_t *p_buff, uint16_t len)
{
    BT_HDR *p_buf;
    UINT8 *p;
    tBTM_STATUS retval = BTM_NO_RESOURCES;

    BTU_MUTEX_LOCK();

    if ((p_buf = (BT_HDR *)GKI_getbuf (sizeof(BT_HDR) + len + HCIC_PREAMBLE_SIZE)) != NULL)
    {
        p = (UINT8 *)(p_buf + 1) + HCIC_PREAMBLE_SIZE;
        memcpy(p, p_buff, len);
        p_buf->offset = HCIC_PREAMBLE_SIZE;
        p_buf->len = len;

        retval = BTM_WriteEIR (p_buf);
    }

    BTU_MUTEX_UNLOCK();

    return (mico_bt_result_t)(retval);
}


mico_bt_result_t mico_bt_dev_set_pin_code_only(mico_bool_t pin_code_only)
{
    tBTM_STATUS retval;
    retval = BTM_SetPinCodeOnly(pin_code_only);
    return retval;
}

void mico_bt_dev_set_pin_type(mico_bool_t fixed_pin, uint8_t *pin_code, uint8_t pin_code_len)
{
    BTU_MUTEX_LOCK();
    BTM_SetPinType(fixed_pin, pin_code, pin_code_len);
    BTU_MUTEX_UNLOCK();
}

void mico_bt_dev_pin_code_reply (BD_ADDR bd_addr, tBTM_STATUS res, UINT8 pin_len, UINT8 *p_pin)
{
    BTU_MUTEX_LOCK();
    BTM_PINCodeReply (bd_addr, res, pin_len, p_pin );
    BTU_MUTEX_UNLOCK();
    return;
}

mico_bt_result_t mico_bt_dev_sec_bond (mico_bt_device_address_t bd_addr, mico_bt_ble_address_type_t bd_addr_type, mico_bt_transport_t transport, uint8_t pin_len, uint8_t *p_pin)
{
    tBTM_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = BTM_SecBondByTransport (bd_addr, bd_addr_type, transport, pin_len, p_pin);
    BTU_MUTEX_UNLOCK();

    return (mico_bt_result_t)(retval);
}

tBTM_STATUS mico_bt_dev_sec_bond_cancel (BD_ADDR bd_addr)
{
    tBTM_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = BTM_SecBondCancel (bd_addr );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

tBTM_STATUS mico_bt_dev_set_encryption (BD_ADDR bd_addr, mico_bt_transport_t transport, void *p_ref_data)
{
    tBTM_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = BTM_SetEncryption (bd_addr, transport, p_ref_data);
    BTU_MUTEX_UNLOCK();

    return (retval);
}

void mico_bt_dev_confirm_req_reply (tBTM_STATUS res, BD_ADDR bd_addr)
{
    BTU_MUTEX_LOCK();
    if(BTM_UseLeLink(bd_addr))
    {
        SMP_ConfirmReply(bd_addr, res);
    }
    else
    {
        BTM_ConfirmReqReply (res, bd_addr );
    }
    BTU_MUTEX_UNLOCK();
    return;
}

void mico_bt_dev_pass_key_req_reply (tBTM_STATUS res, BD_ADDR bd_addr, UINT32 passkey)
{
    BTU_MUTEX_LOCK();
    BTM_PasskeyReqReply (res, bd_addr, passkey );
    BTU_MUTEX_UNLOCK();
    return;
}

void mico_bt_dev_send_key_press_notif (BD_ADDR bd_addr, tBTM_SP_KEY_TYPE type)
{
    BTU_MUTEX_LOCK();
    BTM_SendKeypressNotif (bd_addr, type );
    BTU_MUTEX_UNLOCK();
    return;
}

#if (BTM_OOB_INCLUDED == TRUE)
tBTM_STATUS mico_bt_dev_read_local_oob_data (void)
{
    tBTM_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = BTM_ReadLocalOobData ();
    BTU_MUTEX_UNLOCK();

    return (retval);
}

void mico_bt_dev_remote_oob_data_reply (mico_bt_result_t res, mico_bt_device_address_t bd_addr,
                                              mico_bool_t is_extended_oob_data,
                                              BT_OCTET16 c_192, BT_OCTET16 r_192,
                                              BT_OCTET16 c_256, BT_OCTET16 r_256)
{
    BTU_MUTEX_LOCK();
#if (defined(BTM_BR_SC_INCLUDED) && (BTM_BR_SC_INCLUDED == TRUE))
    if (is_extended_oob_data)
        BTM_RemoteOobExtDataReply ((tBTM_STATUS)res, bd_addr, c_192, r_192, c_256, r_256);
    else
#endif
        BTM_RemoteOobDataReply ((tBTM_STATUS)res, bd_addr, c_192, r_192);
    BTU_MUTEX_UNLOCK();
}

uint16_t mico_bt_dev_build_oob_data(uint8_t *p_data, uint16_t max_len,
                                          mico_bool_t is_extended_oob_data,
                                          BT_OCTET16 c_192, BT_OCTET16 r_192,
                                          BT_OCTET16 c_256, BT_OCTET16 r_256)
{
    uint16_t retval;
    BTU_MUTEX_LOCK();
#if (defined(BTM_BR_SC_INCLUDED) && (BTM_BR_SC_INCLUDED == TRUE))
    if (is_extended_oob_data)
        retval = BTM_BuildOobExtData (p_data, max_len, c_192, r_192, c_256, r_256);
    else
#endif
        retval = BTM_BuildOobData (p_data, max_len, c_192, r_192, 0);
    BTU_MUTEX_UNLOCK();

    return (retval);
}

void mico_bt_smp_oob_data_reply(mico_bt_device_address_t bd_addr, mico_bt_result_t res, uint8_t len, uint8_t *p_data)
{
    BTU_MUTEX_LOCK();
    SMP_OobDataReply (bd_addr, (res == MICO_BT_SUCCESS) ? SMP_SUCCESS : SMP_FAIL , len, p_data);
    BTU_MUTEX_UNLOCK();
}

mico_bool_t mico_bt_smp_create_local_sc_oob_data (mico_bt_device_address_t bd_addr, mico_bt_ble_address_type_t bd_addr_type)
{
    mico_bool_t retval;
    tBLE_BD_ADDR dest_addr;

    BTU_MUTEX_LOCK();
    memcpy(dest_addr.bda, bd_addr, BD_ADDR_LEN);
    dest_addr.type = bd_addr_type;
    retval = SMP_CrLocScOobData (&dest_addr);
    BTU_MUTEX_UNLOCK();

    return (retval);

}

void mico_bt_smp_sc_oob_reply (uint8_t *p_oob_data)
{
    BTU_MUTEX_LOCK();
    SMP_SCOobDataReply (p_oob_data);
    BTU_MUTEX_UNLOCK();
}
#endif  /* BTM_OOB_INCLUDED */


mico_bt_result_t mico_bt_dev_get_bonded_devices(mico_bt_dev_bonded_device_info_t *p_paired_device_list, uint16_t *p_num_devices)
{
    tBTM_STATUS retval = (tBTM_STATUS)MICO_BT_UNSUPPORTED;
#if defined(BTM_INTERNAL_LINKKEY_STORAGE_INCLUDED) && (BTM_INTERNAL_LINKKEY_STORAGE_INCLUDED == TRUE)
    BTU_MUTEX_LOCK();
    retval = (tBTM_STATUS)p_btm_nvram_access->get_bonded_devices(p_paired_device_list,p_num_devices);
    BTU_MUTEX_UNLOCK();
#endif
    return (mico_bt_result_t)retval;
}

mico_bool_t mico_bt_dev_find_bonded_device( mico_bt_device_address_t bd_addr)
{
    extern mico_bool_t mico_bt_nvram_access_find_device( mico_bt_device_address_t key_bdaddr );

    mico_bool_t retval = 0 ;
#if defined(BTM_INTERNAL_LINKKEY_STORAGE_INCLUDED) && (BTM_INTERNAL_LINKKEY_STORAGE_INCLUDED == TRUE)
    BTU_MUTEX_LOCK();
    retval = mico_bt_nvram_access_find_device(bd_addr);
    BTU_MUTEX_UNLOCK();
#endif
    return (mico_bt_result_t)retval;    
}

mico_bt_result_t mico_bt_dev_delete_bonded_device(mico_bt_device_address_t bd_addr)
{
    tBTM_STATUS retval = (tBTM_STATUS)MICO_BT_UNSUPPORTED;
#if defined(BTM_INTERNAL_LINKKEY_STORAGE_INCLUDED) && (BTM_INTERNAL_LINKKEY_STORAGE_INCLUDED == TRUE)
    mico_bt_device_link_keys_t link_keys;
    memcpy(link_keys.bd_addr, bd_addr, BD_ADDR_LEN);
    BTU_MUTEX_LOCK();
    BTM_BleRemoveAddressResolutionDB(&link_keys);
    retval = (tBTM_STATUS)p_btm_nvram_access->delete_bonded_device(bd_addr);
    BTU_MUTEX_UNLOCK();
#endif
    return (mico_bt_result_t)retval;
}

mico_bt_result_t mico_bt_dev_get_key_by_keytype(mico_bt_device_address_t bd_addr, mico_bt_dev_le_key_type_t key_type, mico_bt_security_key_value_t *p_sec_keys)
{
    tBTM_STATUS retval = (tBTM_STATUS)MICO_BT_UNSUPPORTED;
#if defined(BTM_INTERNAL_LINKKEY_STORAGE_INCLUDED) && (BTM_INTERNAL_LINKKEY_STORAGE_INCLUDED == TRUE)
    BTU_MUTEX_LOCK();
    retval = (tBTM_STATUS)btm_sec_nvram_get_key_by_keytype(bd_addr, key_type, p_sec_keys);
    BTU_MUTEX_UNLOCK();
#endif
    return (mico_bt_result_t)retval;
}


/* Register/over-ride default NVRAM access functions for saving/retrieving link keys. */
mico_bt_result_t mico_bt_nvram_access_register(mico_bt_nvram_access_t *p_nvram_access)
{
#if defined(BTM_INTERNAL_LINKKEY_STORAGE_INCLUDED) && (BTM_INTERNAL_LINKKEY_STORAGE_INCLUDED == TRUE)
    if (p_nvram_access)
    {
        p_btm_nvram_access = p_nvram_access;
        return MICO_BT_SUCCESS;
    }
    else
    {
        return MICO_BT_ILLEGAL_VALUE;
    }
#else
    return MICO_BT_UNSUPPORTED;
#endif
}
