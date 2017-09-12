/*****************************************************************************
**
**  Name:          mico_bt_ble_wrapper.c
**
**  Description:   Auto-generated mico_bt_* API wrappers
**
**  Copyright (c) 2014, Broadcom Corp, All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
******************************************************************************/
#include <string.h>
#include <stdio.h>

#include "mico_bt_constants.h"
#include "btm_api.h"
#include "btm_ble_api.h"
#include "brcm_ble.h"
#include "btu.h"
#include "mico_bt_int.h"
#include "bt_types.h"
mico_bt_dev_status_t mico_bt_start_advertisements(mico_bt_ble_advert_mode_t avert_mode, mico_bt_ble_address_type_t directed_advertisement_bdaddr_type, mico_bt_device_address_ptr_t directed_advertisement_bdaddr_ptr)
{
    tBTM_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = BTM_BleStartAdvertisements (avert_mode, directed_advertisement_bdaddr_type, directed_advertisement_bdaddr_ptr);
    BTU_MUTEX_UNLOCK();

    return (retval);
}

mico_bt_ble_advert_mode_t mico_bt_ble_get_current_advert_mode(void)
{
    mico_bt_ble_advert_mode_t retval;

    BTU_MUTEX_LOCK();
    retval = BTM_BleGetAdvertMode ();
    BTU_MUTEX_UNLOCK();

    return (retval);
}




tBTM_STATUS mico_bt_ble_set_advertisement_data (mico_bt_ble_advert_mask_t data_mask, mico_bt_ble_advert_data_t *p_data)
{
    tBTM_STATUS retval;
	

    BTU_MUTEX_LOCK();
//	printf("mico_bt_ble_set_advertisement_data:oscar ->p_data->p_services->num_service=%d , p_data->p_services->p_uuid=%x \n",p_data->p_services->num_service,*(p_data->p_services->p_uuid));

	retval = BTM_BleWriteAdvData (data_mask, p_data );
	
    BTU_MUTEX_UNLOCK();

    return (retval);
}

mico_bt_dev_status_t mico_bt_ble_set_scan_response_data(mico_bt_ble_advert_mask_t data_mask, mico_bt_ble_advert_data_t *p_data)
{
    tBTM_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = BTM_BleWriteScanRsp (data_mask, p_data );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

tBTM_STATUS mico_bt_ble_observe (BOOLEAN start, UINT8 duration, mico_bt_ble_scan_result_cback_t *p_scan_result_cback)
{
    tBTM_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = BTM_BleObserve (start, duration, p_scan_result_cback);
    BTU_MUTEX_UNLOCK();

    return (retval);
}

mico_bt_dev_status_t  mico_bt_ble_scan (mico_bt_ble_scan_type_t scan_type, mico_bool_t duplicate_filter_enable, mico_bt_ble_scan_result_cback_t *p_scan_result_cback)
{
    tBTM_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = BTM_BleScan (scan_type, (BOOLEAN) duplicate_filter_enable, p_scan_result_cback);
    BTU_MUTEX_UNLOCK();

    return (retval);
}

mico_bt_ble_scan_type_t mico_bt_ble_get_current_scan_state(void)
{
    mico_bt_ble_scan_type_t retval;

    BTU_MUTEX_LOCK();
    retval = BTM_BleGetCurrentScanState ();
    BTU_MUTEX_UNLOCK();

    return (retval);
}

void mico_bt_ble_security_grant(BD_ADDR bd_addr, UINT8 res)
{
    BTU_MUTEX_LOCK();
    BTM_SecurityGrant (bd_addr, res );
    BTU_MUTEX_UNLOCK();
    return;
}

mico_bool_t mico_bt_ble_data_signature (mico_bt_device_address_t bd_addr, uint8_t *p_text, uint16_t len,
                                             mico_dev_ble_signature_t signature)
{
    mico_bool_t retval;

    BTU_MUTEX_LOCK();
    retval = BTM_BleDataSignature (bd_addr, p_text, len, signature);
    BTU_MUTEX_UNLOCK();

    return (retval);
}

mico_bool_t mico_bt_ble_verify_signature (mico_bt_device_address_t bd_addr, uint8_t *p_orig,
                                            uint16_t len, uint32_t counter,
                                            uint8_t *p_comp)
{
    mico_bool_t retval;

    BTU_MUTEX_LOCK();
    retval = BTM_BleVerifySignature (bd_addr, p_orig, len, counter,p_comp);
    BTU_MUTEX_UNLOCK();

    return (retval);
}


mico_bool_t mico_bt_ble_set_background_connection_type (mico_bt_ble_conn_type_t conn_type, mico_bt_ble_selective_conn_cback_t *p_select_cback)
{
    mico_bool_t retval;

    BTU_MUTEX_LOCK();
    retval = BTM_BleSetBgConnType (conn_type, p_select_cback );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

mico_bool_t mico_bt_ble_update_background_connection_device(mico_bool_t add_remove, mico_bt_device_address_t remote_bda)
{
    mico_bool_t retval;

    BTU_MUTEX_LOCK();
    retval = BTM_BleUpdateBgConnDev (add_remove, remote_bda );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

mico_bool_t mico_bt_ble_get_background_connection_device_size(uint8_t *size)
{
    BTU_MUTEX_LOCK();
    *size = BTM_BleGetWhiteListSize(BTM_BLE_BRCM_WL_ATTR_INIT);
    BTU_MUTEX_UNLOCK();
    return TRUE;
}

UINT8* mico_bt_ble_check_advertising_data ( UINT8 *p_adv, UINT8 type, UINT8 *p_length)
{
    UINT8* retval;

    //BTU_MUTEX_LOCK();
    retval = BTM_CheckAdvData (p_adv, type, p_length );
    //BTU_MUTEX_UNLOCK();

    return (retval);
}


#if BTM_BLE_PRIVACY_SPT == TRUE
void mico_bt_ble_enable_privacy (mico_bool_t enable)
{
    BTU_MUTEX_LOCK();
    BTM_BleConfigPrivacy (enable);
    BTU_MUTEX_UNLOCK();
}

void mico_bt_ble_enable_mixed_privacy_mode(mico_bool_t mixed_on)
{
    BTU_MUTEX_LOCK();
    BTM_BleEnableMixedPrivacyMode (mixed_on);
    BTU_MUTEX_UNLOCK();
}
#endif

mico_bool_t mico_bt_ble_update_advertising_white_list(mico_bool_t add, const mico_bt_device_address_t remote_bda)
{
    mico_bool_t retval;

    BTU_MUTEX_LOCK();
    retval = BTM_BleUpdateAdvWhitelist (add, (UINT8 *)remote_bda);
    BTU_MUTEX_UNLOCK();

    return (retval);
}

mico_bool_t mico_bt_ble_update_advertisement_filter_policy(mico_bt_ble_advert_filter_policy_t advertising_policy)
{
    mico_bool_t retval;

    BTU_MUTEX_LOCK();
    retval = BTM_BleUpdateAdvFilterPolicy(advertising_policy);
    BTU_MUTEX_UNLOCK();

    return (retval);
}

mico_bool_t mico_bt_ble_get_advertisement_white_list_size(uint8_t *size)
{
    BTU_MUTEX_LOCK();
    *size = BTM_BleGetWhiteListSize(BTM_BLE_BRCM_WL_ATTR_ADV);
    BTU_MUTEX_UNLOCK();
    return TRUE;
}

mico_bool_t mico_bt_ble_get_white_list_capability(uint8_t *size)
{
    BTU_MUTEX_LOCK();
    *size = BTM_BleGetWhiteListCapability();
    BTU_MUTEX_UNLOCK();
    return TRUE;
}

mico_bool_t mico_bt_ble_clear_white_list(void)
{
    BTU_MUTEX_LOCK();
    BTM_BleClearWhiteList();
    BTU_MUTEX_UNLOCK();
    return TRUE;
}


mico_bt_result_t mico_bt_dev_add_device_to_address_resolution_db(mico_bt_device_link_keys_t *p_link_keys)
{
    tBTM_STATUS retval = (tBTM_STATUS)MICO_BT_UNSUPPORTED;
    BTU_MUTEX_LOCK();
    retval = BTM_BleAddAddressResolutionDB(p_link_keys);
    BTU_MUTEX_UNLOCK();
    return ((mico_bt_result_t)retval);
}



mico_bt_result_t mico_bt_dev_remove_device_from_address_resolution_db(mico_bt_device_link_keys_t *p_link_keys)
{
    tBTM_STATUS retval = (tBTM_STATUS)MICO_BT_UNSUPPORTED;
    BTU_MUTEX_LOCK();
    BTM_BleRemoveAddressResolutionDB(p_link_keys);
    BTU_MUTEX_UNLOCK();

    return ((mico_bt_result_t)retval);
}

mico_bool_t mico_bt_ble_get_security_state (mico_bt_device_address_t bd_addr, uint8_t *p_le_sec_flags, uint8_t *p_le_key_size)
{
    mico_bool_t retval;

    BTU_MUTEX_LOCK();
    retval = (mico_bool_t)BTM_GetLeSecurityState(bd_addr, p_le_sec_flags, p_le_key_size);
    BTU_MUTEX_UNLOCK();

    return retval;
}
