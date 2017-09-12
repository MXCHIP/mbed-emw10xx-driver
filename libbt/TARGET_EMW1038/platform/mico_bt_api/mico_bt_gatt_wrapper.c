/*****************************************************************************
**
**  Name:          mico_bt_gatt_wrapper.c
**
**  Description:   Auto-generated mico_bt_* API wrappers
**
**  Copyright (c) 2014, Broadcom Corp, All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
******************************************************************************/


#include "bt_target.h"
#include "btu.h"
#include "gatt_api.h"
#include "mico_bt_int.h"

mico_bt_gatt_status_t mico_bt_gatt_send_indication (uint16_t conn_id, uint16_t attr_handle, uint16_t *val_len, uint8_t *p_val )
{
    tGATT_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = GATTS_HandleValueIndication (conn_id, attr_handle, val_len, p_val );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

mico_bt_gatt_status_t mico_bt_gatt_send_notification (uint16_t conn_id, uint16_t attr_handle, uint16_t *val_len, uint8_t *p_val )
{
    tGATT_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = GATTS_HandleValueNotification (conn_id, attr_handle, val_len, p_val);
    BTU_MUTEX_UNLOCK();

    return (retval);
}

mico_bt_gatt_status_t mico_bt_gatt_send_response(mico_bt_gatt_status_t status, uint16_t conn_id,
    uint16_t attr_handle, uint16_t attr_len, uint16_t offset, uint8_t* p_attr)
{
    tGATT_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = GATTS_SendRsp (conn_id, status, attr_handle, attr_len, offset, p_attr);
    BTU_MUTEX_UNLOCK();

    return (retval);
}

tGATT_STATUS mico_bt_gatt_db_init (UINT8 *p_gatt_db, UINT16 size)
{
    tGATT_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = GATTS_DbInit (p_gatt_db, size);
    BTU_MUTEX_UNLOCK();

    return (retval);
}

tGATT_STATUS mico_bt_gatt_configure_mtu (UINT16 conn_id, UINT16 mtu)
{
    tGATT_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = GATTC_ConfigureMTU (conn_id, mtu);
    BTU_MUTEX_UNLOCK();

    return (retval);
}

tGATT_STATUS mico_bt_gatt_send_discover (UINT16 conn_id, tGATT_DISC_TYPE disc_type, tGATT_DISC_PARAM *p_param )
{
    tGATT_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = GATTC_Discover (conn_id, disc_type, p_param );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

tGATT_STATUS mico_bt_gatt_send_read (UINT16 conn_id, tGATT_READ_TYPE type, tGATT_READ_PARAM *p_read)
{
    tGATT_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = GATTC_Read (conn_id, type, p_read );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

tGATT_STATUS mico_bt_gatt_send_write (UINT16 conn_id, tGATT_WRITE_TYPE type, tGATT_VALUE *p_write)
{
    tGATT_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = GATTC_Write (conn_id, type, p_write );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

tGATT_STATUS mico_bt_gatt_send_execute_write (UINT16 conn_id, BOOLEAN is_execute)
{
    tGATT_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = GATTC_ExecuteWrite (conn_id, is_execute );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

tGATT_STATUS mico_bt_gatt_send_indication_confirm (UINT16 conn_id, UINT16 handle)
{
    tGATT_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = GATTC_SendHandleValueConfirm (conn_id, handle );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

tGATT_STATUS mico_bt_gatt_disconnect (UINT16 conn_id)
{
    tGATT_STATUS retval;

    BTU_MUTEX_LOCK();
    retval = GATT_Disconnect (conn_id );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

