/*****************************************************************************
**
**  Name:          mico_bt_rfcomm_wrapper.c
**
**  Description:   Auto-generated mico_bt_* API wrappers
**
**  Copyright (c) 2014, Broadcom Corp, All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
******************************************************************************/
#include "bt_target.h"
#include "btu.h"
#include "rfc_port_api.h"

int mico_bt_rfcomm_create_connection (UINT16 uuid, UINT8 scn, BOOLEAN is_server, UINT16 mtu, BD_ADDR bd_addr, UINT16 *p_handle, tPORT_CALLBACK *p_mgmt_cb)
{
    int retval;

    BTU_MUTEX_LOCK();
    retval = RFCOMM_CreateConnection (uuid, scn, is_server, mtu, bd_addr,p_handle, p_mgmt_cb );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

int mico_bt_rfcomm_remove_connection (UINT16 handle, BOOLEAN remove_server)
{
    int retval;

    BTU_MUTEX_LOCK();
    if (remove_server)
    {
        retval = RFCOMM_RemoveServer( handle );
    }
    else
    {
        retval =  RFCOMM_RemoveConnection( handle );
    }
    BTU_MUTEX_UNLOCK();

    return (retval);
}

int mico_bt_rfcomm_set_event_callback (UINT16 port_handle, tPORT_CALLBACK *p_port_cb)
{
    int retval;

    BTU_MUTEX_LOCK();
    retval = PORT_SetEventCallback (port_handle, p_port_cb );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

int mico_bt_rfcomm_set_data_callback (UINT16 port_handle, tPORT_DATA_CALLBACK *p_cb)
{
    int retval;

    BTU_MUTEX_LOCK();
    retval = PORT_SetDataCallback (port_handle, p_cb );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

int mico_bt_rfcomm_set_event_mask (UINT16 port_handle, UINT32 mask)
{
    int retval;

    BTU_MUTEX_LOCK();
    retval = PORT_SetEventMask (port_handle, mask );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

int mico_bt_rfcomm_control (UINT16 handle, UINT8 signal)
{
    int retval;

    BTU_MUTEX_LOCK();
    retval = PORT_Control (handle, signal );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

int mico_bt_rfcomm_flow_control (UINT16 handle, BOOLEAN enable)
{
    int retval;

    BTU_MUTEX_LOCK();
    retval = PORT_FlowControl (handle, enable );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

int mico_bt_rfcomm_write_data (UINT16 handle, char *p_data, UINT16 max_len, UINT16 *p_len)
{
    int retval;

    BTU_MUTEX_LOCK();
    retval = PORT_WriteData (handle, p_data, max_len, p_len );
    BTU_MUTEX_UNLOCK();

    return (retval);
}
