/*****************************************************************************
**
**  Name:          mico_bt_sdp_wrapper.c
**
**  Description:   Auto-generated mico_bt_* API wrappers
**
**  Copyright (c) 2014, Broadcom Corp, All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
******************************************************************************/
#include "bt_target.h"
#include "btu.h"
#include "sdp_api.h"

BOOLEAN mico_bt_sdp_init_discovery_db (tSDP_DISCOVERY_DB *p_db, UINT32 len, UINT16 num_uuid, tSDP_UUID *p_uuid_list, UINT16 num_attr, UINT16 *p_attr_list)
{
    BOOLEAN retval;

    BTU_MUTEX_LOCK();
    retval = SDP_InitDiscoveryDb (p_db, len, num_uuid, p_uuid_list, num_attr, p_attr_list );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

BOOLEAN mico_bt_sdp_cancel_service_search (tSDP_DISCOVERY_DB *p_db)
{
    BOOLEAN retval;

    BTU_MUTEX_LOCK();
    retval = SDP_CancelServiceSearch (p_db );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

BOOLEAN mico_bt_sdp_service_search_request (UINT8 *p_bd_addr, tSDP_DISCOVERY_DB *p_db, tSDP_DISC_CMPL_CB *p_cb)
{
    BOOLEAN retval;

    BTU_MUTEX_LOCK();
    retval = SDP_ServiceSearchRequest (p_bd_addr, p_db, p_cb );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

BOOLEAN mico_bt_sdp_service_search_attribute_request (UINT8 *p_bd_addr, tSDP_DISCOVERY_DB *p_db, tSDP_DISC_CMPL_CB *p_cb)
{
    BOOLEAN retval;

    BTU_MUTEX_LOCK();
    retval = SDP_ServiceSearchAttributeRequest (p_bd_addr, p_db, p_cb );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

tSDP_DISC_REC* mico_bt_sdp_find_attribute_in_db (tSDP_DISCOVERY_DB *p_db, UINT16 attr_id, tSDP_DISC_REC *p_start_rec)
{
    tSDP_DISC_REC* retval;

    BTU_MUTEX_LOCK();
    retval = SDP_FindAttributeInDb (p_db, attr_id, p_start_rec );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

tSDP_DISC_ATTR* mico_bt_sdp_find_attribute_in_rec (tSDP_DISC_REC *p_rec, UINT16 attr_id)
{
    tSDP_DISC_ATTR* retval;

    BTU_MUTEX_LOCK();
    retval = SDP_FindAttributeInRec (p_rec, attr_id );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

tSDP_DISC_REC* mico_bt_sdp_find_service_in_db (tSDP_DISCOVERY_DB *p_db, UINT16 service_uuid, tSDP_DISC_REC *p_start_rec)
{
    tSDP_DISC_REC* retval;

    BTU_MUTEX_LOCK();
    retval = SDP_FindServiceInDb (p_db, service_uuid, p_start_rec );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

tSDP_DISC_REC* mico_bt_sdp_find_service_uuid_in_db (tSDP_DISCOVERY_DB *p_db, tBT_UUID *p_uuid, tSDP_DISC_REC *p_start_rec)
{
    tSDP_DISC_REC* retval;

    BTU_MUTEX_LOCK();
    retval = SDP_FindServiceUUIDInDb (p_db, p_uuid, p_start_rec );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

BOOLEAN mico_bt_sdp_find_protocol_list_elem_in_rec (tSDP_DISC_REC *p_rec, UINT16 layer_uuid, tSDP_PROTOCOL_ELEM *p_elem)
{
    BOOLEAN retval;

    BTU_MUTEX_LOCK();
    retval = SDP_FindProtocolListElemInRec (p_rec, layer_uuid, p_elem );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

BOOLEAN mico_bt_sdp_find_protocol_lists_elem_in_rec (tSDP_DISC_REC *p_rec, UINT16 layer_uuid, tSDP_PROTOCOL_ELEM *p_elem)
{
    BOOLEAN retval;

    BTU_MUTEX_LOCK();
    retval = SDP_FindAddProtoListsElemInRec (p_rec, layer_uuid, p_elem );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

BOOLEAN mico_bt_sdp_find_profile_version_in_rec (tSDP_DISC_REC *p_rec, UINT16 profile_uuid, UINT16 *p_version)
{
    BOOLEAN retval;

    BTU_MUTEX_LOCK();
    retval = SDP_FindProfileVersionInRec (p_rec, profile_uuid, p_version );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

BOOLEAN mico_bt_sdp_find_service_uuid_in_rec (tSDP_DISC_REC *p_rec, tBT_UUID *p_uuid)
{
    BOOLEAN retval;

    BTU_MUTEX_LOCK();
    retval = SDP_FindServiceUUIDInRec (p_rec, p_uuid );
    BTU_MUTEX_UNLOCK();

    return (retval);
}

