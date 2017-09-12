/*****************************************************************************
**                                                                           *
**  Name:          wiced_stubs.c                                             *
**                                                                           *
**  Description:   Stubbed functions (not currently needed on this platform  *
**                                                                           *
**  Copyright (c) 2013, Broadcom Corp., All Rights Reserved.                 *
******************************************************************************/
#if BT_TRACE_PROTOCOL==TRUE

#include "bt_target.h"

void DispLMDiagEvent (BT_HDR *p_hdr)
{
}

void thru_acl_data(UINT16 len, UINT8 *p, BOOLEAN is_rcvd)
{
}

void thru_acl_change(UINT8 *p, BOOLEAN is_new)
{
}

char *HCIGetVendorSpecDesc(UINT16 opcode)
{
    return (NULL);
}

UINT8 *scru_dump_hex(UINT8 *p, char *p_title, UINT16 len, UINT32 trace_layer, UINT32 trace_type)
{
    return p;
}

void DispRFCOMMFrame (BT_HDR *p_buf, BOOLEAN tx)
{

}

void DispSmpMsg (BT_HDR *p_buf, BOOLEAN is_recv)
{

}
#endif
