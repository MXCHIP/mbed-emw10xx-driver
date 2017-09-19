/*****************************************************************************
**                                                                           *
**  Name:          mico_bt_main.c                                           *
**                                                                           *
**  Description:
**                                                                           *
**  Copyright (c) 2013, Broadcom Corp., All Rights Reserved.                 *
******************************************************************************/
#include "mico_rtos.h"
#include "bt_target.h"
#include "userial.h"
#include "btu.h"
#include "brcm_api.h"
#include "btm_cfg.h"
#include "btm_api.h"


#if (defined(BTA_INCLUDED) && (BTA_INCLUDED == TRUE))
#include "bta_sys_co.h"
#include "bta_sys_ci.h"
#endif

#if (defined(BT_USE_TRACES) && (BT_USE_TRACES == TRUE))
#define BTE_BTU_STACK_SIZE        0x1800 // 0x1800 used to DEBUG Trace
#define BTE_HCISU_STACK_SIZE      0x1000 // 0x1000 used to DEBUG Trace
#else
#define BTE_BTU_STACK_SIZE        0xA00   // 0x1800 Changed by William to save ram   0xA00
#define BTE_HCISU_STACK_SIZE      0x500   // 0x1000 Changed by William to save ram   0x500
#endif
#define BTE_CODEC_STACK_SIZE      0x1000

void btu_task(UINT32 param);
void bte_hcisu_task(UINT32 param);
void bta_audio_codec_task(UINT32 param);

/******************************************************
 *               Variable Definitions
 ******************************************************/

mico_mutex_t global_trace_mutex;

/* H4 Configuration for BTE */
#if defined(HCISU_H4_INCLUDED) && HCISU_H4_INCLUDED == TRUE
#include "hcisu.h"
#include "userial.h"
#include "hcis_h4.h"
extern const tHCISU_IF hcisu_h4;
static const tHCIS_H4_CFG bte_hcisu_h4_cfg =
{
    USERIAL_HCI_PORT,
    USERIAL_BAUD_115200,
    USERIAL_DATABITS_8 | USERIAL_PARITY_NONE | USERIAL_STOPBITS_1,
#if defined (SLIP_INCLUDED) && SLIP_INCLUDED == TRUE
    HCISU_H4_CFG_FL_SLIP,
//    0,
#else
    0,
#endif

};
#endif /* HCISU_H4_INCLUDED */

/* global tracing variable */
#if (defined(BT_TRACE_PROTOCOL) && (BT_TRACE_PROTOCOL == TRUE))
#include "testercfg.h"
CTesterCfg tester_cfg;
#endif 

int ScrProtocolTraceFlag = 0xFFFFFFFF;


#if (defined(BTA_INCLUDED) && (BTA_INCLUDED == TRUE))
/*******************************************************************************
**
** Function         bta_sys_hw_co_enable
**
** Description      This function is called by the stack to power up the HW
**
** Returns          void
**
*******************************************************************************/
void bta_sys_hw_co_enable( tBTA_SYS_HW_MODULE module )
{
    /* if no client/server asynchronous system like linux-based OS, directly call the ci here */
    bta_sys_hw_ci_enabled( module );
}

/*******************************************************************************
**
** Function         bta_sys_hw_co_disable
**
** Description     This function is called by the stack to power down the HW
**
** Returns          void
**
*******************************************************************************/
void bta_sys_hw_co_disable( tBTA_SYS_HW_MODULE module )
{
    /* platform specific implementation to power-down the HW */


    /* if no client/server asynchronous system like linux-based OS, directly call the ci here */
    bta_sys_hw_ci_disabled( module );

}
#else /* BTA_INCLUDED */

/* Definitions needed when not using BTA */
UINT8 appl_trace_level = BT_TRACE_LEVEL_DEBUG;

#endif

#ifdef MICO_BT_AUDIO
mico_bt_result_t mico_bt_register_audio_cback(tMICO_BT_AUDIO_CBACK *p_audio_cback)
{
    mico_bt_cb.p_audio_cback = p_audio_cback;
    return MICO_SUCCESS;
}
#endif  /* MICO_BT_AUDIO */

OSStatus mico_bt_stack_init(tBTM_EVENT_CBACK *p_bt_management_cback,
                                   const mico_bt_cfg_settings_t     *p_bt_cfg_settings,
                                   const mico_bt_cfg_buf_pool_t     mico_bt_cfg_buf_pools[MICO_BT_CFG_NUM_BUF_POOLS])
{
    OSStatus err = kNoErr;

    //ScrProtocolTraceFlag &= ~SCR_PROTO_TRACE_SDP;

    err = mico_rtos_init_mutex( &global_trace_mutex );
    if ( err != kNoErr )
    {
        APPL_TRACE_ERROR1("application_start - failed:mico_rtos_init_mutex ret=0x%x", err );
    }

#if (defined(BT_TRACE_PROTOCOL) && (BT_TRACE_PROTOCOL == TRUE))
    memset( &tester_cfg, 0, sizeof(CTesterCfg));
#endif 

    /* Configure stack */
    BTM_CfgStack(p_bt_cfg_settings, mico_bt_cfg_buf_pools);

    /* initialize OS */
    //APPL_TRACE_DEBUG0("Call GKI_init" );
    GKI_init( );

    /* Initialize BTE control block */
    BTE_Init(p_bt_management_cback);

    /* Implement platform HW initialization in this macro - can be BTE_InitHW function */
    //APPL_TRACE_DEBUG0("Call BTE_InitHW" );
    /* initialize gpio devices */
    UPIO_Init( NULL );

    /* initialize serial driver */
    USERIAL_Init( NULL );

#if ((defined(HCISU_H4_INCLUDED) && (HCISU_H4_INCLUDED == TRUE)) && !defined(BTE_SIM_APP))  /* For BTE_SIM (Insight) apps, transport is configured in btstk.dll */
    /* Initialize pointer to function that sends hci commands and data to the transport */
    p_hcisu_if  = (tHCISU_IF *)&hcisu_h4;
    p_hcisu_cfg = (void *)&bte_hcisu_h4_cfg;
#endif

    /* create tasks */
    APPL_TRACE_DEBUG0("Call BTE_CreateTasks" );
    GKI_create_task( btu_task, BTU_TASK, (INT8 *)"BTU" , NULL, BTE_BTU_STACK_SIZE);
    GKI_create_task( bte_hcisu_task, HCISU_TASK,(INT8 *)"HCISU", NULL , BTE_HCISU_STACK_SIZE);

#ifdef MICO_BT_AUDIO
    GKI_create_task( bta_audio_codec_task, CODEC_TASK, (INT8 *)"CODEC_TASK", NULL, BTE_CODEC_STACK_SIZE);
#endif

    /* start tasks */
    APPL_TRACE_DEBUG0("Call GKI_run" );
    GKI_run( 0 );

    return MICO_BT_SUCCESS;
}

