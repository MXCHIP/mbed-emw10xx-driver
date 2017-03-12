/*
 * Copyright 2013, Broadcom Corporation
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */

/** @file
 *  Implements user functions for controlling the Wi-Fi system
 *
 *  This file provides end-user functions which allow actions such as scanning for
 *  Wi-Fi networks, joining Wi-Fi networks, getting the MAC address, etc
 *
 */
#include "Platform/wwd_platform_interface.h"
#include "wwd_management.h"
#include "wwd_wifi.h"
#include "Network/wwd_buffer_interface.h"
#include "Network/wwd_network_constants.h"
#include "internal/wwd_sdpcm.h"
#include "wwd_events.h"
#include "wwd_assert.h"
#include "wwd_wlioctl.h"
#include "internal/wwd_internal.h"
#include "RTOS/wwd_rtos_interface.h"
#include "wwd_bus_protocol.h"
#include <string.h> /* For strlen, stricmp, memcpy. memset */
#include <stddef.h>
#include "internal/bcmendian.h"
#include "internal/wwd_ap.h"
#include "wwd_debug.h"

/******************************************************
 * @cond       Constants
 ******************************************************/

#define SSID_MAX_LEN (32)

/* These are the flags in the BSS Capability Information field as defined in section 7.3.1.4 of IEEE Std 802.11-2007  */
#define DOT11_CAP_ESS                     (0x0001)   /** Extended service set capability */
#define DOT11_CAP_IBSS                    (0x0002)   /** Ad-hoc capability (Independent Basic Service Set) */
#define DOT11_CAP_PRIVACY                 (0x0010)   /** Privacy subfield - indicates data confidentiality is required for all data frames exchanged */
#define DOT11_MNG_RSN_ID                      (48)   /** 802.11 management RSN id */
#define DOT11_MNG_WPA_ID                   (221ul)   /** 802.11 management WPA id */
#define DOT11_MNG_HT_CAP                      (45)   /** 802.11 management HT capabilities id */
#define DOT11_MNG_DS_ID                        (3)
#define WL_CHANSPEC_CHAN_MASK             (0x00ff)
#define LEGACY_WL_BSS_INFO_VERSION           (107)   /** older version of wl_bss_info struct */

#define WICED_CREDENTIAL_TEST_TIMEOUT     (1500)

#define WPA_OUI                     "\x00\x50\xF2"   /** WPA OUI */

#define MAX_SUPPORTED_MCAST_ENTRIES   (10)

#define WLC_EVENT_MSG_LINK      (0x01)

#define JOIN_ASSOCIATED             (uint32_t)(1 << 0)
#define JOIN_AUTHENTICATED          (uint32_t)(1 << 1)
#define JOIN_LINK_READY             (uint32_t)(1 << 2)
#define JOIN_SECURITY_COMPLETE      (uint32_t)(1 << 3)
#define JOIN_COMPLETE               (uint32_t)(1 << 4)
#define JOIN_NO_NETWORKS            (uint32_t)(1 << 5)
#define JOIN_WRONG_KEY              (uint32_t)(1 << 6) // haibo added 

#define DEFAULT_JOIN_TIMEOUT      (7000)

#ifndef DEFAULT_PM2_SLEEP_RET_TIME
#define DEFAULT_PM2_SLEEP_RET_TIME   (40)
#endif

#define PM1_POWERSAVE_MODE          ( 1 )
#define PM2_POWERSAVE_MODE          ( 2 )
#define NO_POWERSAVE_MODE           ( 0 )
#define NULL_FRAMES_WITH_PM_SET_LIMIT ( 100 )

#define VALID_SECURITY_MASK    0x00FFFFFF

#define CHECK_IOCTL_BUFFER( buff )  if ( buff == NULL ) {  wiced_assert("Allocation failed\n", 0 == 1); return WICED_ERROR; }
#define CHECK_RETURN( expr )  { wwd_result_t check_res = (expr); if ( check_res != WWD_SUCCESS ) { wiced_assert("Command failed\n", 0 == 1); return check_res; } }
#define CHECK_RETURN_UNSUPPORTED_OK( expr )  { wwd_result_t check_res = (expr); if ( check_res != WWD_SUCCESS ) { wiced_assert("Command failed\n", check_res == WWD_WLAN_UNSUPPORTED); return check_res; } }
#define CHECK_RETURN_UNSUPPORTED_CONTINUE( expr )  { wwd_result_t check_res = (expr); if ( check_res != WWD_SUCCESS && check_res != WWD_WLAN_UNSUPPORTED ) { wiced_assert("Command failed\n", 0 == 1); return check_res; } }
#define RETURN_WITH_ASSERT( expr )  { wwd_result_t check_res = (expr); wiced_assert("Command failed\n", check_res == WWD_SUCCESS ); return check_res; }
#define PACKET_FILTER_LIST_BUFFER_MAX_LEN ( WICED_LINK_MTU - sizeof(IOVAR_STR_PKT_FILTER_LIST) - IOCTL_OFFSET )

/** @endcond */

/******************************************************
 *             Local Structures
 ******************************************************/

#pragma pack(1)

typedef struct
{
    uint8_t  element_id;
    uint8_t  element_length;
    uint16_t version;
    uint32_t group_key_suite;
    uint16_t pairwise_suite_count;
    uint32_t pairwise_suite_list[1];
} rsn_ie_t;

typedef struct
{
    uint8_t  element_id;
    uint8_t  element_length;
    uint8_t  stuff[6];
    uint8_t  multicast_suite[4];
    uint16_t unicast_count;
    uint8_t  unicast_suit[1][4];
} wpa_ie_t;

typedef struct
{
    uint8_t  element_id;
    uint8_t  element_length;
    uint16_t ht_capabilities_info;
    uint8_t  ampdu_parameters;
    uint8_t  mcs[8];
    uint16_t highest_supported_data_rate;
    uint8_t  tx_supported_mcs_set;
    uint8_t  tx_mcs_info;
    uint16_t ht_extended_capabilities;
    uint32_t transmit_beam_forming_capabilities;
    uint8_t  antenna_selection_capabilities;
} ht_capabilities_ie_t;

typedef struct
{
    uint32_t    entry_count;
    wiced_mac_t macs[1];
} mcast_list_t;

typedef struct
{
    uint32_t cfg;
    uint32_t val;
} bss_setbuf_t;

typedef struct
{
    int32_t     value;
    wiced_mac_t mac_address;
} client_rssi_t;

#pragma pack()

/******************************************************
 *             Static Variables
 ******************************************************/
static int use_5g_join_param = 0;
static wiced_scan_result_callback_t   scan_result_callback;
static wiced_scan_result_t**          scan_result_ptr;

uint32_t wiced_join_status;
const wiced_event_num_t join_events[]  = { WLC_E_SET_SSID, WLC_E_LINK, WLC_E_AUTH, WLC_E_DEAUTH_IND, WLC_E_PSK_SUP, WLC_E_ROAM, WLC_E_NONE };
static const wiced_event_num_t scan_events[]  = { WLC_E_ESCAN_RESULT, WLC_E_NONE };

/* Note: monitor_mode_enabled variable is accessed by SDPCM */
wiced_bool_t monitor_mode_enabled = WICED_FALSE;

static wiced_bool_t wifi_join_aping = WICED_FALSE; // yhb added 

wiced_bool_t wwd_wifi_p2p_go_is_up = WICED_FALSE;

/* Values are in 100's of Kbit/sec (1 = 100Kbit/s). Arranged as:
 * [Bit index]
 *    [0] = 20Mhz only
 *       [0] = Long GI
 *       [1] = Short GI
 *    [1] = 40MHz support
 *       [0] = Long GI
 *       [1] = Short GI
 */
static const uint16_t mcs_data_rate_lookup_table[32][2][2] =
{
    [0 ] = { {  65  ,   72  },  {   135 ,   150 } },
    [1 ] = { {  130 ,   144 },  {   270 ,   300 } },
    [2 ] = { {  195 ,   217 },  {   405 ,   450 } },
    [3 ] = { {  260 ,   289 },  {   540 ,   600 } },
    [4 ] = { {  390 ,   433 },  {   810 ,   900 } },
    [5 ] = { {  520 ,   578 },  {   1080,   1200} },
    [6 ] = { {  585 ,   650 },  {   1215,   1350} },
    [7 ] = { {  650 ,   722 },  {   1350,   1500} },
    [8 ] = { {  130 ,   144 },  {   270 ,   300 } },
    [9 ] = { {  260 ,   289 },  {   540 ,   600 } },
    [10] = { {  390 ,   433 },  {   810 ,   900 } },
    [11] = { {  520 ,   578 },  {   1080,   1200} },
    [12] = { {  780 ,   867 },  {   1620,   1800} },
    [13] = { {  1040,   1156},  {   2160,   2400} },
    [14] = { {  1170,   1300},  {   2430,   2700} },
    [15] = { {  1300,   1444},  {   2700,   3000} },
    [16] = { {  195 ,   217 },  {   405 ,   450 } },
    [17] = { {  390 ,   433 },  {   810 ,   900 } },
    [18] = { {  585 ,   650 },  {   1215,   1350} },
    [19] = { {  780 ,   867 },  {   1620,   1800} },
    [20] = { {  1170,   1300},  {   2430,   2700} },
    [21] = { {  1560,   1733},  {   3240,   3600} },
    [22] = { {  1755,   1950},  {   3645,   4050} },
    [23] = { {  1950,   2167},  {   4050,   4500} },
    [24] = { {  260 ,   288 },  {   540 ,   600 } },
    [25] = { {  520 ,   576 },  {   1080,   1200} },
    [26] = { {  780 ,   868 },  {   1620,   1800} },
    [27] = { {  1040,   1156},  {   2160,   2400} },
    [28] = { {  1560,   1732},  {   3240,   3600} },
    [29] = { {  2080,   2312},  {   4320,   4800} },
    [30] = { {  2340,   2600},  {   4860,   5400} },
    [31] = { {  2600,   2888},  {   5400,   6000} },
};

/******************************************************
 *             Static Function prototypes
 ******************************************************/

static /*@null@*/ uint8_t*        wlu_parse_tlvs           ( /*@returned@*/ uint8_t* tlv_buf, uint32_t buflen, uint32_t key );
static            wiced_bool_t    wlu_is_wpa_ie            ( uint8_t** wpaie, uint8_t** tlvs, uint32_t* tlvs_len );
static /*@null@*/ void*           wiced_join_events_handler( const wiced_event_header_t* event_header, const uint8_t* event_data, /*@returned@*/ void* handler_user_data );
static            void*           scan_result_handler      ( const wiced_event_header_t* event_header, const uint8_t* event_data, /*@returned@*/ void* handler_user_data );
static            wiced_result_t  wiced_wifi_prepare_join  ( wiced_security_t security, /*@unique@*/ const uint8_t* security_key, uint8_t key_length, host_semaphore_type_t* semaphore );

/******************************************************
 *             Function definitions
 ******************************************************/
void set_5g_join_flag(void);

void set_5g_join_flag(void)
{
    use_5g_join_param = 1;
}

/*
 * NOTE: search references of function wlu_get in wl/exe/wlu.c to find what format the returned IOCTL data is.
 */

wiced_result_t wiced_wifi_scan( wiced_scan_type_t                    scan_type,
                                wiced_bss_type_t                     bss_type,
                                const wiced_ssid_t*                  optional_ssid,
                                const wiced_mac_t*                   optional_mac,
                                /*@unique@*/ const uint16_t*         optional_channel_list,
                                const wiced_scan_extended_params_t*  optional_extended_params,
                                wiced_scan_result_callback_t         callback,
                                wiced_scan_result_t**                result_ptr,
                                void*                                user_data
                              )
{
    wiced_buffer_t     buffer;
    wl_escan_params_t* scan_params;
    wiced_result_t     retval;
    uint16_t           param_size        = offsetof( wl_escan_params_t, params ) + WL_SCAN_PARAMS_FIXED_SIZE;
    uint16_t           channel_list_size = 0;

    wiced_assert("Bad args", callback != NULL);

    /* Determine size of channel_list, and add it to the parameter size so correct sized buffer can be allocated */
    if ( optional_channel_list != NULL )
    {
        for ( channel_list_size = 0; optional_channel_list[channel_list_size] != 0; channel_list_size++ )
        {
        }
        param_size = (uint16_t) ( param_size + channel_list_size * sizeof(uint16_t) );
    }

    if ( WICED_SUCCESS != ( retval = wiced_management_set_event_handler( scan_events, scan_result_handler, user_data ) ) )
    {
        return retval;
    }

    /* Allocate a buffer for the IOCTL message */
    scan_params = (wl_escan_params_t*) wiced_get_iovar_buffer( &buffer, param_size, IOVAR_STR_ESCAN );
    CHECK_IOCTL_BUFFER( scan_params );

    /* Clear the scan parameters structure */
    memset( scan_params, 0, sizeof(wl_escan_params_t) );

    /* Fill in the appropriate details of the scan parameters structure */
    scan_params->version = htod32(ESCAN_REQ_VERSION);
    scan_params->action = htod16(WL_SCAN_ACTION_START);
    scan_params->params.scan_type = (int8_t) scan_type;
    scan_params->params.bss_type = (int8_t) bss_type;

    /* Fill out the SSID parameter if provided */
    if ( optional_ssid != NULL )
    {
        scan_params->params.ssid.SSID_len = optional_ssid->len;
        memcpy( scan_params->params.ssid.SSID, optional_ssid->val, scan_params->params.ssid.SSID_len );
    }

    /* Fill out the BSSID parameter if provided */
    if ( optional_mac != NULL )
    {
        memcpy( scan_params->params.bssid.octet, optional_mac, sizeof(wiced_mac_t) );
    }
    else
    {
        memset( scan_params->params.bssid.octet, 0xff, sizeof(wiced_mac_t) );
    }

    /* Fill out the extended parameters if provided */
    if ( optional_extended_params != NULL )
    {
        scan_params->params.nprobes = optional_extended_params->number_of_probes_per_channel;
        scan_params->params.active_time = optional_extended_params->scan_active_dwell_time_per_channel_ms;
        scan_params->params.passive_time = optional_extended_params->scan_passive_dwell_time_per_channel_ms;
        scan_params->params.home_time = optional_extended_params->scan_home_channel_dwell_time_between_channels_ms;
    }
    else
    {
        scan_params->params.nprobes = (int32_t) -1;
        scan_params->params.active_time = (int32_t) -1;
        scan_params->params.passive_time = (int32_t) -1;
        scan_params->params.home_time = (int32_t) -1;
    }

    /* Copy the channel list parameter if provided */
    if ( ( channel_list_size > 0 ) && ( optional_channel_list != NULL ) )
    {
        memcpy( scan_params->params.channel_list, optional_channel_list, channel_list_size * sizeof(uint16_t) );
        scan_params->params.channel_num = (int32_t) channel_list_size;
    }

    scan_result_callback = callback;
    scan_result_ptr = result_ptr;

    /* Send the Incremental Scan IOVAR message - blocks until the response is received */
    retval = wiced_send_iovar( SDPCM_SET, buffer, 0, SDPCM_STA_INTERFACE );

    /* Return the success of the IOCTL */
    return retval;
}

wiced_result_t wiced_wifi_stop_scan(void)
{
    wiced_buffer_t     buffer;
    wl_escan_params_t* scan_params;
    wiced_result_t     retval;
    uint16_t           param_size        = offsetof( wl_escan_params_t, params ) + WL_SCAN_PARAMS_FIXED_SIZE;

    /* Allocate a buffer for the IOCTL message */
    scan_params = (wl_escan_params_t*) wiced_get_iovar_buffer( &buffer, param_size, IOVAR_STR_ESCAN );
    CHECK_IOCTL_BUFFER( scan_params );

    /* Clear the scan parameters structure */
    memset( scan_params, 0, sizeof(wl_escan_params_t) );

    /* Fill in the appropriate details of the scan parameters structure */
    scan_params->version = htod32(ESCAN_REQ_VERSION);
    scan_params->action = htod16(WL_SCAN_ACTION_ABORT);
    scan_params->params.nprobes = (int32_t) -1;
    scan_params->params.active_time = (int32_t) -1;
    scan_params->params.passive_time = (int32_t) -1;
    scan_params->params.home_time = (int32_t) -1;
    
    /* Send the Incremental Scan IOVAR message - blocks until the response is received */
    retval = wiced_send_iovar( SDPCM_SET, buffer, 0, SDPCM_STA_INTERFACE );

    /* Return the success of the IOCTL */
    return retval;
}


/** Handles scan result events
 *
 *  This function receives scan record events, and parses them into a better format, then passes the results
 *  to the user application.
 *
 * @param event_header     : The event details
 * @param event_data       : The data for the event which contains the scan result structure
 * @param handler_user_data: data which will be passed to user application
 *
 * @returns : handler_user_data parameter
 *
 */
static void* scan_result_handler( const wiced_event_header_t* event_header, const uint8_t* event_data, /*@returned@*/ void* handler_user_data )
{
    wiced_scan_result_t* record;
    wl_escan_result_t*   eresult;
    wl_bss_info_t*       bss_info;
    uint16_t             chanspec;
    uint8_t*             cp;
    uint32_t             len;
    uint8_t*             rsnie;
    uint8_t*             wpaie;
    uint8_t              rate_num;
    uint8_t*             dsie;
    uint8_t*             parse;
    uint32_t             parse_len;
    ht_capabilities_ie_t* ht_capabilities_ie;

    if ( event_header->status == WLC_E_STATUS_SUCCESS )
    {
        scan_result_callback( NULL, handler_user_data );
        return handler_user_data;
    }

    if ( event_header->status != WLC_E_STATUS_PARTIAL )
    {
        return handler_user_data;
    }

    if (scan_result_ptr == NULL)
    {
        scan_result_callback( (wiced_scan_result_t**)event_data, handler_user_data );
        return handler_user_data;
    }

    record = (wiced_scan_result_t*) ( *scan_result_ptr );
    eresult = (wl_escan_result_t*) event_data;
    bss_info = &eresult->bss_info[0];

    wiced_assert( "More than one result returned by firmware", eresult->bss_count == 1 );

    /* Copy the SSID into the output record structure */
    record->SSID.len = bss_info->SSID_len;
    memset( record->SSID.val, 0, (size_t) SSID_MAX_LEN );
    memcpy( record->SSID.val, bss_info->SSID, record->SSID.len );

    /* Copy the BSSID into the output record structure */
    memcpy( (void*) record->BSSID.octet, (const void*) bss_info->BSSID.octet, sizeof(wiced_mac_t) );

    /* Copy the RSSID into the output record structure */
    record->signal_strength = bss_info->RSSI;
    record->on_channel = WICED_TRUE;

    /* find maximum data rate and put it in the output record structure */
    record->max_data_rate = 0;
    for ( rate_num = 0; rate_num < dtoh16(bss_info->rateset.count); rate_num++ )
    {
        uint32_t rate = ( bss_info->rateset.rates[rate_num] & 0x7f ) * (unsigned int) 500;
        if ( record->max_data_rate < rate )
        {
            record->max_data_rate = rate;
        }
    }

    /* Write the BSS type into the output record structure */
    record->bss_type = ( ( bss_info->capability & DOT11_CAP_ESS  ) != 0 ) ? WICED_BSS_TYPE_INFRASTRUCTURE :
                       ( ( bss_info->capability & DOT11_CAP_IBSS ) != 0 ) ? WICED_BSS_TYPE_ADHOC :
                                                                            WICED_BSS_TYPE_UNKNOWN;

    /* Determine the network security.
     * Some of this section has been copied from the standard broadcom host driver file wl/exe/wlu.c function wl_dump_wpa_rsn_ies
     */
    cp = (uint8_t*) ( ( (uint8_t*) bss_info ) + dtoh16(bss_info->ie_offset) );
    len = dtoh32(bss_info->ie_length);

    /* Find an RSN IE (Robust-Security-Network Information-Element) */
    rsnie = wlu_parse_tlvs( cp, len, (uint32_t) DOT11_MNG_RSN_ID );

    /* Find a WPA IE */
    if (rsnie == NULL)
    {
        parse = cp;
        parse_len = len;
        while ( ( wpaie = wlu_parse_tlvs( parse, parse_len, (uint32_t) DOT11_MNG_WPA_ID ) ) != 0 )
        {
            if ( wlu_is_wpa_ie( &wpaie, &parse, &parse_len ) != WICED_FALSE )
            {
                break;
            }
        }
    }

    /* Check if AP is configured for WPA2 */
    if ( rsnie != NULL )
    {
        uint16_t a;
        record->security = (wiced_security_t)WPA2_SECURITY;

        /* Check the RSN contents to see if there are any references to TKIP cipher (2) in the group key or pairwise keys. If so it must be mixed mode. */
        if ( ( NTOH32(((rsn_ie_t*)rsnie)->group_key_suite) & 0xFF ) == (uint32_t) 2 )
        {
            record->security |= TKIP_ENABLED;
        }
        if ( ( NTOH32(((rsn_ie_t*)rsnie)->group_key_suite) & 0xFF ) == (uint32_t) 4 )
        {
            record->security |= AES_ENABLED;
        }

        for ( a = 0; a < ( (rsn_ie_t*) rsnie )->pairwise_suite_count; ++a )
        {
            if ( ( NTOH32(((rsn_ie_t*)rsnie)->pairwise_suite_list[a]) & 0xFF ) == (uint32_t) 2 )
            {
                record->security |= TKIP_ENABLED;
            }

            if ( ( NTOH32(((rsn_ie_t*)rsnie)->pairwise_suite_list[a]) & 0xFF ) == (uint32_t) 4 )
            {
                record->security |= AES_ENABLED;
            }
        }

        if (record->security == WPA2_SECURITY)
        {
            record->security = WICED_SECURITY_UNKNOWN;
        }
    }
    /* Check if AP is configured for WPA */
    else if ( wpaie != NULL)
    {
        uint16_t a;
        /* Check if it's AES or TKIP */
        record->security = WICED_SECURITY_WPA_TKIP_PSK;
        for ( a = 0; a < ( (wpa_ie_t*) wpaie )->unicast_count; ++a )
        {
            if ( ((wpa_ie_t*)wpaie)->unicast_suit[a][3] == (uint32_t) 4 )
            {
                record->security = WICED_SECURITY_WPA_AES_PSK;
            }
        }
    }
    /* Check if AP is configured for WEP */
    else if ( ( bss_info->capability & DOT11_CAP_PRIVACY ) != 0 ) /* else if the capabilities field indicates privacy, then security supports WEP */
    {
        record->security = WICED_SECURITY_WEP_PSK;
    }
    else
    {
        /* Otherwise no security */
        record->security = WICED_SECURITY_OPEN;
    }

    /* Update the maximum data rate with 11n rates from the HT Capabilities IE */
    ht_capabilities_ie = (ht_capabilities_ie_t*)wlu_parse_tlvs( cp, len, (uint32_t) DOT11_MNG_HT_CAP );
    if ( ht_capabilities_ie != NULL )
    {
        uint8_t a;
        uint8_t supports_40mhz = (ht_capabilities_ie->ht_capabilities_info & (1 << 1)) != 0 ? 1 : 0;
        uint8_t short_gi[2]    = { (ht_capabilities_ie->ht_capabilities_info & (1 << 5)) != 0 ? 1 : 0,
                                   (ht_capabilities_ie->ht_capabilities_info & (1 << 6)) != 0 ? 1 : 0 };

        /* Find highest bit from MCS info */
        for (a = 31; a != 0xFF; --a)
        {
            if ( (ht_capabilities_ie->mcs[a / 8] & ( 1 << ( a % 8 ) )) != 0 )
            {
                break;
            }
        }
        if (a != 0xFF)
        {
            record->max_data_rate = 100UL * mcs_data_rate_lookup_table[a][supports_40mhz][short_gi[supports_40mhz]];
        }
    }

    /* Get the channel */
    chanspec = dtoh16( bss_info->chanspec );
    record->channel = ( (uint8_t) ( chanspec & WL_CHANSPEC_CHAN_MASK ) );

    /* Old WLAN firmware reported off channel probe responses - parse the response to check the channel */
    dsie =  wlu_parse_tlvs( cp, len, (uint32_t) DOT11_MNG_DS_ID );
    if ( ( dsie != NULL ) && ( record->channel != dsie[2] ) )
    {
        /* Received an off channel report - the RSSI will be
         * wrong, so just set it to a magic value
         */
        record->channel = dsie[2];
        record->on_channel = WICED_FALSE;
    }

    /* Copy the band into the output record structure */
    chanspec = dtoh16( bss_info->chanspec );

    if ( ( dtoh32(bss_info->version) != (uint32_t) LEGACY_WL_BSS_INFO_VERSION ) &&
         ( bss_info->n_cap != 0 ) )
    {
        record->band = ( ( chanspec & WL_CHANSPEC_BAND_MASK ) == (uint16_t) WL_CHANSPEC_BAND_2G ) ? WICED_802_11_BAND_2_4GHZ : WICED_802_11_BAND_5GHZ;
    }
    else
    {
        record->band = WICED_802_11_BAND_2_4GHZ;
    }

    scan_result_callback( scan_result_ptr, handler_user_data );
    if ( *scan_result_ptr == NULL )
    {
#if 0
        wiced_management_set_event_handler( scan_events, NULL, NULL );
#endif /* if 0 */
    }

    return handler_user_data;
}


wiced_result_t wiced_wifi_join( const char* ssid, wiced_security_t auth_type, const uint8_t* security_key, uint8_t key_length, host_semaphore_type_t* semaphore )
{
    wiced_buffer_t buffer;
    host_semaphore_type_t join_sema;
    wiced_result_t result;
    wlc_ssid_t* ssid_params;

    if ( strlen( ssid ) <= (size_t) 0 )
    {
        WPRINT_WWD_DEBUG(("wiced_wifi_join: SSID too short\r\n"));
        return WICED_BADSSIDLEN;
    }
    if ( strlen( ssid ) > (size_t) SSID_MAX_LEN )
    {
        WPRINT_WWD_DEBUG(("wiced_wifi_join: SSID too long\r\n"));
        return WICED_BADSSIDLEN;
    }
    if ( semaphore == NULL )
    {
        if ( WICED_SUCCESS != ( result = host_rtos_init_semaphore( &join_sema ) ) )
        {
            return result;
        }
        result = wiced_wifi_prepare_join( auth_type, security_key, key_length, &join_sema );
    }
    else
    {
        result = wiced_wifi_prepare_join( auth_type, security_key, key_length, semaphore );
    }
    if ( result == WICED_SUCCESS )
    {
        /* Join network */
        ssid_params = (struct wlc_ssid *) wiced_get_ioctl_buffer( &buffer, sizeof(wlc_ssid_t) );
        CHECK_IOCTL_BUFFER( ssid_params );
        memset( ssid_params, 0, sizeof(wlc_ssid_t) );
        ssid_params->SSID_len = strlen( ssid );
        memcpy( ssid_params->SSID, ssid, ssid_params->SSID_len );
        result = wiced_send_ioctl( SDPCM_SET, WLC_SET_SSID, buffer, 0, SDPCM_STA_INTERFACE );

        if ( result == WICED_SUCCESS && semaphore == NULL )
        {
            result = host_rtos_get_semaphore( &join_sema, DEFAULT_JOIN_TIMEOUT, WICED_FALSE );
            wiced_assert( "Get semaphore failed", ( result == WICED_SUCCESS ) || ( result == WICED_TIMEOUT ) );
            host_rtos_deinit_semaphore( &join_sema );
            result = wiced_wifi_is_ready_to_transceive( WICED_STA_INTERFACE );
            if (result  != WICED_SUCCESS )
            {
                wiced_wifi_leave( );
            }
            return result;
        }
    }
    if ( semaphore == NULL )
    {
        host_rtos_deinit_semaphore( &join_sema );
    }
    return result;
}


wiced_result_t wiced_wifi_join_specific( const wiced_scan_result_t* ap, const uint8_t* security_key, uint8_t key_length, host_semaphore_type_t* semaphore )
{
    wiced_buffer_t        buffer;
    host_semaphore_type_t join_semaphore;
    wiced_result_t        result;
    wl_join_params_t*     join_params;
    wl_join_params_5g_t* join_params_5g;
    
    if ( semaphore == NULL )
    {
        result = host_rtos_init_semaphore( &join_semaphore );
        if ( result != WICED_SUCCESS )
        {
            return result;
        }
        result = wiced_wifi_prepare_join( ap->security, security_key, key_length, &join_semaphore );
    }
    else
    {
        result = wiced_wifi_prepare_join( ap->security, security_key, key_length, semaphore );
    }
    if ( result == WICED_SUCCESS )
    {
        /* Check if soft AP is running, if so, move its current channel to the the destination AP */
        if ( wiced_wifi_is_ready_to_transceive( WICED_AP_INTERFACE ) == WICED_SUCCESS )
        {
            uint32_t current_softap_channel = 0;
            wwd_wifi_get_channel( WICED_AP_INTERFACE, &current_softap_channel );
            if ( current_softap_channel != ap->channel )
            {
                wwd_wifi_set_channel( WICED_AP_INTERFACE, ap->channel );
                host_rtos_delay_milliseconds( 100 );
            }
        }

        if (use_5g_join_param == 1) {
            /* Join network */
            join_params_5g = (wl_join_params_5g_t*) wiced_get_ioctl_buffer( &buffer, sizeof(wl_join_params_5g_t) );
            CHECK_IOCTL_BUFFER( join_params_5g );
            memset( join_params_5g, 0, sizeof(wl_join_params_t) );

            join_params_5g->ssid.SSID_len = ap->SSID.len;
            memcpy( join_params_5g->ssid.SSID, ap->SSID.val, join_params_5g->ssid.SSID_len );
            memcpy( &join_params_5g->params.bssid, &ap->BSSID, sizeof(wiced_mac_t) );
            join_params_5g->params.chanspec_num = (uint32_t) 1;
            join_params_5g->params.chanspec_list[0] = (wl_chanspec_t) htod16((ap->channel | WL_CHANSPEC_BAND_2G | WL_CHANSPEC_BW_20 | WL_CHANSPEC_CTL_SB_NONE));
        } else {
            /* Join network */
            join_params = (wl_join_params_t*) wiced_get_ioctl_buffer( &buffer, sizeof(wl_join_params_t) );
            CHECK_IOCTL_BUFFER( join_params );
            memset( join_params, 0, sizeof(wl_join_params_t) );
            
            join_params->ssid.SSID_len = ap->SSID.len;
            memcpy( join_params->ssid.SSID, ap->SSID.val, join_params->ssid.SSID_len );
            memcpy( &join_params->params.bssid, &ap->BSSID, sizeof(wiced_mac_t) );
#ifdef CHIP_HAS_BSSID_CNT_IN_ASSOC_PARAMS
            join_params->params.bssid_cnt = 0;
#endif /* ifdef CHIP_HAS_BSSID_CNT_IN_ASSOC_PARAMS */
            join_params->params.chanspec_num = (uint32_t) 1;
            join_params->params.chanspec_list[0] = (wl_chanspec_t) htod16((ap->channel | WL_CHANSPEC_BAND_2G | WL_CHANSPEC_BW_20 | WL_CHANSPEC_CTL_SB_NONE));
        }
        result = wiced_send_ioctl( SDPCM_SET, WLC_SET_SSID, buffer, 0, SDPCM_STA_INTERFACE );

        if ( result == WICED_SUCCESS && semaphore == NULL )
        {
            host_rtos_get_semaphore( &join_semaphore, (uint32_t) DEFAULT_JOIN_TIMEOUT, WICED_FALSE );
            host_rtos_deinit_semaphore( &join_semaphore );
            result = wiced_wifi_is_ready_to_transceive( WICED_STA_INTERFACE );
            if (result  != WICED_SUCCESS )
            {
                wiced_wifi_leave( );
            }
            return result;
        }
    }
    if ( semaphore == NULL )
    {
        host_rtos_deinit_semaphore( &join_semaphore );
    }
    return result;
}

static wiced_result_t wiced_wifi_prepare_join( wiced_security_t auth_type, /*@unique@*/ const uint8_t* security_key, uint8_t key_length, host_semaphore_type_t* semaphore )
{
    wiced_buffer_t buffer;
    wiced_result_t retval;
    uint16_t a;
    uint32_t* wsec_setting;
    uint32_t* data;
    uint32_t* infra;
    uint32_t* wpa_auth;
    uint32_t* auth;
    static int last_auth = 0;

    if ( ( ( ( key_length > (uint8_t) WSEC_MAX_PSK_LEN ) ||
             ( key_length < (uint8_t) WSEC_MIN_PSK_LEN ) ) &&
           ( ( auth_type == WICED_SECURITY_WPA_TKIP_PSK ) ||
             ( auth_type == WICED_SECURITY_WPA_AES_PSK ) ||
             ( auth_type == WICED_SECURITY_WPA2_AES_PSK ) ||
             ( auth_type == WICED_SECURITY_WPA2_TKIP_PSK ) ||
             ( auth_type == WICED_SECURITY_WPA2_MIXED_PSK ) ) ) )
    {
#if 0
        WPRINT_WWD_DEBUG(("wiced_wifi_prepare_join: Security key invalid\r\n"));
#endif /* if 0 */
        return WICED_INVALID_KEY;
    }

    /* Clear the current join status */
    wiced_join_status = 0;

    /* Set Wireless Security Type */
    wsec_setting = (uint32_t*) wiced_get_ioctl_buffer( &buffer, (uint16_t) 4 );
    CHECK_IOCTL_BUFFER( wsec_setting );
    *wsec_setting = (uint32_t) ((auth_type&0xFF) & ~WPS_ENABLED);
    retval = wiced_send_ioctl( SDPCM_SET, WLC_SET_WSEC, buffer, 0, SDPCM_STA_INTERFACE );
    wiced_assert("wiced_wifi_prepare_join: set security type failed\r\n", retval == WICED_SUCCESS );

    /* Set supplicant variable */
    data = wiced_get_iovar_buffer( &buffer, (uint16_t) 4, IOVAR_STR_SUP_WPA );
    CHECK_IOCTL_BUFFER( data );
    *data = (uint32_t) ( ( ( auth_type == WICED_SECURITY_WPA_TKIP_PSK ) ||
                           ( auth_type == WICED_SECURITY_WPA_AES_PSK ) ||
                           ( auth_type == WICED_SECURITY_WPA2_AES_PSK ) ||
                           ( auth_type == WICED_SECURITY_WPA2_TKIP_PSK ) ||
                           ( auth_type == WICED_SECURITY_WPA2_MIXED_PSK ) ) ? 1 : 0 );
    retval = wiced_send_iovar( SDPCM_SET, buffer, 0, SDPCM_STA_INTERFACE );
    wiced_assert("wiced_wifi_prepare_join: set supplicant failed\r\n", retval == WICED_SUCCESS );

    /* Set the EAPOL version to whatever the AP is using (-1) */
    data = wiced_get_iovar_buffer( &buffer, (uint16_t) 4, IOVAR_STR_SUP_WPA2_EAPVER );
    CHECK_IOCTL_BUFFER( data );
    *data = (uint32_t)-1;
    retval = wiced_send_iovar( SDPCM_SET, buffer, 0, SDPCM_STA_INTERFACE );
    wiced_assert("wiced_wifi_prepare_join: set WPA2 EAP version failed\r\n", retval == WICED_SUCCESS );

    /* Send WPA Key */
    switch ( auth_type )
    {
        case WICED_SECURITY_OPEN:
        case WICED_SECURITY_WPS_OPEN:
        case WICED_SECURITY_WPS_SECURE:
            break;
        case WICED_SECURITY_WPA_TKIP_PSK:
        case WICED_SECURITY_WPA_AES_PSK:
        case WICED_SECURITY_WPA2_AES_PSK:
        case WICED_SECURITY_WPA2_TKIP_PSK:
        case WICED_SECURITY_WPA2_MIXED_PSK:
        {
            wsec_pmk_t* psk = wiced_get_ioctl_buffer( &buffer, sizeof(wsec_pmk_t) );
            CHECK_IOCTL_BUFFER( psk );
            memset( psk, 0, sizeof(wsec_pmk_t) );
            memcpy( psk->key, security_key, key_length );
            psk->key_len = key_length;
            psk->flags = (uint16_t) WSEC_PASSPHRASE;
            host_rtos_delay_milliseconds(1); /* Delay required to allow radio firmware to be ready to receive PMK and avoid intermittent failure */
            retval = wiced_send_ioctl( SDPCM_SET, WLC_SET_WSEC_PMK, buffer, 0, SDPCM_STA_INTERFACE );
            wiced_assert("wiced_wifi_prepare_join: set WPA key failed\r\n", retval == WICED_SUCCESS );
        }
            break;

        case WICED_SECURITY_WEP_PSK:
        case WICED_SECURITY_WEP_SHARED:
            for ( a = 0; a < key_length; a = (uint16_t) ( a + 2 + security_key[1] ) )
            {
                const wiced_wep_key_t* in_key = (const wiced_wep_key_t*) &security_key[a];
                wl_wsec_key_t* out_key = (wl_wsec_key_t*) wiced_get_ioctl_buffer( &buffer, sizeof(wl_wsec_key_t) );
                CHECK_IOCTL_BUFFER( out_key );
                memset( out_key, 0, sizeof(wl_wsec_key_t) );
                out_key->index = in_key->index;
                out_key->len = in_key->length;
                memcpy( out_key->data, in_key->data, in_key->length );
                switch ( in_key->length )
                {
                    case 5:
                        out_key->algo = (uint32_t) CRYPTO_ALGO_WEP1;
                        break;
                    case 13:
                        out_key->algo = (uint32_t)CRYPTO_ALGO_WEP128;
                        break;
                    case 16:
                        /* default to AES-CCM */
                        out_key->algo = (uint32_t) CRYPTO_ALGO_AES_CCM;
                        break;
                    case 32:
                        out_key->algo = (uint32_t) CRYPTO_ALGO_TKIP;
                        break;
                    default:
                        host_buffer_release(buffer, WICED_NETWORK_TX);
                        return WICED_INVALID_KEY;
                }
                /* Set the first entry as primary key by default */
                if ( a == 0 )
                {
                    out_key->flags |= WL_PRIMARY_KEY;
                }
                out_key->index = htod32(out_key->index);
                out_key->len   = htod32(out_key->len);
                out_key->algo  = htod32(out_key->algo);
                out_key->flags = htod32(out_key->flags);
                retval = wiced_send_ioctl( SDPCM_SET, WLC_SET_KEY, buffer, NULL, SDPCM_STA_INTERFACE );
                wiced_assert("wiced_wifi_prepare_join: set WEP key failed\r\n", retval == WICED_SUCCESS );
            }
            break;
        case WICED_SECURITY_FORCE_32_BIT:
        case WICED_SECURITY_UNKNOWN:
        default:
            wiced_assert("wiced_wifi_prepare_join: Unknown security type\r\n", 0 != 0 );
            break;
    }

    /* Set infrastructure mode */
    infra = (uint32_t*) wiced_get_ioctl_buffer( &buffer, (uint16_t) 4 );
    CHECK_IOCTL_BUFFER( infra );
    *infra = (uint32_t) 1;
    retval = wiced_send_ioctl( SDPCM_SET, WLC_SET_INFRA, buffer, 0, SDPCM_STA_INTERFACE );
    wiced_assert("wiced_wifi_prepare_join: set infrastructure mode failed\r\n", retval == WICED_SUCCESS );

    /* Set authentication type */
    auth = (uint32_t*) wiced_get_ioctl_buffer( &buffer, (uint16_t) 4 );
    CHECK_IOCTL_BUFFER( auth );
    
    if (WICED_SECURITY_WEP_PSK == auth_type) {
        if (last_auth == 0) {
            *auth = 1;
            last_auth = 1;
        } else {
            *auth = 0;
            last_auth = 0;
    }
    } else
        *auth = 0; /*  0 = OpenSystem , 1= SharedKey */
    
    retval = wiced_send_ioctl( SDPCM_SET, WLC_SET_AUTH, buffer, 0, SDPCM_STA_INTERFACE );
    wiced_assert("wiced_wifi_prepare_join: set auth type failed\r\n", retval == WICED_SUCCESS );

    /* Set WPA authentication mode */
    wpa_auth = (uint32_t*) wiced_get_ioctl_buffer( &buffer, (uint16_t) 4 );
    CHECK_IOCTL_BUFFER( wpa_auth );

    switch ( auth_type )
    {
        case WICED_SECURITY_OPEN:
        case WICED_SECURITY_WPS_OPEN:
        case WICED_SECURITY_WPS_SECURE:
            *wpa_auth = WPA_AUTH_DISABLED;
            wiced_join_status |= JOIN_SECURITY_COMPLETE;
            break;
        case WICED_SECURITY_WPA_TKIP_PSK:
        case WICED_SECURITY_WPA_AES_PSK:
            *wpa_auth = (uint32_t) WPA_AUTH_PSK;
            break;
        case WICED_SECURITY_WPA2_AES_PSK:
        case WICED_SECURITY_WPA2_TKIP_PSK:
        case WICED_SECURITY_WPA2_MIXED_PSK:
            *wpa_auth = (uint32_t) WPA2_AUTH_PSK;
            break;
        case WICED_SECURITY_WEP_PSK:
        case WICED_SECURITY_WEP_SHARED:
            *wpa_auth = WPA_AUTH_DISABLED;
            wiced_join_status |= JOIN_SECURITY_COMPLETE;
            break;
        case WICED_SECURITY_UNKNOWN:
        case WICED_SECURITY_FORCE_32_BIT:
        default:
            WPRINT_WWD_DEBUG(("Bad Security type\r\n"));
            *wpa_auth = WPA_AUTH_DISABLED;
            break;
    }

    retval = wiced_send_ioctl( SDPCM_SET, WLC_SET_WPA_AUTH, buffer, 0, SDPCM_STA_INTERFACE );
    wiced_assert("wiced_wifi_prepare_join: set WPA auth mode failed\r\n", retval == WICED_SUCCESS );

    wifi_join_aping = WICED_TRUE;
    retval = wiced_management_set_event_handler( join_events, wiced_join_events_handler, (void*) semaphore );
    return retval;
}


wiced_result_t wiced_wifi_leave( void )
{
    wiced_buffer_t buffer;
    wiced_result_t result;

    if (wifi_join_aping != WICED_TRUE)
        return WICED_SUCCESS;
    
    if ( WICED_SUCCESS != ( result = wiced_management_set_event_handler( join_events, NULL, NULL ) ) )
    {
        return result;
    }

    /* Disassociate from AP */
    if ( NULL == wiced_get_ioctl_buffer( &buffer, 0 ) )
    {
        return WICED_ERROR;
    }
    result = wiced_send_ioctl( SDPCM_SET, WLC_DISASSOC, buffer, 0, SDPCM_STA_INTERFACE );
    if (result == WICED_SUCCESS)
    {
        wiced_join_status = 0;
    }
    wifi_join_aping = WICED_FALSE;
    return result;
}

wiced_result_t wiced_wifi_deauth_sta( const wiced_mac_t* mac, wiced_dot11_reason_code_t reason )
{
    wiced_buffer_t buffer;
    wiced_result_t result;
    scb_val_t* scb_val;

    scb_val = (scb_val_t *) wiced_get_ioctl_buffer( &buffer, sizeof(scb_val_t) );
    CHECK_IOCTL_BUFFER( scb_val );
    memset((char *)scb_val, 0, sizeof(scb_val_t));
    memcpy((char *)&scb_val->ea, (char *) mac, sizeof(wiced_mac_t));
    scb_val->val = (uint32_t)reason;
    result = wiced_send_ioctl( SDPCM_SET, WLC_SCB_DEAUTHENTICATE_FOR_REASON, buffer, 0, SDPCM_AP_INTERFACE );

    return result;
}

/** Callback for join events
 *  This is called when the WLC_E_SET_SSID event is received,
 *  indicating that the system has joined successfully.
 *  Wakes the thread which was doing the join, allowing it to resume.
 */
static /*@null@*/ void* wiced_join_events_handler( const wiced_event_header_t* event_header, const uint8_t* event_data, /*@returned@*/ void* handler_user_data )
{
    host_semaphore_type_t* semaphore = (host_semaphore_type_t*) handler_user_data;
    wiced_bool_t join_complete = WICED_FALSE;
    /*@-noeffect@*/
    UNUSED_PARAMETER(event_data);
    /*@+noeffect@*/

    if ( event_header->interface != (uint8_t) WICED_STA_INTERFACE )
    {
        return handler_user_data;
    }

    switch ( event_header->event_type )
    {
        case WLC_E_ROAM:
            if ( event_header->status == WLC_E_STATUS_SUCCESS )
            {
                wiced_join_status |= JOIN_LINK_READY;
            }
            break;

        case WLC_E_PSK_SUP:
            if ( event_header->status == WLC_SUP_KEYED )
            {
                /* Successful WPA key exchange */
                wiced_join_status |= JOIN_SECURITY_COMPLETE;
            }
            else if ( event_header->status != WLC_SUP_LAST_BASIC_STATE && event_header->status != WLC_SUP_KEYXCHANGE )
            {
                wiced_join_status = JOIN_WRONG_KEY;
                /* WPA PSK error - abort (usually means key was incorrect) */
                join_complete = WICED_TRUE;
            }
            break;

        case WLC_E_SET_SSID:
            if ( event_header->status == WLC_E_STATUS_SUCCESS )
            {
                /* SSID has been successfully set. */
                wiced_join_status |= JOIN_COMPLETE;
            }
            else if ( event_header->status == WLC_E_STATUS_NO_NETWORKS )
            {
                wiced_join_status = JOIN_NO_NETWORKS;
                join_complete = WICED_TRUE;
            }
            else
            {
                join_complete = WICED_TRUE;
            }
            break;

        case WLC_E_LINK:
            if ( ( event_header->flags & WLC_EVENT_MSG_LINK ) != 0 )
            {
                wiced_join_status |= JOIN_LINK_READY;
            }
            else
            {
                wiced_join_status &= ~JOIN_LINK_READY;
#if 0
                WPRINT_WWD_INFO(("%s link is down\r\n", (event_header->interface == WICED_STA_INTERFACE) ? "STA" : "AP"));
#endif /* if 0 */
            }
            break;

        case WLC_E_DEAUTH_IND:
            wiced_join_status &= ~JOIN_AUTHENTICATED;
            break;

        case WLC_E_AUTH:
            if ( event_header->status == WLC_E_STATUS_SUCCESS )
            {
                wiced_join_status |= JOIN_AUTHENTICATED;
            }
            else
            {
                /* We cannot authenticate. Perhaps we're blocked */
                join_complete = WICED_TRUE;
            }
            break;

        default:
            wiced_assert( "Received event which was not registered\r\n", 0 != 0 );
            break;
    }

    if ( wiced_wifi_is_ready_to_transceive( WICED_STA_INTERFACE ) == WICED_SUCCESS )
    {
        join_complete = WICED_TRUE;
    }

    if ( join_complete == WICED_TRUE )
    {
        if ( semaphore != NULL )
        {
            host_rtos_set_semaphore( semaphore, WICED_FALSE );
        }
        return NULL;
    }
    else
    {
        return handler_user_data;
    }
}


wiced_result_t wiced_wifi_get_mac_address( wiced_mac_t* mac )
{
    wiced_buffer_t buffer;
    wiced_buffer_t response;
    wiced_result_t result;

    memset( (uint8_t *)mac, 0x0, sizeof(wiced_mac_t) );

    if ( NULL == wiced_get_iovar_buffer( &buffer, sizeof(wiced_mac_t), IOVAR_STR_CUR_ETHERADDR ) )
    {
        return WICED_ERROR;
    }
    result = wiced_send_iovar( SDPCM_GET, buffer, &response, SDPCM_STA_INTERFACE );
    if ( result == WICED_SUCCESS )
    {
        memcpy( mac, host_buffer_get_current_piece_data_pointer( response ), sizeof(wiced_mac_t) );
        host_buffer_release( response, WICED_NETWORK_RX );
    }
    return result;
}


wiced_result_t wiced_wifi_set_mac_address( wiced_mac_t mac )
{
    wiced_buffer_t buffer;

    uint32_t* data = (uint32_t*) wiced_get_iovar_buffer( &buffer, sizeof(wiced_mac_t), IOVAR_STR_CUR_ETHERADDR );
    CHECK_IOCTL_BUFFER( data );
    memcpy( data, &mac, sizeof(wiced_mac_t) );
    return wiced_send_iovar( SDPCM_SET, buffer, NULL, SDPCM_STA_INTERFACE );
}

wiced_result_t wiced_wifi_set_nmode( int mode )
{
    wiced_buffer_t buffer;
    wiced_result_t result;
    int* data;

    data = wiced_get_iovar_buffer( &buffer, 4, IOVAR_STR_NMODE );
    if ( NULL ==  data)
    {
        return WICED_ERROR;
    }

    *data = mode;
    result = wiced_send_iovar( SDPCM_SET, buffer, 0, SDPCM_STA_INTERFACE );
    return result;
}

wiced_result_t wiced_wifi_is_ready_to_transceive( wiced_interface_t interface )
{
    /* Handle Splint bug */ /*@-noeffect@*/ (void) wiced_wifi_ap_is_up; /*@+noeffect@*/

    switch ( interface )
    {
        case WICED_CONFIG_INTERFACE:
        case WICED_AP_INTERFACE:
            return ( wiced_wifi_ap_is_up == WICED_TRUE ) ? WICED_SUCCESS : WICED_ERROR;

        case WICED_STA_INTERFACE:
            switch ( wiced_join_status )
            {
                case JOIN_NO_NETWORKS:
                    return WICED_NOTFOUND;

                case JOIN_AUTHENTICATED | JOIN_LINK_READY | JOIN_SECURITY_COMPLETE | JOIN_COMPLETE:
                    return WICED_SUCCESS;

                case 0:
                case JOIN_SECURITY_COMPLETE: /* For open/WEP */
                    return WICED_NOT_AUTHENTICATED;

                case JOIN_AUTHENTICATED | JOIN_LINK_READY | JOIN_SECURITY_COMPLETE:
                    return WICED_UNFINISHED;

                case JOIN_WRONG_KEY:
                case JOIN_AUTHENTICATED | JOIN_LINK_READY:
                case JOIN_AUTHENTICATED | JOIN_LINK_READY | JOIN_COMPLETE:
                    return WICED_NOT_KEYED;

                default:
                    return WICED_ERROR;
            }
            /* Disables Eclipse static analysis warning */
            /* No break needed due to returns in all case paths */
            /* no break */
        default:
            return WICED_ERROR;
    }
}


wiced_result_t wiced_wifi_enable_powersave_with_throughput( uint8_t return_to_sleep_delay_ms )
{
    wiced_buffer_t buffer;
    uint32_t* data;

    if( return_to_sleep_delay_ms > 100 )
    {
        return_to_sleep_delay_ms = 100;
    }

    /* Set the maximum time to wait before going back to sleep */
    data = (uint32_t*) wiced_get_iovar_buffer( &buffer, (uint16_t) 4, IOVAR_STR_PM2_SLEEP_RET );
    CHECK_IOCTL_BUFFER( data );
    *data = (uint32_t) ( return_to_sleep_delay_ms / 10 )* 10;
    (void) wiced_send_iovar( SDPCM_SET, buffer, NULL, SDPCM_STA_INTERFACE );

    data = (uint32_t*) wiced_get_iovar_buffer( &buffer, (uint16_t) 4, "pm_limit" );
    CHECK_IOCTL_BUFFER( data );
    *data = (uint32_t) NULL_FRAMES_WITH_PM_SET_LIMIT;
    (void) wiced_send_iovar( SDPCM_SET, buffer, NULL, SDPCM_STA_INTERFACE );

    /* set PM2 fast return to sleep powersave mode */
    data = (uint32_t*) wiced_get_ioctl_buffer( &buffer, (uint16_t) 4 );
    CHECK_IOCTL_BUFFER( data );
    *data = (uint32_t) PM2_POWERSAVE_MODE;
    return wiced_send_ioctl( SDPCM_SET, WLC_SET_PM, buffer, NULL, SDPCM_STA_INTERFACE );
}


wiced_result_t wiced_wifi_enable_powersave( void )
{
    wiced_buffer_t buffer;
    uint32_t* data;

    /* Set legacy powersave mode - PM1 */
    data = (uint32_t*) wiced_get_ioctl_buffer( &buffer, (uint16_t) 4 );
    CHECK_IOCTL_BUFFER( data );
    *data = (uint32_t) PM1_POWERSAVE_MODE;
    return wiced_send_ioctl( SDPCM_SET, WLC_SET_PM, buffer, NULL, SDPCM_STA_INTERFACE );
}


wiced_result_t wiced_wifi_disable_powersave( void )
{
    wiced_buffer_t buffer;

    uint32_t* data = (uint32_t*) wiced_get_ioctl_buffer( &buffer, (uint16_t) 4 );
    CHECK_IOCTL_BUFFER( data );
    *data = NO_POWERSAVE_MODE;
    return wiced_send_ioctl( SDPCM_SET, WLC_SET_PM, buffer, NULL, SDPCM_STA_INTERFACE );
}


wiced_result_t wiced_wifi_get_tx_power( uint8_t* dbm )
{
    wiced_buffer_t buffer;
    wiced_buffer_t response;
    wiced_result_t     result;

    uint32_t* data = (uint32_t*) wiced_get_iovar_buffer( &buffer, (uint16_t) 4, IOVAR_STR_QTXPOWER );
    CHECK_IOCTL_BUFFER( data );
    result = wiced_send_iovar( SDPCM_GET, buffer, &response, SDPCM_STA_INTERFACE );

    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    data = (uint32_t*) host_buffer_get_current_piece_data_pointer( response );
    *dbm = (uint8_t) ( *data / 4 );
    host_buffer_release( response, WICED_NETWORK_RX );
    return WICED_SUCCESS;
}


wiced_result_t wiced_wifi_set_tx_power( uint8_t dbm )
{
    wiced_buffer_t buffer;

    uint32_t* data = (uint32_t*) wiced_get_iovar_buffer( &buffer, (uint16_t) 4, IOVAR_STR_QTXPOWER );
    CHECK_IOCTL_BUFFER( data );
    if ( dbm == (uint8_t) 0xFF )
    {
        *data = (uint32_t) 127;
    }
    else
    {
        *data = (uint32_t) ( 4 * dbm );
    }
    return wiced_send_iovar( SDPCM_SET, buffer, NULL, SDPCM_STA_INTERFACE );
}


wiced_result_t wiced_wifi_set_listen_interval( uint8_t listen_interval, wiced_listen_interval_time_unit_t time_unit )
{
	uint32_t* data;
    wiced_buffer_t buffer;
    uint8_t listen_interval_dtim;
    wiced_result_t result;

    if (time_unit == WICED_LISTEN_INTERVAL_TIME_UNIT_DTIM)
    {
    	listen_interval_dtim = listen_interval;
    }
    else
    {
        /* If the wake interval measured in DTIMs is set to 0, the wake interval is measured in beacon periods */
    	listen_interval_dtim = 0;

    	/* The wake period is measured in beacon periods, set the value as required */
		data = (uint32_t*) wiced_get_iovar_buffer( &buffer, (uint16_t) 4, IOVAR_STR_LISTEN_INTERVAL_BEACON );
		CHECK_IOCTL_BUFFER( data );
		*data = (uint32_t) listen_interval;
		result = wiced_send_iovar( SDPCM_SET, buffer, NULL, SDPCM_STA_INTERFACE );
		wiced_assert("Failed to set listen interval beacon\r\n", result == WICED_SUCCESS);
    }

	data = (uint32_t*) wiced_get_iovar_buffer( &buffer, (uint16_t) 4, IOVAR_STR_LISTEN_INTERVAL_DTIM );
	CHECK_IOCTL_BUFFER( data );
	*data = (uint32_t) listen_interval_dtim;
    result = wiced_send_iovar( SDPCM_SET, buffer, NULL, SDPCM_STA_INTERFACE );
    wiced_assert("Failed to set set listen interval dtim\r\n", result == WICED_SUCCESS);

    result = wiced_wifi_set_listen_interval_assoc( (uint16_t)listen_interval );

    return result;
}


wiced_result_t wiced_wifi_set_listen_interval_assoc( uint16_t listen_interval )
{
    wiced_buffer_t buffer;

    uint32_t* data = (uint32_t*) wiced_get_iovar_buffer( &buffer, (uint16_t) 4, IOVAR_STR_LISTEN_INTERVAL_ASSOC );
    CHECK_IOCTL_BUFFER( data );
    *data = (uint32_t) listen_interval;
    return wiced_send_iovar( SDPCM_SET, buffer, NULL, SDPCM_STA_INTERFACE );
}


wiced_result_t wiced_wifi_get_listen_interval( wiced_listen_interval_t* li )
{
    wiced_buffer_t buffer;
    wiced_buffer_t response;
    int*           data;
    wiced_result_t ret;

    data = (int*)wiced_get_iovar_buffer( &buffer, 4, IOVAR_STR_LISTEN_INTERVAL_BEACON );
    CHECK_IOCTL_BUFFER( data );
    memset( data, 0, 1 );
    ret = wiced_send_iovar( SDPCM_GET, buffer, &response, SDPCM_STA_INTERFACE );
    if (ret == WICED_SUCCESS)
    {
        memcpy( (uint8_t *) &(li->beacon), (char *)host_buffer_get_current_piece_data_pointer( response ), 1 );
        host_buffer_release(response, WICED_NETWORK_RX);
    }
    else
    {
        goto exit;
    }

    data = (int*)wiced_get_iovar_buffer( &buffer, 4, IOVAR_STR_LISTEN_INTERVAL_DTIM );
    CHECK_IOCTL_BUFFER( data );
    memset( data, 0, 1 );
    ret = wiced_send_iovar( SDPCM_GET, buffer, &response, SDPCM_STA_INTERFACE );
    if (ret == WICED_SUCCESS)
    {
        memcpy( (uint8_t *) &(li->dtim), (char *)host_buffer_get_current_piece_data_pointer( response ), 1 );
        host_buffer_release(response, WICED_NETWORK_RX);
    }
    else
    {
        goto exit;
    }

    data = (int*)wiced_get_iovar_buffer( &buffer, 4, IOVAR_STR_LISTEN_INTERVAL_ASSOC );
    CHECK_IOCTL_BUFFER( data );
    memset( data, 0, 4 );
    ret = wiced_send_iovar( SDPCM_GET, buffer, &response, SDPCM_STA_INTERFACE );
    if (ret == WICED_SUCCESS)
    {
        memcpy( (uint16_t *) &(li->assoc), (char *)host_buffer_get_current_piece_data_pointer( response ), 2 );
        host_buffer_release(response, WICED_NETWORK_RX);
    }

    exit:
        return ret;
}


wiced_result_t wiced_wifi_set_ofdm_dutycycle( uint8_t duty_cycle_val )
{
    wiced_buffer_t buffer;
    uint32_t* data;
    if( duty_cycle_val > 100 )
    {
        return WICED_BADARG;
    }
    data = (uint32_t*) wiced_get_iovar_buffer( &buffer, (uint16_t) 4, IOVAR_STR_DUTY_CYCLE_OFDM );
    CHECK_IOCTL_BUFFER( data );
    *data = (uint32_t)duty_cycle_val;

    return wiced_send_iovar( SDPCM_SET, buffer, NULL, SDPCM_STA_INTERFACE );
}


wiced_result_t wiced_wifi_set_cck_dutycycle( uint8_t duty_cycle_val )
{
    wiced_buffer_t buffer;
    uint32_t* data;
    if( duty_cycle_val > 100 )
    {
        return WICED_BADARG;
    }
    data = (uint32_t*) wiced_get_iovar_buffer( &buffer, (uint16_t) 4, IOVAR_STR_DUTY_CYCLE_CCK );
    CHECK_IOCTL_BUFFER( data );
    *data = (uint32_t)duty_cycle_val;

    return wiced_send_iovar( SDPCM_SET, buffer, NULL, SDPCM_STA_INTERFACE );
}


wiced_result_t wiced_wifi_get_ofdm_dutycycle( uint8_t* duty_cycle_value )
{
    wiced_buffer_t buffer;
    wiced_buffer_t response;
    wiced_result_t     result;

    uint32_t* data = (uint32_t*) wiced_get_iovar_buffer( &buffer, (uint16_t) 4, IOVAR_STR_DUTY_CYCLE_OFDM );
    CHECK_IOCTL_BUFFER( data );
    result = wiced_send_iovar( SDPCM_GET, buffer, &response, SDPCM_STA_INTERFACE );

    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    data = (uint32_t*) host_buffer_get_current_piece_data_pointer( response );
    *duty_cycle_value = (uint8_t)*data;
    host_buffer_release( response, WICED_NETWORK_RX );

    return WICED_SUCCESS;
}


wiced_result_t wiced_wifi_get_cck_dutycycle( uint8_t* duty_cycle_value )
{
    wiced_buffer_t buffer;
    wiced_buffer_t response;
    wiced_result_t     result;

    uint32_t* data = (uint32_t*) wiced_get_iovar_buffer( &buffer, (uint16_t) 4, IOVAR_STR_DUTY_CYCLE_CCK );
    CHECK_IOCTL_BUFFER( data );
    result = wiced_send_iovar( SDPCM_GET, buffer, &response, SDPCM_STA_INTERFACE );

    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    data = (uint32_t*) host_buffer_get_current_piece_data_pointer( response );
    *duty_cycle_value = (uint8_t)*data;
    host_buffer_release( response, WICED_NETWORK_RX );

    return WICED_SUCCESS;
}


wiced_result_t wiced_wifi_get_pmk( char* psk, uint8_t psk_length, char* pmk )
{
    wsec_pmk_t*    psk_ioctl;
    wiced_buffer_t buffer;
    wiced_buffer_t response;

    psk_ioctl = ( wsec_pmk_t* )wiced_get_ioctl_buffer( &buffer, sizeof(wsec_pmk_t) );
    memcpy( psk_ioctl->key, psk, psk_length );
    psk_ioctl->key[psk_length] = 0;
    psk_ioctl->key_len      = psk_length;
    psk_ioctl->flags        = 0;

    if ( wiced_send_ioctl( SDPCM_GET, WLC_GET_WSEC_PMK, buffer, &response, SDPCM_STA_INTERFACE ) == WICED_SUCCESS )
    {
        wsec_pmk_t* psk_info = (wsec_pmk_t*) host_buffer_get_current_piece_data_pointer( response );
        if ( psk_info->key_len == WSEC_MAX_PSK_LEN )
        {
            memcpy( pmk, psk_info->key, psk_info->key_len );
            host_buffer_release( response, WICED_NETWORK_RX );
            return WICED_SUCCESS;
        }
        else
        {
            host_buffer_release( response, WICED_NETWORK_RX );
            return WICED_ERROR;
        }
    }

    return WICED_ERROR;
}


wiced_result_t wiced_wifi_register_multicast_address( wiced_mac_t* mac )
{
    wiced_buffer_t buffer;
    wiced_buffer_t response;
    uint16_t a;
    wiced_result_t result;
    mcast_list_t* orig_mcast_list;
    mcast_list_t* new_mcast_list;

    /* Get the current multicast list */
    CHECK_IOCTL_BUFFER( wiced_get_iovar_buffer( &buffer, sizeof(uint32_t) + MAX_SUPPORTED_MCAST_ENTRIES * sizeof(wiced_mac_t), IOVAR_STR_MCAST_LIST ) );
    result = wiced_send_iovar( SDPCM_GET, buffer, &response, SDPCM_STA_INTERFACE );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    /* Verify address is not currently registered */
    orig_mcast_list = (mcast_list_t*) host_buffer_get_current_piece_data_pointer( response );
    for ( a = 0; a < orig_mcast_list->entry_count; ++a )
    {
        /* Check if any address matches */
        if ( 0 == memcmp( mac, &orig_mcast_list->macs[a], sizeof(wiced_mac_t) ) )
        {
            /* A matching address has been found so we can stop now. */
            host_buffer_release( response, WICED_NETWORK_RX );
            return WICED_SUCCESS;
        }
    }

    /* Add the provided address to the list and write the new multicast list */
    new_mcast_list = (mcast_list_t*) wiced_get_iovar_buffer( &buffer, (uint16_t) ( sizeof(uint32_t) + ( orig_mcast_list->entry_count + 1 ) * sizeof(wiced_mac_t) ), IOVAR_STR_MCAST_LIST );
    CHECK_IOCTL_BUFFER( new_mcast_list );
    new_mcast_list->entry_count = orig_mcast_list->entry_count;
    memcpy( new_mcast_list->macs, orig_mcast_list->macs, orig_mcast_list->entry_count * sizeof(wiced_mac_t) );
    host_buffer_release( response, WICED_NETWORK_RX );
    memcpy( &new_mcast_list->macs[new_mcast_list->entry_count], mac, sizeof(wiced_mac_t) );
    ++new_mcast_list->entry_count;
    return wiced_send_iovar( SDPCM_SET, buffer, NULL, SDPCM_STA_INTERFACE );
}


wiced_result_t wiced_wifi_unregister_multicast_address( wiced_mac_t* mac )
{
    wiced_buffer_t buffer;
    wiced_buffer_t response;
    uint16_t a;
    wiced_result_t result;
    mcast_list_t* orig_mcast_list;

    /* Get the current multicast list */
    CHECK_IOCTL_BUFFER( wiced_get_iovar_buffer( &buffer, sizeof(uint32_t) + MAX_SUPPORTED_MCAST_ENTRIES * sizeof(wiced_mac_t), IOVAR_STR_MCAST_LIST ) );
    result = wiced_send_iovar( SDPCM_GET, buffer, &response, SDPCM_STA_INTERFACE );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    /* Find the address, assuming it is part of the list */
    orig_mcast_list = (mcast_list_t*) host_buffer_get_current_piece_data_pointer( response );
    if ( orig_mcast_list->entry_count != 0 )
    {
        mcast_list_t* new_mcast_list = (mcast_list_t*) wiced_get_iovar_buffer( &buffer, (uint16_t) ( sizeof(uint32_t) + ( orig_mcast_list->entry_count - 1 ) * sizeof(wiced_mac_t) ), IOVAR_STR_MCAST_LIST );
        CHECK_IOCTL_BUFFER( new_mcast_list );
        for ( a = 0; a < orig_mcast_list->entry_count; ++a )
        {
            if ( 0 == memcmp( mac, &orig_mcast_list->macs[a], sizeof(wiced_mac_t) ) )
            {
                /* Copy the existing list up to the matching address */
                memcpy( new_mcast_list->macs, orig_mcast_list->macs, a * sizeof(wiced_mac_t) );

                /* Skip the current address and copy the remaining entries */
                memcpy( &new_mcast_list->macs[a], &orig_mcast_list->macs[a + 1], ( size_t )( orig_mcast_list->entry_count - a - 1 ) * sizeof(wiced_mac_t) );

                new_mcast_list->entry_count = orig_mcast_list->entry_count - 1;
                host_buffer_release( response, WICED_NETWORK_RX );
                return wiced_send_iovar( SDPCM_SET, buffer, NULL, SDPCM_STA_INTERFACE );
            }
        }
        /* There was something in the list, but the request MAC wasn't there */
        host_buffer_release( buffer, WICED_NETWORK_TX );
    }
    /* If we get here than the address wasn't in the list or the list was empty */
    host_buffer_release( response, WICED_NETWORK_RX );
    return WICED_SUCCESS;
}


wiced_result_t wiced_wifi_get_rssi( int32_t* rssi )
{
    wiced_buffer_t buffer;
    wiced_buffer_t response;
    wiced_result_t result;

    CHECK_IOCTL_BUFFER( wiced_get_ioctl_buffer( &buffer, (uint16_t) 4 ) );
    result = wiced_send_ioctl( SDPCM_GET, WLC_GET_RSSI, buffer, &response, SDPCM_STA_INTERFACE );
    if ( result == WICED_SUCCESS )
    {
        memcpy( rssi, host_buffer_get_current_piece_data_pointer( response ), sizeof(int32_t) );
        host_buffer_release( response, WICED_NETWORK_RX );
    }
    return result;
}

wiced_result_t wiced_wifi_get_ap_client_rssi( int32_t* rssi, wiced_mac_t* client_mac_addr  )
{
    wiced_result_t result;
    client_rssi_t* client_rssi;
    wiced_buffer_t buffer;
    wiced_buffer_t response;

    /* WLAN expects buffer size to be 4-byte aligned */
    client_rssi = (client_rssi_t*) wiced_get_ioctl_buffer( &buffer, ROUND_UP( sizeof(client_rssi_t), sizeof(uint32_t)) );
	CHECK_IOCTL_BUFFER( client_rssi );

	memcpy(&client_rssi->mac_address, client_mac_addr, sizeof(wiced_mac_t));
    client_rssi->value = 0;

    result = wiced_send_ioctl( SDPCM_GET, WLC_GET_RSSI, buffer, &response, SDPCM_AP_INTERFACE );
    if ( result == WICED_SUCCESS )
    {
        memcpy( rssi, host_buffer_get_current_piece_data_pointer( response ), sizeof(int32_t) );
        host_buffer_release( response, WICED_NETWORK_RX );
        return WICED_SUCCESS;
    }

    return result;
}


wiced_result_t wiced_wifi_select_antenna( wiced_antenna_t antenna )
{
    wiced_buffer_t buffer;
    wiced_result_t retval;
    uint32_t* data;

    data = wiced_get_ioctl_buffer( &buffer, (uint16_t) 4 );
    CHECK_IOCTL_BUFFER( data );
    *data = (uint32_t) antenna;
    retval = wiced_send_ioctl( SDPCM_SET, WLC_SET_TXANT, buffer, 0, SDPCM_STA_INTERFACE );
    wiced_assert("Failed to set TX antenna\r\n", retval == WICED_SUCCESS);

    data = wiced_get_ioctl_buffer( &buffer, (uint16_t) 4 );
    CHECK_IOCTL_BUFFER( data );
    *data = (uint32_t) antenna;
    retval = wiced_send_ioctl( SDPCM_SET, WLC_SET_ANTDIV, buffer, 0, SDPCM_STA_INTERFACE );
    wiced_assert("Failed to set RX antenna\r\n", retval == WICED_SUCCESS);

    return retval;
}


wiced_result_t wiced_wifi_set_roam_trigger( int32_t trigger_level )
{
    wiced_result_t retval;
    wiced_buffer_t buffer;
    uint32_t* data = wiced_get_ioctl_buffer( &buffer, (uint16_t) 4 );
    CHECK_IOCTL_BUFFER( data );
    *data = (uint32_t)trigger_level;
    retval = wiced_send_ioctl( SDPCM_SET, WLC_SET_ROAM_TRIGGER, buffer, 0, SDPCM_STA_INTERFACE );
    wiced_assert("Failed to set roam trigger\r\n", retval == WICED_SUCCESS);
    return retval;
}
wiced_result_t wlan_wifi_set_roam_delta( int32_t delta );
wiced_result_t wlan_wifi_get_roam_delta( int32_t *delta );
wiced_result_t wlan_wifi_get_bcnprd( int32_t *prd );

wiced_result_t wlan_wifi_set_roam_delta( int32_t delta )
{
    wiced_result_t retval;
    wiced_buffer_t buffer;
    uint32_t* data = wiced_get_ioctl_buffer( &buffer, (uint16_t) 4 );
    CHECK_IOCTL_BUFFER( data );
    *data = (uint32_t)delta;
    retval = wiced_send_ioctl( SDPCM_SET, WLC_SET_ROAM_DELTA, buffer, 0, SDPCM_STA_INTERFACE );
    wiced_assert("Failed to set roam delta\r\n", retval == WICED_SUCCESS);
    return retval;
}

wiced_result_t wlan_wifi_get_roam_delta( int32_t *delta )
{
	wiced_buffer_t buffer;
	wiced_buffer_t response;
	wiced_result_t result;

	CHECK_IOCTL_BUFFER( wiced_get_ioctl_buffer( &buffer, (uint16_t) 4 ) );
	result = wiced_send_ioctl( SDPCM_GET, WLC_GET_ROAM_DELTA, buffer, &response, SDPCM_STA_INTERFACE );
	if ( result == WICED_SUCCESS )
	{
		memcpy( delta, host_buffer_get_current_piece_data_pointer( response ), sizeof(int32_t) );
		host_buffer_release( response, WICED_NETWORK_RX );
	}
	return result;
}



wiced_result_t wlan_wifi_get_bcnprd( int32_t *prd )
{
	wiced_buffer_t buffer;
	wiced_buffer_t response;
	wiced_result_t result;

	CHECK_IOCTL_BUFFER( wiced_get_ioctl_buffer( &buffer, (uint16_t) 4 ) );
	result = wiced_send_ioctl( SDPCM_GET, WLC_GET_BCNPRD, buffer, &response, SDPCM_AP_INTERFACE );
	if ( result == WICED_SUCCESS )
	{
		memcpy( prd, host_buffer_get_current_piece_data_pointer( response ), sizeof(int32_t) );
		host_buffer_release( response, WICED_NETWORK_RX );
	}
	return result;
}


wiced_result_t wiced_wifi_send_action_frame(wl_action_frame_t* action_frame)
{
    wiced_buffer_t buffer;
    wl_action_frame_t* frame;
    uint32_t* a = (uint32_t*)wiced_get_iovar_buffer(&buffer, sizeof(wl_action_frame_t)+4, "bsscfg:"IOVAR_STR_ACTION_FRAME);
    CHECK_IOCTL_BUFFER( a );
    *a = 1;
    frame = (wl_action_frame_t*)(a+1);
    memcpy(frame, action_frame, sizeof(wl_action_frame_t));
    return wiced_send_iovar(SDPCM_SET, buffer, NULL, SDPCM_STA_INTERFACE);
}


wiced_result_t wiced_wifi_get_acparams_sta( edcf_acparam_t *acp )
{
    wiced_buffer_t buffer;
    wiced_buffer_t response;
    wiced_result_t ret;

    int* data = (int*)wiced_get_iovar_buffer( &buffer, 64, IOVAR_STR_AC_PARAMS_STA );
    CHECK_IOCTL_BUFFER( data );
    memset( data, 0, 64 );
    ret = wiced_send_iovar( SDPCM_GET, buffer, &response, SDPCM_STA_INTERFACE );
    if (ret == WICED_SUCCESS)
    {
        memcpy( (char *)acp, (char *)host_buffer_get_current_piece_data_pointer( response ), ( sizeof( edcf_acparam_t ) * 4 ) );
        host_buffer_release(response, WICED_NETWORK_RX);
    }

    return ret;
}

wiced_result_t wiced_wifi_enable_monitor_mode( void )
{
    wiced_buffer_t buffer;
    uint32_t*      data;

    /* Enable allmulti mode */
    data = wiced_get_iovar_buffer(&buffer, 4, "allmulti");
    *data = 1;
    if (wiced_send_iovar(SDPCM_SET, buffer, NULL, SDPCM_STA_INTERFACE) != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    /* Enable monitor mode */
    data = wiced_get_ioctl_buffer(&buffer, 4);
    *data = 1;
    if (wiced_send_ioctl(SDPCM_SET, WLC_SET_MONITOR, buffer, NULL, SDPCM_STA_INTERFACE) != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    monitor_mode_enabled = WICED_TRUE;
    return WICED_SUCCESS;
}

wiced_result_t wiced_wifi_disable_monitor_mode( void )
{
    wiced_buffer_t buffer;
    uint32_t*      data;

    /* Disable allmulti mode */
    data = wiced_get_iovar_buffer(&buffer, 4, "allmulti");
    *data = 0;
    if (wiced_send_iovar(SDPCM_SET, buffer, NULL, SDPCM_STA_INTERFACE) != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    /* Disable monitor mode */
    data = wiced_get_ioctl_buffer(&buffer, 4);
    *data = 0;
    if (wiced_send_ioctl(SDPCM_SET, WLC_SET_MONITOR, buffer, NULL, SDPCM_STA_INTERFACE) != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    monitor_mode_enabled = WICED_FALSE;
    return WICED_SUCCESS;
}

/******************************************************
 *             Wiced-internal functions
 ******************************************************/

/** Set the Wi-Fi device down. Internal use only
 *
 * @param interface
 * @return
 */
wiced_result_t wiced_wifi_set_down(wiced_interface_t interface)
{
    wiced_buffer_t buffer;
    wiced_result_t retval;
    /*@-noeffect@*/
    UNUSED_PARAMETER( interface );
    /*@+noeffect@*/

    /* Send DOWN command */
    (void) wiced_get_ioctl_buffer( &buffer, 0 );
    retval = wiced_send_ioctl( SDPCM_SET, WLC_DOWN, buffer, 0, SDPCM_STA_INTERFACE );
    return retval;
}

wiced_result_t wiced_wifi_manage_custom_ie( wiced_interface_t interface, wiced_custom_ie_action_t action, /*@unique@*/ uint8_t* oui, uint8_t subtype, void* data, uint16_t length, uint16_t which_packets)
{
    wiced_buffer_t buffer;
    vndr_ie_setbuf_t* ie_setbuf;
    uint32_t* a = (uint32_t*)wiced_get_iovar_buffer(&buffer, (uint16_t)(sizeof(vndr_ie_setbuf_t) + length + 4), "bsscfg:" IOVAR_STR_VENDOR_IE);
    CHECK_IOCTL_BUFFER( a );
    *a = interface;
    ie_setbuf = (vndr_ie_setbuf_t*)(a+1);

    /* Copy the vndr_ie SET command ("add"/"del") to the buffer */
    if (action == WICED_ADD_CUSTOM_IE)
    {
        memcpy( (char*)ie_setbuf->cmd, "add", 3 );
    }
    else
    {
        memcpy( (char*)ie_setbuf->cmd, "del", 3 );
    }
    ie_setbuf->cmd[3] = 0;

    /* Set the values */
    ie_setbuf->vndr_ie_buffer.iecount = (int32_t) 1;

    ie_setbuf->vndr_ie_buffer.vndr_ie_list[0].pktflag = which_packets;
    ie_setbuf->vndr_ie_buffer.vndr_ie_list[0].vndr_ie_data.id  = 0xdd;
    ie_setbuf->vndr_ie_buffer.vndr_ie_list[0].vndr_ie_data.len = (uint8_t)(length + sizeof(ie_setbuf->vndr_ie_buffer.vndr_ie_list[0].vndr_ie_data.oui) + 1); /* +1: one byte for sub type */

    /*@-usedef@*/ /* Stop lint warning about vndr_ie_list array element not yet being defined */
    memcpy( ie_setbuf->vndr_ie_buffer.vndr_ie_list[0].vndr_ie_data.oui, oui, (size_t) 3 );
    /*@+usedef@*/

    ie_setbuf->vndr_ie_buffer.vndr_ie_list[0].vndr_ie_data.data[0] = subtype;

    memcpy(&ie_setbuf->vndr_ie_buffer.vndr_ie_list[0].vndr_ie_data.data[1], data, length);

    return wiced_send_iovar(SDPCM_SET, buffer, NULL, (sdpcm_interface_t) WICED_STA_INTERFACE );
}


void wiced_wifi_prioritize_acparams( const edcf_acparam_t *acp, int *priority )
{
    int aci;
    int aifsn;
    int ecwmin;
    int ecwmax;
    int ranking_basis[AC_COUNT];
    int *p;

    p = priority;

    for (aci = 0; aci < AC_COUNT; aci++, acp++, p++)
    {
        aifsn  = acp->ACI & EDCF_AIFSN_MASK;
        ecwmin = acp->ECW & EDCF_ECWMIN_MASK;
        ecwmax = (acp->ECW & EDCF_ECWMAX_MASK) >> EDCF_ECWMAX_SHIFT;
        ranking_basis[aci] = aifsn + ecwmin + ecwmax; // Default AC_VO will be the littlest ranking value
        *p = 1; // Initialise priority starting at 1
    }

    p = priority;

    // Primitive ranking method which works for AC priority swapping when values for cwmin, cwmax and aifsn are varied
    for (aci = 0; aci < AC_COUNT; aci++, p++) // Compare each ACI against each other ACI
    {
        int i = 0;
        for (i = 0; i < AC_COUNT; i++)
        {
            if ( i != aci )
            {
                // Smaller ranking value has higher priority, so increment priority for each ACI which has a higher ranking value
                if (ranking_basis[aci] < ranking_basis[i])
                {
                    *p = *p + 1;
                }
            }
        }
    }
}


int wiced_wifi_get_available_tos( wiced_qos_access_category_t ac, const edcf_acparam_t *acp )
{
    int tos = -1;
    static const wiced_qos_access_category_t ac_values[] = { WMM_AC_VO, WMM_AC_VI, WMM_AC_BE, WMM_AC_BK }; // Highest to lowest priority
    static const int start_idx[] = { 2, 3, 1, 0 }; // Starting index in the order the AC parameters come in: WMM_AC_BE, WMM_AC_BK, WMM_AC_VI, WMM_AC_VO
    static const wiced_ip_header_tos_t tos_values[] = { TOS_VO7, TOS_VI, TOS_BE, TOS_BK }; // Highest to lowest priority
    int i;

    if ( acp != NULL )
    {
        // For the given AC look up the highest AC available which does not require admission control and map it to a TOS value
        for ( i = start_idx[ac]; i < AC_COUNT; i++ )
        {
            if ( ( acp[ac_values[i]].ACI & EDCF_ACM_MASK ) == 0 )
            {
                tos = tos_values[i];
                break;
            }
        }
    }

    return tos;
}


void print_ac_params( const edcf_acparam_t *acp, int *priority )
{
    int acm;
    int aci, aifsn, ecwmin, ecwmax, txop;
    static const char ac_names[AC_COUNT][6] = {"AC_BE", "AC_BK", "AC_VI", "AC_VO"};

    if ( acp != NULL )
    {
        for (aci = 0; aci < AC_COUNT; aci++, acp++)
        {
            if (((acp->ACI & EDCF_ACI_MASK) >> EDCF_ACI_SHIFT) != aci)
            {
                printf("Warning: AC params out of order\n");
            }
            acm = (acp->ACI & EDCF_ACM_MASK) ? 1 : 0;
            aifsn = acp->ACI & EDCF_AIFSN_MASK;
            ecwmin = acp->ECW & EDCF_ECWMIN_MASK;
            ecwmax = (acp->ECW & EDCF_ECWMAX_MASK) >> EDCF_ECWMAX_SHIFT;
            txop = (uint16_t)acp->TXOP;
            printf("%s: raw: ACI 0x%x ECW 0x%x TXOP 0x%x\r\n", ac_names[aci], acp->ACI, acp->ECW, txop);
            printf("       dec: aci %d acm %d aifsn %d " "ecwmin %d ecwmax %d txop 0x%x\r\n", aci, acm, aifsn, ecwmin, ecwmax, txop);
                /* CWmin = 2^(ECWmin) - 1 */
                /* CWmax = 2^(ECWmax) - 1 */
                /* TXOP = number of 32 us units */
            printf("       eff: CWmin %d CWmax %d TXop %dusec\r\n", EDCF_ECW2CW(ecwmin), EDCF_ECW2CW(ecwmax), EDCF_TXOP2USEC(txop));
        }
    }

    if ( priority != NULL )
    {
        for (aci = 0; aci < AC_COUNT; aci++, priority++)
        {
            printf("%s: ACI %d Priority %d\r\n", ac_names[aci], aci, *priority);
        }
    }
}


wiced_result_t wiced_wifi_get_channel( uint32_t* channel)
{
    return wwd_wifi_get_channel(WICED_STA_INTERFACE, channel);
}

wiced_result_t wwd_wifi_get_channel( wiced_interface_t interface, uint32_t* channel )
{
    wiced_buffer_t buffer;
    wiced_buffer_t response;
    wiced_result_t result;

    wiced_get_ioctl_buffer( &buffer, sizeof(channel_info_t) );
    result = wiced_send_ioctl( SDPCM_GET, WLC_GET_CHANNEL, buffer, &response, interface );
    if ( result == WICED_SUCCESS )
    {
        channel_info_t* channel_info = (channel_info_t*) host_buffer_get_current_piece_data_pointer( response );
        *channel = (uint32_t) channel_info->hw_channel;
        host_buffer_release( response, WICED_NETWORK_RX );
    }
    return result;
}

wiced_result_t wiced_wifi_set_channel( uint32_t channel)
{
    return wwd_wifi_set_channel(WICED_STA_INTERFACE, channel);
}

wiced_result_t wwd_wifi_set_channel( wiced_interface_t interface, uint32_t channel )
{
    wiced_buffer_t    buffer;
    wiced_result_t    result;
    uint32_t*         data;
    wl_chan_switch_t* chan_switch;

    switch ( interface )
    {
        case WICED_STA_INTERFACE:
            data = wiced_get_ioctl_buffer( &buffer, sizeof(uint32_t) );
            *data = channel;
            result = wiced_send_ioctl( SDPCM_GET, WLC_SET_CHANNEL, buffer, NULL, SDPCM_STA_INTERFACE );
            break;

        case WICED_AP_INTERFACE:
        case WICED_CONFIG_INTERFACE:
            chan_switch = (wl_chan_switch_t*)wiced_get_iovar_buffer( &buffer, sizeof(wl_chan_switch_t), "csa" );
            chan_switch->chspec = (wl_chanspec_t)(WL_CHANSPEC_BAND_2G | WL_CHANSPEC_BW_20 | WL_CHANSPEC_CTL_SB_NONE | channel);
            chan_switch->count  = 1;
            chan_switch->mode   = 1;
            chan_switch->reg    = 0;
            result = wiced_send_iovar( SDPCM_SET, buffer, 0, SDPCM_AP_INTERFACE );
            break;

        default:
            result = WICED_BADARG;
    }

    return result;
}

wiced_result_t wiced_wifi_get_counters( wiced_interface_t interface, wl_cnt_v6_t* counters )
{
    wiced_buffer_t buffer;
    wiced_buffer_t response;

    wiced_get_iovar_buffer( &buffer, sizeof(wl_cnt_v6_t), IOVAR_STR_COUNTERS );
    if ( wiced_send_iovar( SDPCM_GET, buffer, &response, (sdpcm_interface_t)interface ) == WICED_SUCCESS )
    {
        wl_cnt_v6_t* received_counters = (wl_cnt_v6_t*) host_buffer_get_current_piece_data_pointer( response );
        memcpy(counters, received_counters, sizeof(wl_cnt_v6_t));
        host_buffer_release( response, WICED_NETWORK_RX );
        return WICED_SUCCESS;
    }
    else
    {
        return WICED_ERROR;
    }

}

wiced_result_t wiced_wifi_set_packet_filter_mode( wiced_packet_filter_mode_t mode )
{
    wiced_buffer_t buffer;

    uint32_t* data = (uint32_t*)wiced_get_iovar_buffer( &buffer, sizeof(uint32_t), IOVAR_STR_PKT_FILTER_MODE );
    CHECK_IOCTL_BUFFER( data );
    *data = (uint32_t)mode;
    return wiced_send_iovar( SDPCM_SET, buffer, NULL, SDPCM_STA_INTERFACE );
}

wiced_result_t wiced_wifi_add_packet_filter( uint8_t filter_id, const wiced_packet_filter_settings_t* settings )
{
    wl_pkt_filter_t* packet_filter;
    wiced_buffer_t   buffer;
    uint32_t         buffer_length = ( 2 * (uint32_t)settings->pattern_list[0].mask_size ) + WL_PKT_FILTER_FIXED_LEN + WL_PKT_FILTER_PATTERN_FIXED_LEN;

    /* Pattern list currently isn't supported */
    if ( settings->pattern_count > 1 )
    {
        return WICED_UNSUPPORTED;
    }

    packet_filter = (wl_pkt_filter_t*) wiced_get_iovar_buffer( &buffer, (uint16_t)buffer_length , IOVAR_STR_PKT_FILTER_ADD );
    CHECK_IOCTL_BUFFER( packet_filter );

    /* Copy filter entries */
    packet_filter->id                   = filter_id;
    packet_filter->type                 = 0;
    packet_filter->negate_match         = settings->rule;
    packet_filter->u.pattern.offset     = (uint32_t)settings->pattern_list[0].offset;
    packet_filter->u.pattern.size_bytes = settings->pattern_list[0].mask_size;

    /* Copy mask */
    memcpy( packet_filter->u.pattern.mask_and_pattern, settings->pattern_list[0].mask, settings->pattern_list[0].mask_size );

    /* Copy filter pattern */
    memcpy( packet_filter->u.pattern.mask_and_pattern + settings->pattern_list[0].mask_size, settings->pattern_list[0].pattern, settings->pattern_list[0].mask_size );

    return wiced_send_iovar( SDPCM_SET, buffer, NULL, SDPCM_STA_INTERFACE );
}

wiced_result_t wiced_wifi_remove_packet_filter( uint8_t filter_id )
{
    wiced_buffer_t buffer;

    uint32_t* data = (uint32_t*)wiced_get_iovar_buffer( &buffer, sizeof(uint32_t), IOVAR_STR_PKT_FILTER_DELETE );
    CHECK_IOCTL_BUFFER( data );
    *data = (uint32_t)filter_id;
    return wiced_send_iovar( SDPCM_SET, buffer, NULL, SDPCM_STA_INTERFACE );
}

wiced_result_t wiced_wifi_enable_packet_filter( uint8_t filter_id )
{
    return wiced_wifi_toggle_packet_filter( filter_id, WICED_TRUE );
}

wiced_result_t wiced_wifi_disable_packet_filter( uint8_t filter_id )
{
    return wiced_wifi_toggle_packet_filter( filter_id, WICED_FALSE );
}

wiced_result_t wiced_wifi_toggle_packet_filter( uint8_t filter_id, wiced_bool_t enable )
{
    wiced_buffer_t buffer;

    wl_pkt_filter_enable_t* data = wiced_get_iovar_buffer( &buffer, sizeof(wl_pkt_filter_enable_t), IOVAR_STR_PKT_FILTER_ENABLE );
    CHECK_IOCTL_BUFFER( data );
    data->id     = (uint32_t)filter_id;
    data->enable = (uint32_t)enable;
    return wiced_send_iovar( SDPCM_SET, buffer, NULL, SDPCM_STA_INTERFACE );
}

wiced_result_t wiced_wifi_get_packet_filter_stats( uint8_t filter_id, wiced_packet_filter_stats_t* stats )
{
    wiced_buffer_t buffer;
    wiced_buffer_t response;
    wiced_result_t ret;

    uint32_t* data = (uint32_t*)wiced_get_iovar_buffer( &buffer, sizeof(uint32_t) + sizeof(wiced_packet_filter_stats_t), IOVAR_STR_PKT_FILTER_STATS );
    CHECK_IOCTL_BUFFER( data );

    memset( data, 0, sizeof(uint32_t) + sizeof(wiced_packet_filter_stats_t) );
    *data = (uint32_t)filter_id;

    ret = wiced_send_iovar( SDPCM_GET, buffer, &response, SDPCM_STA_INTERFACE );

    if ( ret == WICED_SUCCESS )
    {
        memcpy( (char *)stats, (char *)host_buffer_get_current_piece_data_pointer( response ), ( sizeof(wiced_packet_filter_stats_t) ) );
        host_buffer_release( response, WICED_NETWORK_RX );
    }

    return ret;
}

wiced_result_t wiced_wifi_clear_packet_filter_stats( uint32_t filter_id )
{
    wiced_buffer_t buffer;

    uint32_t* data = wiced_get_iovar_buffer( &buffer, sizeof(uint32_t), IOVAR_STR_PKT_FILTER_CLEAR_STATS );
    CHECK_IOCTL_BUFFER( data );
    *data = (uint32_t)filter_id;
    return wiced_send_iovar( SDPCM_SET, buffer, NULL, SDPCM_STA_INTERFACE );
}

wiced_result_t wiced_wifi_get_enabled_packet_filter_list( uint32_t* count, wiced_packet_filter_t** list )
{
    return wiced_wifi_get_packet_filter_list( count, list, WICED_TRUE );
}

wiced_result_t wiced_wifi_get_disabled_packet_filter_list( uint32_t* count, wiced_packet_filter_t** list )
{
    return wiced_wifi_get_packet_filter_list( count, list, WICED_FALSE );
}

wiced_result_t wiced_wifi_get_packet_filter_list( uint32_t* count, wiced_packet_filter_t** list, wiced_bool_t enabled_list )
{
    wiced_buffer_t buffer;
    wiced_buffer_t response;
    wiced_result_t result;
    uint32_t*      data;

    data = wiced_get_iovar_buffer( &buffer, PACKET_FILTER_LIST_BUFFER_MAX_LEN, IOVAR_STR_PKT_FILTER_LIST );
    CHECK_IOCTL_BUFFER( data );
    *data = (uint32_t)enabled_list;

    result = wiced_send_iovar( SDPCM_GET, buffer, &response, SDPCM_STA_INTERFACE );

    if ( result == WICED_SUCCESS )
    {
        wl_pkt_filter_list_t* filter_list  = (wl_pkt_filter_list_t*)host_buffer_get_current_piece_data_pointer( response );
        uint32_t              filter_ptr   = (uint32_t)filter_list->filter;
        uint32_t              i;

        *list  = (wiced_packet_filter_t*)malloc_named( "pkt_filter", sizeof( wiced_packet_filter_t ) * filter_list->num );
        *count = filter_list->num;

        for ( i = 0; i < filter_list->num; i++ )
        {
            wl_pkt_filter_t*       in_filter  = (wl_pkt_filter_t*)filter_ptr;
            wiced_packet_filter_t* out_filter = (wiced_packet_filter_t*)&(*list)[i];

            out_filter->id                               = in_filter->id;
            out_filter->settings.pattern_count           = 1;
            out_filter->settings.rule                    = ( in_filter->negate_match == 0 ) ? WICED_PACKET_FILTER_RULE_POSITIVE_MATCHING : WICED_PACKET_FILTER_RULE_NEGATIVE_MATCHING;
            out_filter->settings.pattern_list            = (wiced_packet_filter_pattern_t*)malloc_named( "pkt_filter", sizeof( wiced_packet_filter_pattern_t ) + in_filter->u.pattern.size_bytes * 2 );
            out_filter->settings.pattern_list->offset    = (uint16_t)in_filter->u.pattern.offset;
            out_filter->settings.pattern_list->mask_size = (uint16_t)in_filter->u.pattern.size_bytes;
            out_filter->next                             = &(*list)[i + 1];

            /* Copy mask */
            out_filter->settings.pattern_list->mask = (uint8_t*)out_filter->settings.pattern_list + sizeof( wiced_packet_filter_pattern_t );
            memcpy( out_filter->settings.pattern_list->mask, in_filter->u.pattern.mask_and_pattern, in_filter->u.pattern.size_bytes );

            /* Copy pattern  */
            out_filter->settings.pattern_list->pattern = (uint8_t*)out_filter->settings.pattern_list + sizeof(wiced_packet_filter_pattern_t) + in_filter->u.pattern.size_bytes;
            memcpy( out_filter->settings.pattern_list->pattern, in_filter->u.pattern.mask_and_pattern + in_filter->u.pattern.size_bytes, in_filter->u.pattern.size_bytes );

            /* Update WL filter pointer */
            filter_ptr += (WL_PKT_FILTER_FIXED_LEN + WL_PKT_FILTER_PATTERN_FIXED_LEN + 2 * in_filter->u.pattern.size_bytes);

            /* WLAN returns word-aligned filter list */
            filter_ptr = ROUND_UP( filter_ptr, 4 );
        }

        (*list)[i-1].next = NULL;

        host_buffer_release( response, WICED_NETWORK_RX );
    }

    return result;
}

wiced_result_t wiced_wifi_delete_packet_filter_list( wiced_packet_filter_t* list )
{
    wiced_packet_filter_t* filter = list;

    while ( filter != NULL )
    {
        wiced_packet_filter_t* next = filter->next;
        malloc_transfer_to_curr_thread( filter->settings.pattern_list );
        free( filter->settings.pattern_list );
        filter = next;
    }

    malloc_transfer_to_curr_thread( list );
    free( list );
    return WICED_SUCCESS;
}

wiced_result_t wiced_wifi_add_keep_alive( wiced_keep_alive_packet_t* keep_alive_packet_info )
{
    wiced_buffer_t buffer;

    uint16_t length = (uint16_t)(keep_alive_packet_info->packet_length + WL_MKEEP_ALIVE_FIXED_LEN);
    wl_mkeep_alive_pkt_t* packet_info = (wl_mkeep_alive_pkt_t*) wiced_get_iovar_buffer( &buffer, length, IOVAR_STR_MKEEP_ALIVE );
    CHECK_IOCTL_BUFFER( packet_info );

    packet_info->version       = htod16(WL_MKEEP_ALIVE_VERSION);
    packet_info->length        = htod16(WL_MKEEP_ALIVE_FIXED_LEN);
    packet_info->keep_alive_id = keep_alive_packet_info->keep_alive_id;
    packet_info->period_msec   = htod32(keep_alive_packet_info->period_msec);
    packet_info->len_bytes     = htod16(keep_alive_packet_info->packet_length);
    memcpy(packet_info->data, keep_alive_packet_info->packet, keep_alive_packet_info->packet_length);

    return wiced_send_iovar( SDPCM_SET, buffer, NULL, SDPCM_STA_INTERFACE );
}

wiced_result_t wiced_wifi_get_keep_alive( wiced_keep_alive_packet_t* keep_alive_packet_info )
{
	wl_mkeep_alive_pkt_t* packet_info;
    wiced_buffer_t        buffer;
    wiced_buffer_t        response;
    wiced_result_t        retval;
    uint32_t*             data;
    uint16_t              max_info_length = (uint16_t)(WL_MKEEP_ALIVE_FIXED_LEN + keep_alive_packet_info->packet_length);

    wiced_assert("Bad args", (keep_alive_packet_info != NULL) && (keep_alive_packet_info->packet_length > 4) && (keep_alive_packet_info->keep_alive_id <= 3));

    data = wiced_get_iovar_buffer( &buffer, max_info_length, IOVAR_STR_MKEEP_ALIVE );  /* get a buffer to store the keep_alive info into */
    CHECK_IOCTL_BUFFER( data );
    memset( data, 0, max_info_length );
    data[0] = keep_alive_packet_info->keep_alive_id;

    retval = wiced_send_iovar( SDPCM_GET, buffer, &response, SDPCM_STA_INTERFACE );
    if (retval == WICED_SUCCESS)
    {
    	packet_info = (wl_mkeep_alive_pkt_t*)host_buffer_get_current_piece_data_pointer( response );
    	keep_alive_packet_info->packet_length = packet_info->len_bytes;
    	keep_alive_packet_info->period_msec = packet_info->period_msec;
        memcpy( keep_alive_packet_info->packet, packet_info->data, (size_t)MIN(keep_alive_packet_info->packet_length, (host_buffer_get_current_piece_size(response)-WL_MKEEP_ALIVE_FIXED_LEN)) );
        host_buffer_release( response, WICED_NETWORK_RX );
    }

    return retval;
}

wiced_result_t wiced_wifi_disable_keep_alive( uint8_t id )
{
    wiced_keep_alive_packet_t packet_info;
    packet_info.keep_alive_id = id;
    packet_info.period_msec   = 0;
    packet_info.packet_length = 0;
    packet_info.packet        = NULL;
    return wiced_wifi_add_keep_alive( &packet_info );
}

wiced_result_t wiced_wifi_get_associated_client_list( void* client_list_buffer, uint16_t buffer_length )
{
    wiced_buffer_t buffer;
    wiced_buffer_t response;
    wiced_result_t result;

    wiced_maclist_t* data = (wiced_maclist_t*)wiced_get_ioctl_buffer( &buffer, buffer_length );
    CHECK_IOCTL_BUFFER( data );
    data->count = ((wiced_maclist_t*)client_list_buffer)->count;
    result = wiced_send_ioctl( SDPCM_GET, WLC_GET_ASSOCLIST, buffer, &response, SDPCM_AP_INTERFACE );
    if ( result == WICED_SUCCESS )
    {
        memcpy( client_list_buffer, (void*) host_buffer_get_current_piece_data_pointer( response ), (size_t)MIN(host_buffer_get_current_piece_size(response), buffer_length) );
        host_buffer_release( response, WICED_NETWORK_RX );
    }
    return result;
}

wiced_result_t wiced_wifi_test_credentials( wiced_scan_result_t* ap, const uint8_t* security_key, uint8_t key_length )
{
    host_semaphore_type_t semaphore;
    wiced_result_t        result;
    uint32_t              previous_softap_channel = 0;

    host_rtos_init_semaphore( &semaphore );

    /* Check if soft AP interface is up, if so, record its current channel */
    if ( wiced_wifi_is_ready_to_transceive( WICED_AP_INTERFACE ) == WICED_SUCCESS )
    {
        wwd_wifi_get_channel( WICED_AP_INTERFACE, &previous_softap_channel );
    }

    /* Try and join the AP with the credentials provided, but only wait for a short time */
    wiced_wifi_join_specific( ap, security_key, key_length, &semaphore );
    result = host_rtos_get_semaphore( &semaphore, WICED_CREDENTIAL_TEST_TIMEOUT, WICED_FALSE );

    /* Immediately leave so we can go back to the original soft AP channel */
    wiced_wifi_leave();

    /* If applicable, move the soft AP back to its original channel */
    if ( ( previous_softap_channel != 0 ) && ( previous_softap_channel != ap->channel ) )
    {
        wwd_wifi_set_channel( WICED_AP_INTERFACE, previous_softap_channel );
    }

    host_rtos_deinit_semaphore( &semaphore );

    return result;
}

wiced_result_t wiced_wifi_get_ap_info( wl_bss_info_t* ap_info, wiced_security_t* security )
{
    wiced_buffer_t buffer;
    wiced_buffer_t response;
    uint32_t* data;

    /* Read the BSS info */
    data = (uint32_t*) wiced_get_ioctl_buffer( &buffer, sizeof(wl_bss_info_t) + 4 );
    CHECK_IOCTL_BUFFER( data );
    *data = sizeof(wl_bss_info_t) + 4;
    if ( wiced_send_ioctl( SDPCM_GET, WLC_GET_BSS_INFO, buffer, &response, SDPCM_STA_INTERFACE ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    memcpy( ap_info, (void*) (host_buffer_get_current_piece_data_pointer( response ) + 4), sizeof(wl_bss_info_t) );
    host_buffer_release( response, WICED_NETWORK_RX );

    /* Read the WSEC setting */
    data = (uint32_t*) wiced_get_ioctl_buffer( &buffer, sizeof(uint32_t) );
    CHECK_IOCTL_BUFFER( data );
    if ( wiced_send_ioctl( SDPCM_GET, WLC_GET_WSEC, buffer, &response, SDPCM_STA_INTERFACE ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    memcpy( security, (void*) host_buffer_get_current_piece_data_pointer( response ), sizeof(uint32_t) );
    host_buffer_release( response, WICED_NETWORK_RX );

    /* Read the WPA auth setting */
    data = (uint32_t*) wiced_get_ioctl_buffer( &buffer, sizeof(uint32_t) );
    CHECK_IOCTL_BUFFER( data );
    if ( wiced_send_ioctl( SDPCM_GET, WLC_GET_WPA_AUTH, buffer, &response, SDPCM_STA_INTERFACE ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    data = (uint32_t*)host_buffer_get_current_piece_data_pointer( response );
    if (*data == WPA2_AUTH_PSK)
    {
        *security |= WPA2_SECURITY;
    }
    else if (*data == WPA_AUTH_PSK)
    {
        *security |= WPA_SECURITY;
    }
    else
    {
        host_buffer_release( response, WICED_NETWORK_RX );
        return WICED_ERROR;
    }
    host_buffer_release( response, WICED_NETWORK_RX );

    return WICED_SUCCESS;
}


wiced_result_t wiced_wifi_get_ap_name(char*ssid, int*ssid_len, char*bssid)
{
    wiced_buffer_t buffer;
    wiced_buffer_t response;
    uint32_t* data;
    wl_bss_info_t *ap_info;
    
    /* Read the BSS info */
    data = (uint32_t*) wiced_get_ioctl_buffer( &buffer, sizeof(wl_bss_info_t) + 4 );
    CHECK_IOCTL_BUFFER( data );
    *data = sizeof(wl_bss_info_t) + 4;
    if ( wiced_send_ioctl( SDPCM_GET, WLC_GET_BSS_INFO, buffer, &response, SDPCM_STA_INTERFACE ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    ap_info = (wl_bss_info_t *)(host_buffer_get_current_piece_data_pointer( response ) + 4);
	if (ap_info->SSID_len > 32)
		ap_info->SSID_len = 32;
    memcpy(ssid, ap_info->SSID, ap_info->SSID_len);
    *ssid_len = ap_info->SSID_len;
    memcpy(bssid, ap_info->BSSID.octet, 6);
    
    host_buffer_release( response, WICED_NETWORK_RX );

    return WICED_SUCCESS;
}

wwd_result_t wwd_wifi_set_iovar_value( const char* iovar, uint32_t value, wwd_interface_t interface )
{
    wiced_buffer_t buffer;
    uint32_t*      data;

    data = (uint32_t*) wiced_get_iovar_buffer( &buffer, (uint16_t) sizeof(value), iovar );
    CHECK_IOCTL_BUFFER( data );
    *data = value;
    return wiced_send_iovar( SDPCM_SET, buffer, NULL, interface );
}

wwd_result_t wwd_wifi_get_iovar_value( const char* iovar, uint32_t* value, wwd_interface_t interface )
{
    wiced_buffer_t buffer;
    wiced_buffer_t response;

    CHECK_IOCTL_BUFFER( wiced_get_iovar_buffer( &buffer, 4, iovar ) );
    CHECK_RETURN_UNSUPPORTED_OK( wiced_send_iovar( SDPCM_GET, buffer, &response, interface ) );

    *value = *(uint32_t*) host_buffer_get_current_piece_data_pointer( response );
    host_buffer_release( response, WICED_NETWORK_RX );
    return WWD_SUCCESS;
}

wwd_result_t wwd_wifi_set_ioctl_value( uint32_t ioctl, uint32_t value, wwd_interface_t interface )
{
    wiced_buffer_t buffer;
    uint32_t*      data;

    data = (uint32_t*) wiced_get_ioctl_buffer( &buffer, (uint16_t) sizeof(value) );
    CHECK_IOCTL_BUFFER( data );
    *data = value;
    return wiced_send_ioctl( SDPCM_SET, ioctl, buffer, 0, interface );
}

wwd_result_t wwd_wifi_get_ioctl_value( uint32_t ioctl, uint32_t* value, wwd_interface_t interface )
{
    wiced_buffer_t buffer;
    wiced_buffer_t response;

    CHECK_IOCTL_BUFFER( wiced_get_ioctl_buffer( &buffer, (uint16_t) sizeof(*value) ) );
    CHECK_RETURN_UNSUPPORTED_OK( wiced_send_ioctl( SDPCM_GET, ioctl, buffer, &response, interface ) );

    *value = * (uint32_t*) host_buffer_get_current_piece_data_pointer( response );
    host_buffer_release( response, WICED_NETWORK_RX );
    return WWD_SUCCESS;
}

/******************************************************
 *             Static Functions
 ******************************************************/

/** Searches for a specific WiFi Information Element in a byte array
 *
 * Traverse a string of 1-byte tag/1-byte length/variable-length value
 * triples, returning a pointer to the substring whose first element
 * matches tag
 *
 * @note : This funciton has been copied directly from the standard broadcom host driver file wl/exe/wlu.c
 *
 *
 * @param tlv_buf : The byte array containing the Information Elements (IEs)
 * @param buflen  : The length of the tlv_buf byte array
 * @param key     : The Information Element tag to search for
 *
 * @return    NULL : if no matching Information Element was found
 *            Non-Null : Pointer to the start of the matching Information Element
 */

static /*@null@*/ uint8_t* wlu_parse_tlvs( /*@returned@*/ uint8_t* tlv_buf, uint32_t buflen, uint32_t key )
{
    uint8_t* cp     = tlv_buf;
    int32_t  totlen = (int32_t) buflen;

    /* find tagged parameter */
    while ( totlen >= (int32_t) 2 )
    {
        uint32_t tag;
        int32_t len;

        tag = *cp;
        len = (int32_t) *( cp + 1 );

        /* validate remaining totlen */
        if ( ( tag == key ) && ( totlen >= ( len + 2 ) ) )
        {
            return ( cp );
        }

        cp += ( len + 2 );
        totlen -= ( len + 2 );
    }

    return NULL;
}

/** Checks if a WiFi Information Element is a WPA entry
 *
 * Is this body of this tlvs entry a WPA entry? If
 * not update the tlvs buffer pointer/length
 *
 * @note : This funciton has been copied directly from the standard broadcom host driver file wl/exe/wlu.c
 *
 * @param wpaie    : The byte array containing the Information Element (IE)
 * @param tlvs     : The larger IE array to be updated if not a WPA IE
 * @param tlvs_len : The current length of larger IE array
 *
 * @return    WICED_TRUE  : if IE matches the WPA OUI (Organizationally Unique Identifier) and its type = 1
 *            WICED_FALSE : otherwise
 */
static wiced_bool_t wlu_is_wpa_ie( uint8_t** wpaie, uint8_t** tlvs, uint32_t* tlvs_len )
{
    uint8_t* ie = *wpaie;

    /* If the contents match the WPA_OUI and type=1 */
    if ( ( ie[1] >= (uint8_t) 6 ) &&
         ( memcmp( &ie[2], WPA_OUI "\x01", (size_t) 4 ) == 0 ) )
    {
        return WICED_TRUE;
    }

    /* point to the next ie */
    ie += ie[1] + 2;
    /* calculate the length of the rest of the buffer */
    *tlvs_len -= (uint32_t) ( ie - *tlvs );
    /* update the pointer to the start of the buffer */
    *tlvs = ie;

    return WICED_FALSE;
}


wiced_result_t wiced_wifi_get_wifi_version( char* version, uint8_t length )
{
    wiced_buffer_t buffer;
    wiced_buffer_t response;
    wiced_result_t result;

    if ( NULL == wiced_get_iovar_buffer( &buffer, length, IOVAR_STR_VERSION ) )
    {
        return WICED_ERROR;
    }
    result = wiced_send_iovar( SDPCM_GET, buffer, &response, SDPCM_STA_INTERFACE );
    if ( result == WICED_SUCCESS )
    {
        memcpy( version, host_buffer_get_current_piece_data_pointer( response ), length );
        host_buffer_release( response, WICED_NETWORK_RX );
    }
    return result;
}

wiced_result_t wiced_wifi_set_maxassoc( uint32_t n )
{
    wiced_buffer_t buffer;
    wiced_result_t result;
    uint32_t* data;
    
    data = (uint32_t*) wiced_get_iovar_buffer( &buffer, 4, IOVAR_STR_MAXASSOC );
    CHECK_IOCTL_BUFFER( data );
    *data = n;
    result = wiced_send_iovar( SDPCM_SET, buffer, NULL, SDPCM_STA_INTERFACE ); 
    
    return result;
}

wiced_result_t wlan_set_mpc( int enable );
wiced_result_t wlan_set_mpc( int enable )
{
    wiced_buffer_t buffer;
    wiced_result_t result;
    int* data;
    
    data = (int*) wiced_get_iovar_buffer( &buffer, 4, "mpc" );
    CHECK_IOCTL_BUFFER( data );
    *data = enable;
    result = wiced_send_iovar( SDPCM_SET, buffer, NULL, SDPCM_STA_INTERFACE ); 
    
    return result;
}


