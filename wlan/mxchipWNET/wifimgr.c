/* WIFI mamagement file */
#include "string.h"
#include "stdlib.h"
//#include "wiced_utilities.h"
//#include "wiced_dct.h"
//#include "wiced_platform.h"
//#include "wiced_rtos.h"
//#include "wiced_tcpip.h"
//#include "wiced_time.h"
#include "wiced_wifi.h"
#include "wiced_defaults.h"
#include "wiced_network.h"
#include "wiced_management.h"
#include "wwd_debug.h"
#include "wwd_assert.h"

#include "wwd_events.h"
//#include "dns_redirect.h"
#include "lwip/inet.h"

#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "arch/perf.h"
#include "lwip/udp.h"

//#include "mxchipWNET.h"
#include "wifimgr.h"
#include "mxchip_debug.h"


#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wpointer-sign"
#pragma GCC diagnostic ignored "-Wformat"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wimplicit-function-declaration"
#endif /* ifdef __GNUC__ */


/******************************************************
 *                      Macros
 ******************************************************/

#define CMP_MAC( a, b )  (((a[0])==(b[0]))&& \
                          ((a[1])==(b[1]))&& \
                          ((a[2])==(b[2]))&& \
                          ((a[3])==(b[3]))&& \
                          ((a[4])==(b[4]))&& \
                          ((a[5])==(b[5])))

#define NULL_MAC( a )  (((a[0])==0)&& \
                        ((a[1])==0)&& \
                        ((a[2])==0)&& \
                        ((a[3])==0)&& \
                        ((a[4])==0)&& \
                        ((a[5])==0))

#define DUAL_MODE_RETRY_INTERVAL 60*1000
#define STA_MODE_RETRY_INTERVAL 100
#define STA_MODE_MAX_SCANTIME 10*1000

#define MAX_SCAN_NUM    (32)
#define WLC_EVENT_MSG_LINK      (0x01)    /** link is up */

typedef enum {
    STOPPED,
	SLEEPING,
	SCANING,
	JOINING,
	CONNECTED,
	ADVTRY,
}STA_STATE;

enum {
    WIFI_ERROR_INIT = 1,
    WIFI_ERROR_NOGW,
    WIFI_ERROR_NOBUS_CREDIT,
};


#define USER_SCAN_FLAG      0x01
#define USER_ADV_SCAN_FLAG  0x02
#define MF_SCAN_FLAG        0x04
#define JOIN_SCAN_FLAG      0x08
#define EASYLINK_SCAN_FLAG  0x10
#define CMD_SCAN_FLAG       0x20

/******************************************************
 *                    Structures
 ******************************************************/
    
struct mxchip_timer {
    uint32_t timeout;
    void (*handler)(void);
    struct mxchip_timer *next;
    int valid;
};

typedef struct
{
    char                ssid[33];
    char                key[65];
    char                bssid[6];
    int                 mode;
    int                 valid;
} config_ap_entry_t;

/******************************************************
 *               Variables Definitions
 ******************************************************/
static uint8_t sta_connected = 0, uap_connected = 0;
static uint32_t sta_retry_interval = STA_MODE_RETRY_INTERVAL;
static STA_STATE sta_state = STOPPED;
static const wiced_event_num_t      link_events[]  = { WLC_E_LINK, WLC_E_ROAM, WLC_E_NONE };
#if 0
struct mxchip_timer *timer_head = NULL;
#endif

static config_ap_entry_t g_ap_entry;
static apinfo_adv_t _apinfo;

wiced_scan_result_t     result_buff[1];
wiced_scan_result_t *p_scan_buff = result_buff;
static wiced_scan_result_t *p_result_head = NULL;

static ap_list_info_t gApList;
static ap_list_info_adv_t gApAdvList;

static wiced_ip_setting_t sta_ip_settings, uap_ip_settings;
static wiced_ip_address_t dns_server_address;
static int sta_dhcp_mode = 0;

static uint32_t timer_thread_wait = NEVER_TIMEOUT;
static mico_semaphore_t timer_sem, wifimgr_sem, scan_sem;
static int user_req_scan = 0;
static int user_stop_sta = 0;

static int softap_channel = 6;
static int no_pmk_cache = 0;
static char active_scan_ssid[33];

static int health_monitor_enabled = 1;
static uint8_t connected_ap_bssid[6];

/******************************************************
 *               Function Definitions
 ******************************************************/
static void free_ap_list(void);
#if 0
static void mxchip_timer_tick(void);
#endif
static void health_monitor_thread(uint32_t arg);
static void wifimgr_thread(uint32_t arg);
static void update_connected_ap_info(void);

extern int user_scan(int advance_scan);

extern void sta_up(void);
extern void wifi_reboot_event(int type);
extern void ApListAdvCallback(void *pApAdvList);

void wifimgr_debug_enable(bool enable)
{
    if (enable)
        system_debug_enable(2, (debug_printf)printf);
    else
        system_debug_enable(0, (debug_printf)NULL);
}

void set_no_pmk_cache(void)
{
    no_pmk_cache = 1;
}

static uint8_t rssi_2_quility(int rssi)
{
    if (rssi >= -55)
        return 100;
    if (rssi <= -95)
        return 0;
    return (rssi+95)*5/2;
}

static int sectype_bcm2mx(wiced_security_t sec)
{
    int ret;

    switch (sec) {
    case WICED_SECURITY_OPEN:
        ret = SEC_TYPE_NONE;
        break;
    case WICED_SECURITY_WEP_PSK:
        ret = SEC_TYPE_WEP;
        break;
    case WICED_SECURITY_WPA_TKIP_PSK:
        ret = SEC_TYPE_WPA_TKIP;
        break;
    case WICED_SECURITY_WPA_AES_PSK:
        ret = SEC_TYPE_WPA_AES;
        break;
    case WICED_SECURITY_WPA2_AES_PSK:
        ret = SEC_TYPE_WPA2_AES;
        break;
    case WICED_SECURITY_WPA2_TKIP_PSK:
        ret = SEC_TYPE_WPA2_TKIP;
        break;
    case WICED_SECURITY_WPA2_MIXED_PSK:
        ret = SEC_TYPE_WPA2_MIXED;
        break;
    default:
        ret = SEC_TYPE_NONE;
        break;
    }

    return ret;
}

static int sectype_mx2bcm(int sec)
{
    int ret;

    switch (sec) {
    case SEC_TYPE_NONE:
        ret = WICED_SECURITY_OPEN;
        break;
    case SEC_TYPE_WEP:
        ret = WICED_SECURITY_WEP_PSK;
        break;
    case SEC_TYPE_WPA_TKIP:
        ret = WICED_SECURITY_WPA_TKIP_PSK;
        break;
    case SEC_TYPE_WPA_AES:
        ret = WICED_SECURITY_WPA_AES_PSK;
        break;
    case SEC_TYPE_WPA2_AES:
        ret = WICED_SECURITY_WPA2_AES_PSK;
        break;
    case SEC_TYPE_WPA2_TKIP:
        ret = WICED_SECURITY_WPA2_TKIP_PSK;
        break;
    case SEC_TYPE_WPA2_MIXED:
        ret = WICED_SECURITY_WPA2_MIXED_PSK;
        break;
    default:
        ret = WICED_SECURITY_OPEN;
        break;
    }

    return ret;
}

static void enable_ps(void)
{
//    wiced_wifi_enable_powersave();
    wiced_wifi_enable_powersave_with_throughput(100);
}

static void set_sta_connection(uint8_t connection)
{
	if (sta_connected == connection)
		return;
    sta_connected = connection;
    
	if (connection == 1) {
        system_debug_printf(SYSTEM_DEBUG_DEBUG, "Connected\r\n");
		wifi_status_callback(WIFI_STATE_STA_UP);
	} else {
	    system_debug_printf(SYSTEM_DEBUG_DEBUG, "Disconnected\r\n");
		wifi_status_callback(WIFI_STATE_STA_DOWN);
    }

    if (is_ps_enabled()) {
		if (connection == 1)
			SetTimer(0, enable_ps);
    }
	
}

static void stopscan(void)
{
    wiced_wifi_stop_scan();
    msleep(10);
}

int get_sta_connection(void)
{
	return sta_connected;
}

static void set_uap_connection(uint8_t connection)
{
	if (uap_connected == connection)
		return;

    uap_connected = connection;
	if (connection == 1) {
		wifi_status_callback(WIFI_STATE_UAP_UP);
	} else {
		wifi_status_callback(WIFI_STATE_UAP_DOWN);
    }

}

int get_uap_connection(void)
{
	return uap_connected;
}

int get_connection(void)
{
	if (sta_connected == 1)
        return 1;
    if (uap_connected == 1)
        return 1;
    return 0;
}

void set_sta_retry_interval(uint32_t interval)
{
    if (interval < 100)
        interval = 100;
    
    sta_retry_interval = interval;
}

void sta_up()
{
    set_sta_connection(1);
}

static void sta_link_up()
{
}

void sta_down()
{
    set_sta_connection(0);
    wiced_ip_down( WICED_STA_INTERFACE ) ;
}

static void _uap_up(void)
{
    if (uap_ip_settings.ip_address.version == 0)
        wiced_ip_up(WICED_AP_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL);
    else
        wiced_ip_up(WICED_AP_INTERFACE, WICED_USE_INTERNAL_DHCP_SERVER, &uap_ip_settings);

    set_uap_connection(1);
    
    if (DUAL_MODE_RETRY_INTERVAL > sta_retry_interval)
        sta_retry_interval = DUAL_MODE_RETRY_INTERVAL;
}

static void uap_down()
{
    wiced_wifi_stop_ap();

    set_uap_connection(0);
}

static void set_static_ip(wiced_interface_t iface, uint8_t *ip, uint8_t*mask, uint8_t*gw, uint8_t*dns)
{
    wiced_ip_setting_t *pip;
    uint32_t tmp;
    
    if (iface == WICED_AP_INTERFACE)
        pip = &uap_ip_settings;
    else
        pip = &sta_ip_settings;

    tmp = inet_addr((const char*)ip);
    SET_IPV4_ADDRESS((pip->ip_address), htonl(tmp));
    tmp = inet_addr((const char*)mask);
    if (tmp == 0xFFFFFFFF)
        tmp = 0x00FFFFFF;// if not set valid netmask, set as 255.255.255.0
    SET_IPV4_ADDRESS((pip->netmask), htonl(tmp));
    tmp = inet_addr((const char*)gw);
    SET_IPV4_ADDRESS((pip->gateway), htonl(tmp));

    if (iface == WICED_STA_INTERFACE) {
        tmp = inet_addr((const char*)dns);
        SET_IPV4_ADDRESS(dns_server_address, htonl(tmp));
    }
}

static int sta_connect_with_apinfo(apinfo_adv_t *ap, char*key, int key_len)
{
    wiced_scan_result_t p;
    void *pkey = key, *rptkey;
	int rtpkeylen;
    char pmk[65];
#ifdef WOLFSSL_MAX_STRENGTH	
	if ((ap->security <=  SEC_TYPE_WEP) || (ap->security >  SEC_TYPE_WPA2_MIXED))
		return WICED_ERROR;
#endif

    memset(&p, 0, sizeof(wiced_scan_result_t));
    p.security = sectype_mx2bcm(ap->security);

    p.SSID.len = strlen(ap->ssid);
    if (p.SSID.len > 32)
        p.SSID.len = 32;
    memcpy(p.SSID.val, ap->ssid, 32);
    memcpy(p.BSSID.octet, ap->bssid, 6);
    p.channel = ap->channel;
	rptkey = key;
	rtpkeylen = key_len;
    if (ap->security == SEC_TYPE_WEP) {
        wiced_wep_key_t wep_key;

        memset(&wep_key, 0, sizeof(wep_key));
        wep_key.index = 0;
        if ((key_len == 10) || (key_len == 26)) {
            str2hex(key, wep_key.data, 32);
            wep_key.length = key_len/2;
        } else {
            wep_key.length = key_len;
            memcpy(wep_key.data, key, key_len);
        }
        pkey = &wep_key;
        key_len = 1;
    }
    
    if ( (no_pmk_cache==0) && !(NULL_MAC(ap->bssid)) && wiced_wifi_join_specific(&p, (uint8_t*)pkey, key_len, NULL ) == WICED_SUCCESS)
    {
        if ((ap->security>SEC_TYPE_WEP) && ( key_len != WSEC_MAX_PSK_LEN ))
        {
            if (WICED_SUCCESS == wiced_wifi_get_pmk(key, key_len, pmk)) {
                pmk[64] = 0;
                rptkey = pmk;
                rtpkeylen = 64;
            }
        } 

		SetTimer(10, update_connected_ap_info);
        connected_ap_info(ap, rptkey, rtpkeylen);
        return WICED_SUCCESS;
    }

    /* Try scan and join AP */
    if (wiced_wifi_join((char*)ap->ssid, p.security, (uint8_t*)pkey, key_len, NULL) == WICED_SUCCESS)
    {
        /* Extract the calculated PMK and store it in the DCT to speed up future associations */
        if ((ap->security>SEC_TYPE_WEP)&&( key_len != WSEC_MAX_PSK_LEN ))
        {
            if (WICED_SUCCESS == wiced_wifi_get_pmk(key, key_len, pmk)) {
                pmk[64] = 0;
                rptkey = pmk;
                rtpkeylen = 64;
            }
        } 

		SetTimer(10, update_connected_ap_info);
        connected_ap_info(ap, rptkey, rtpkeylen);
        return WICED_SUCCESS;
    }
    else
    {
        return WICED_ERROR;
    }
}

int try_adv_connect(void)
{
	int ret = sta_connect_with_apinfo(&_apinfo, g_ap_entry.key, strlen(g_ap_entry.key));

	if (ret != 0)
		return ret;
	
	system_debug_printf(SYSTEM_DEBUG_DEBUG, "Fast connect ap success, request IP..\r\n");
    if (sta_dhcp_mode == 1) {
        ret = wiced_ip_up(WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL);
        if (ret != WICED_SUCCESS) {
            sta_state = SLEEPING;
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "Request IP fail.\r\n");
            mico_rtos_set_semaphore(&wifimgr_sem);
            return ret;
        } else {
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "Request IP SUCCESS, UP.\r\n");
            station_up();
        }
    } else {
        system_debug_printf(SYSTEM_DEBUG_DEBUG, "Static up.\r\n");
        wiced_ip_up(WICED_STA_INTERFACE, WICED_USE_STATIC_IP, &sta_ip_settings);
        dns_client_add_server_address( dns_server_address );
        sta_up();
    }
    
    sta_state = CONNECTED;
    system_debug_printf(SYSTEM_DEBUG_DEBUG, "Set STA connected\r\n");
	return ret;
}

int StartAdvNetwork(network_InitTypeDef_adv_st* mxconfig)
{
    int ret = WICED_SUCCESS;

	system_debug_printf(SYSTEM_DEBUG_DEBUG, "Adv connect {%s %s(%d)}\r\n",
		mxconfig->ap_info.ssid, mxconfig->key, mxconfig->key_len);
    sta_disconnect();
	while(user_stop_sta == 1)
		msleep(1);
	memcpy(&_apinfo, &mxconfig->ap_info, sizeof(_apinfo));
    set_sta_retry_interval(mxconfig->wifi_retry_interval);
    // save AP information.
    memset(&g_ap_entry, 0, sizeof(g_ap_entry));
    g_ap_entry.mode = AP_CLIENT_MODE;
    strncpy(g_ap_entry.ssid, mxconfig->ap_info.ssid, 32);
    if (mxconfig->key_len > 64)
        mxconfig->key_len = 64;
    //memcpy(g_ap_entry.bssid, mxconfig->ap_info.bssid, 6);
    strncpy(g_ap_entry.key, mxconfig->key, mxconfig->key_len);
    
    g_ap_entry.valid = 1;
    if (mxconfig->dhcpMode == 1)
        memset(&sta_ip_settings, 0, sizeof(sta_ip_settings));
    else
        set_static_ip(WICED_STA_INTERFACE, mxconfig->local_ip_addr, mxconfig->net_mask, 
            mxconfig->gateway_ip_addr, mxconfig->dnsServer_ip_addr);
    sta_dhcp_mode = mxconfig->dhcpMode;

	//if (sta_connect_with_apinfo(&mxconfig->ap_info, mxconfig->key, mxconfig->key_len) != 0) 
	{
        sta_state = ADVTRY;
        mico_rtos_set_semaphore(&wifimgr_sem);
        return WICED_SUCCESS;
    }
#if 0
    system_debug_printf(SYSTEM_DEBUG_DEBUG, "Fast connect ap success, request IP..\r\n");
    if (mxconfig->dhcpMode == 1) {
        ret = wiced_ip_up(WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL);
        if (ret != WICED_SUCCESS) {
            sta_state = SLEEPING;
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "Request IP fail.\r\n");
            mico_rtos_set_semaphore(&wifimgr_sem);
            return ret;
        } else {
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "Request IP SUCCESS, UP.\r\n");
            station_up();
        }
    } else {
        system_debug_printf(SYSTEM_DEBUG_DEBUG, "Static up.\r\n");
        wiced_ip_up(WICED_STA_INTERFACE, WICED_USE_STATIC_IP, &sta_ip_settings);
        dns_client_add_server_address( dns_server_address );
        sta_up();
    }
    
    sta_state = CONNECTED;
    system_debug_printf(SYSTEM_DEBUG_DEBUG, "Set STA connected\r\n");
#endif	
	return ret;
}


int StartNetworkWithbssid(mxchip_wifi_adv_config_t* mxconfig)
{
    int ret = WICED_SUCCESS;

    sta_disconnect();
	while(user_stop_sta == 1)
		msleep(1);
    set_sta_retry_interval(mxconfig->wifi_retry_interval);
    // save AP information.
    memset(&g_ap_entry, 0, sizeof(g_ap_entry));
    g_ap_entry.mode = AP_CLIENT_MODE;
    strncpy(g_ap_entry.ssid, mxconfig->ap_info.ssid, 32);
    if (mxconfig->key_len > 64)
        mxconfig->key_len = 64;
    memcpy(g_ap_entry.bssid, mxconfig->ap_info.bssid, 6);
    strncpy(g_ap_entry.key, mxconfig->key, mxconfig->key_len);
    
    g_ap_entry.valid = 1;
    if (mxconfig->dhcpMode == 1)
        memset(&sta_ip_settings, 0, sizeof(sta_ip_settings));
    else
        set_static_ip(WICED_STA_INTERFACE, mxconfig->local_ip_addr, mxconfig->net_mask, 
            mxconfig->gateway_ip_addr, mxconfig->dnsServer_ip_addr);
    sta_dhcp_mode = mxconfig->dhcpMode;
    
	if (sta_connect_with_apinfo(&mxconfig->ap_info, mxconfig->key, mxconfig->key_len) != 0) {
        sta_state = SLEEPING;
        mico_rtos_set_semaphore(&wifimgr_sem);
        return WICED_SUCCESS;
    }
	
    if (mxconfig->dhcpMode == 1) {
        ret = wiced_ip_up(WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL);
        if (ret != WICED_SUCCESS) {
            sta_state = SLEEPING;
            mico_rtos_set_semaphore(&wifimgr_sem);
        } else {
            station_up();
        }
    } else {
        wiced_ip_up(WICED_STA_INTERFACE, WICED_USE_STATIC_IP, &sta_ip_settings);
        dns_client_add_server_address( dns_server_address );
        sta_up();
    }
    
    sta_state = CONNECTED;
    
	return ret;
}

int StartNetwork(network_InitTypeDef_st* mxconfig)
{
    int ret = WICED_SUCCESS;
    int key_len;
    char ssid[33];

    ssid[32] = '\0';
    strncpy(ssid, mxconfig->wifi_ssid, 32);

    key_len = strlen(mxconfig->wifi_key);
    if (key_len > 64)
        key_len = 64;
    system_debug_printf(SYSTEM_DEBUG_DEBUG, "connect {%s %s}\r\n",
		ssid, mxconfig->wifi_key);
    if (mxconfig->wifi_mode == WIFI_MODE_STA) {
        sta_disconnect();
		while(user_stop_sta == 1)
			msleep(1);
        set_sta_retry_interval(mxconfig->wifi_retry_interval);
        if (mxconfig->dhcpMode == 1)
            memset(&sta_ip_settings, 0, sizeof(sta_ip_settings));
        else
            set_static_ip(WICED_STA_INTERFACE, mxconfig->local_ip_addr, mxconfig->net_mask, 
                mxconfig->gateway_ip_addr, mxconfig->dnsServer_ip_addr);
        sta_dhcp_mode = mxconfig->dhcpMode;
        // save AP information.
        memset(&g_ap_entry, 0, sizeof(g_ap_entry));
        g_ap_entry.mode = AP_CLIENT_MODE;
        strncpy(g_ap_entry.ssid, ssid, 32);
        strncpy(g_ap_entry.key, mxconfig->wifi_key, 64);
        
        g_ap_entry.valid = 1;
        

        sta_state = SLEEPING;
        mico_rtos_set_semaphore(&wifimgr_sem);
    } else {
        if (uap_connected == 1) {
            wiced_ip_down(WICED_AP_INTERFACE);
            uap_down();
            uap_connected = 0;
        } 
        if (mxconfig->dhcpMode == 1)
            memset(&uap_ip_settings, 0, sizeof(uap_ip_settings));
        else
            set_static_ip(WICED_AP_INTERFACE, mxconfig->local_ip_addr, mxconfig->net_mask, 
                mxconfig->gateway_ip_addr, mxconfig->dnsServer_ip_addr);
        msleep(50);

        if (key_len < 8)
            ret = wiced_wifi_start_ap(ssid, WICED_SECURITY_OPEN, NULL, 0, softap_channel);
        else
            ret = wiced_wifi_start_ap(ssid, WICED_SECURITY_WPA2_AES_PSK, mxconfig->wifi_key, key_len, softap_channel);

        if (ret == WICED_SUCCESS)
            SetTimer(100, _uap_up);
    }

    return ret;
}

void wlan_set_channel(int channel)
{
    softap_channel = channel;
}

int sta_disconnect(void)
{
    system_debug_printf(SYSTEM_DEBUG_DEBUG, "STA disconnect\r\n");
    user_stop_sta = 1;
    mico_rtos_set_semaphore(&wifimgr_sem);
    
    
    return WICED_SUCCESS;
}

int uap_stop(void)
{
    wiced_ip_down(WICED_AP_INTERFACE);
    if (uap_connected == 1) {
        uap_down();
    } 
    return WICED_SUCCESS;
}

int wlan_disconnect( void )
{
    system_debug_printf(SYSTEM_DEBUG_DEBUG, "wlan disconnect\r\n");
    user_stop_sta = 1;
    mico_rtos_set_semaphore(&wifimgr_sem);
    
    wiced_ip_down(WICED_AP_INTERFACE);
    if (uap_connected == 1) {
        uap_down();
        uap_connected = 0;
    } 

    return WICED_SUCCESS;
}

static void append_ap_list(wiced_scan_result_t *result, ap_list_info_t *ApList, int num)
{
	int i, j;
    uint8_t quality = rssi_2_quility(result->signal_strength);
    ap_info_t * list = ApList->ApList;
    
    if (ApList->ApNum==num)
        return;
    
	for(i=0;i<num;i++) {
		if(quality > list[i].ApPower)
			break;
	}
    
	for (j=num; j>i; j--) {
        memcpy(&list[j], &list[j-1], sizeof(ap_info_t));
	}
    
	strncpy(list[i].ssid, (char*)result->SSID.val, 32);
    list[i].ApPower = quality;

}

static void append_ap_adv_list(wiced_scan_result_t *result, ap_list_info_adv_t *ApList, int num)
{
	int i, j;
    //uint8_t quality = rssi_2_quility(result->signal_strength);
    ap_info_adv_t * list = ApList->ApList;
    
    if (ApList->ApNum==num)
        return;
    
    for ( i = 0; i < num; i++ ) {
        if ( result->signal_strength > list[i].signal_strength )
            break;
    }
    
	for (j=num; j>i; j--) {
        memcpy(&list[j], &list[j-1], sizeof(ap_info_adv_t));
	}
    
	strncpy(list[i].ssid, (char*)result->SSID.val, 32);
    list[i].signal_strength = result->signal_strength;
    list[i].channel = result->channel;
    memcpy(list[i].bssid, result->BSSID.octet, 6);
    list[i].security = sectype_bcm2mx(result->security);
}

static void get_scan_result_list(ap_list_info_t *ApList)
{
    wiced_scan_result_t* p;
    int i;
    
    p = p_result_head;
    ApList->ApNum = 0;
	ApList->ApList = NULL;
    while(p != NULL) {
        ApList->ApNum++;
        p = p->next;
    }
    if (ApList->ApNum > MAX_SCAN_NUM)
        ApList->ApNum = MAX_SCAN_NUM;

    if (ApList->ApNum > 0) {
        ApList->ApList = (ap_info_t *)malloc(sizeof(ap_info_t) * ApList->ApNum);
    }
    
    if (ApList->ApList == NULL) {
        ApList->ApNum = 0;
        return;
    }

    p = p_result_head;
    i = 0;
    while(p != NULL) {
        append_ap_list(p, ApList, i++);
        p = p->next;
        if (i == MAX_SCAN_NUM)
            break;
    }
}

static void scan_finished(void)
{
    scan_report(&gApList);

    if (gApList.ApList != NULL) {
        free(gApList.ApList);
		gApList.ApList = NULL;
	}
}

static void get_scan_result_adv_list(ap_list_info_adv_t *ApList)
{
    wiced_scan_result_t* p;
    int i;
    
    p = p_result_head;
    ApList->ApNum = 0;
    while(p != NULL) {
        ApList->ApNum++;
        p = p->next;
    }
    if (ApList->ApNum > MAX_SCAN_NUM)
        ApList->ApNum = MAX_SCAN_NUM;

    if (ApList->ApNum > 0) {
        ApList->ApList = (ap_info_adv_t *)malloc(sizeof(ap_info_adv_t) * ApList->ApNum);
    }
    
    if (ApList->ApList == NULL) {
        ApList->ApNum = 0;
        return;
    }

    p = p_result_head;
    i = 0;
    while(p != NULL) {
        append_ap_adv_list(p, ApList, i++);
        p = p->next;
        if (i == MAX_SCAN_NUM)
            break;
    }
}

static void scan_adv_finished(void)
{
    ApListAdvCallback((void*)&gApAdvList);

    if (gApAdvList.ApList != NULL)
        free(gApAdvList.ApList);
}

static int select_ap_to_join(void)
{
    wiced_scan_result_t* p;
    int i;
    int ret = WICED_NOTFOUND;
    int retry_count = 0;
    int match_ap_num = 0;
	uint8_t *key = g_ap_entry.key;
	int keylen = strlen(g_ap_entry.key);
	wiced_wep_key_t wep_key;
	uint32_t channel;
	
#define MAX_RETRY_COUNT 5

    p = p_result_head;
    while(p!= NULL) {
        system_debug_printf(SYSTEM_DEBUG_DEBUG, 
               "%02x-%02x-%02x-%02x-%02x-%02x, rssi %d, channel %d\r\n",
               p->BSSID.octet[0],p->BSSID.octet[1],p->BSSID.octet[2],
               p->BSSID.octet[3],p->BSSID.octet[4],p->BSSID.octet[5],
               p->signal_strength, p->channel);
        p = p->next;
    }
    p = p_result_head;
    while(p != NULL) {
        if (!NULL_MAC(g_ap_entry.bssid)) {
            if (!CMP_MAC(g_ap_entry.bssid, p->BSSID.octet)) {
                p = p->next;
                continue;
            }
        } else if (strncmp(g_ap_entry.ssid, p->SSID.val, 32) != 0) {
            p = p->next;
            continue;
        }
#ifdef WOLFSSL_MAX_STRENGTH
		if ((p->security == WICED_SECURITY_OPEN) ||
			(p->security & WEP_ENABLED)){
			system_debug_printf(SYSTEM_DEBUG_DEBUG, 
               "%02x-%02x-%02x-%02x-%02x-%02x, security %x, bypass\r\n",
               p->BSSID.octet[0],p->BSSID.octet[1],p->BSSID.octet[2],
               p->BSSID.octet[3],p->BSSID.octet[4],p->BSSID.octet[5],
               p->security);
			p = p->next;
			continue;
		}
#endif
        { // find the sepcific AP, connect it
            match_ap_num++;
            retry_count = 0;
            system_debug_printf(SYSTEM_DEBUG_DEBUG, 
               "find ap: %02x-%02x-%02x-%02x-%02x-%02x, sec %x, onchannel %d\r\n",
               p->BSSID.octet[0],p->BSSID.octet[1],p->BSSID.octet[2],
               p->BSSID.octet[3],p->BSSID.octet[4],p->BSSID.octet[5],
               p->security, p->on_channel);
			if (p->security == WICED_SECURITY_WEP_PSK) {
                memset(&wep_key, 0, sizeof(wep_key));
                wep_key.index = 0;
                if ((keylen == 10) || (keylen == 26)) {
                    str2hex(g_ap_entry.key, wep_key.data, 32);
                    wep_key.length = keylen/2;
                } else {
                    wep_key.length = keylen;
                    memcpy(wep_key.data, g_ap_entry.key, keylen);
                }
                key = (uint8_t*)&wep_key;
				keylen = 1;
            }
            if (no_pmk_cache==0) {
                do {
					system_debug_printf(SYSTEM_DEBUG_DEBUG, "join specific sec %d keylen %d\r\n",
						p->security, keylen);
                    ret = wiced_wifi_join_specific(p, key, keylen, NULL );
                    retry_count++;
                    if (retry_count == MAX_RETRY_COUNT)
                        break;
                    if ((ret == WICED_SUCCESS)||(ret == WICED_NOT_KEYED)) {
                        break;
                    }
                } while (1);
            }
			if (ret != WICED_SUCCESS) {
				retry_count = 0;
                do {
					system_debug_printf(SYSTEM_DEBUG_DEBUG, "join sec %d keylen %d\r\n",
						p->security, keylen);
                    /* If join-specific failed, try scan and join AP */
                    ret = wiced_wifi_join( (char*) p->SSID.val, p->security, key, keylen, NULL );
                    retry_count++;
                    if (retry_count == MAX_RETRY_COUNT)
                        break;
                    if ((ret == WICED_SUCCESS)||(ret == WICED_NOT_KEYED)) {
                        break;
                    }
                } while (1);
            }
            if (ret == WICED_SUCCESS)
            {
                apinfo_adv_t ap_info;

                system_debug_printf(SYSTEM_DEBUG_DEBUG, "join succes, request IP...\r\n");
                strncpy(ap_info.ssid, p->SSID.val, 32);
                memcpy(ap_info.bssid, p->BSSID.octet, 6);
				wiced_wifi_get_channel((uint32_t*)&channel);
				ap_info.channel = channel;
                ap_info.security = sectype_bcm2mx(p->security);
				memcpy(connected_ap_bssid, ap_info.bssid,6);
                if ((no_pmk_cache==0) && ((p->security & WPA_SECURITY) || (p->security & WPA2_SECURITY))) {
                    if (WICED_SUCCESS == wiced_wifi_get_pmk( g_ap_entry.key, strlen(g_ap_entry.key), g_ap_entry.key)) 
                        connected_ap_info(&ap_info, g_ap_entry.key, 64);
                    else
                        connected_ap_info(&ap_info, g_ap_entry.key, strlen(g_ap_entry.key));
                } else {
                    connected_ap_info(&ap_info, g_ap_entry.key, strlen(g_ap_entry.key));
                }
                
                if (sta_dhcp_mode == 1) {
                    if (WICED_SUCCESS != wiced_ip_up(WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL)) {
                        system_debug_printf(SYSTEM_DEBUG_DEBUG, "DHCP timeout\r\n");
                        return WICED_ERROR;
                    } else {
                        station_up();
                    }
                } else {
                    wiced_ip_up(WICED_STA_INTERFACE, WICED_USE_STATIC_IP, &sta_ip_settings);
                    dns_client_add_server_address( dns_server_address );
                    sta_up();
                }
                return WICED_SUCCESS;
            }else {
                system_debug_printf(SYSTEM_DEBUG_DEBUG, "join fail ret %d\r\n", ret);
                join_fail(ret);
            }
        }
                
        p = p->next;
    }
    
    if (match_ap_num == 0)
        join_fail(WICED_NOTFOUND);
    system_debug_printf(SYSTEM_DEBUG_DEBUG, "retry later\r\n");
    
    return WICED_ERROR;
}

static void insert_result_by_rssi(wiced_scan_result_t* p)
{
    wiced_scan_result_t *next, *prev;

    if ((p_result_head == NULL) || (p_result_head->signal_strength <= p->signal_strength)) {
        p->next = p_result_head;
        p_result_head = p;
        goto DONE;
    }
    
    prev = p_result_head;
    next = p_result_head->next;
    while(next != NULL) {
        if (next->signal_strength <= p->signal_strength) {
            p->next = next;
            prev->next = p;
            goto DONE;
        } else {
            prev = prev->next;
            next = prev->next;
        }
    }
    prev->next = p;

DONE:
    return;
}

/**
 *  Scan result callback
 *  Called whenever a scan result is available
 *
 *  @param result_ptr : pointer to pointer for location where result is stored. The inner pointer
 *                      can be updated to cause the next result to be put in a new location.
 *  @param user_data : unused
 */
static void scan_results_handler( wiced_scan_result_t ** result_ptr, void * user_data )
{
    wiced_scan_result_t* record ;
    wiced_scan_result_t* p;

    if ( result_ptr == NULL )
    {

        mico_rtos_set_semaphore(&scan_sem);
        
        return;
    }
	
	record = ( *result_ptr );
    /* Check the list of BSSID values which have already in the ap list */
    p = p_result_head;
    while (p!=NULL) {
        if (CMP_MAC( p->BSSID.octet, record->BSSID.octet ) )
        {
            /* already seen this BSSID, if the saved signal is off channel and this time is valid siganl, replace it */
            if ((record->on_channel == 1) &&(p->on_channel != 1))
                p->signal_strength = record->signal_strength;
            memset( *result_ptr, 0, sizeof(wiced_scan_result_t) );// clean and let it reused
            return;
        }
        p = p->next;
    }
    
    /* New BSSID - add it to the list */
    p = (wiced_scan_result_t*)malloc(sizeof(wiced_scan_result_t));
    if (p != NULL) {

        memcpy(p, record, sizeof(wiced_scan_result_t));
        insert_result_by_rssi(p);
    }

    return;

}

/*!
 ******************************************************************************
 * Scans for access points and prints out results
 *
 * @return  0 for success, otherwise error
 */

int user_scan(int advance_scan)
{
    if (advance_scan)
        user_req_scan |= USER_ADV_SCAN_FLAG;
    else
        user_req_scan |= USER_SCAN_FLAG;
    
    mico_rtos_set_semaphore(&wifimgr_sem);
    return 0;
}

int user_active_scan(char *ssid, int advance_scan)
{
    if (advance_scan)
        user_req_scan |= USER_ADV_SCAN_FLAG;
    else
        user_req_scan |= USER_SCAN_FLAG;

    if (ssid[0] != 0)
        strncpy(active_scan_ssid, ssid, 33);
    mico_rtos_set_semaphore(&wifimgr_sem);
    return 0;
}

static void free_ap_list(void)
{
    wiced_scan_result_t* p, *q;

    p = p_result_head;
    p_result_head = NULL;
    
    while(p != NULL) {
        q = p->next;
        free(p);
        p = q;
    }   
}

static void update_connected_ap_info(void)
{
	uint8_t bssid[6];
	char ssid[32];
	int ssidlen, ret;
	
	ret = wiced_wifi_get_ap_name(ssid, &ssidlen, bssid);
	if (ret == WICED_SUCCESS) {
		memcpy(connected_ap_bssid, bssid, 6);
	}
}



static int number_of_probes_per_channel=5, ms_per_channel=110;

void set_scan_param(int number, int ms)
{
	number_of_probes_per_channel = number;
	ms_per_channel = ms;
}

/* start scan and wait scan report. */
static int start_scan(uint8_t *ssid) 
{
    int ret;
    //uint16_t chlist[] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,0 };
    wiced_scan_extended_params_t extparam;
    wiced_ssid_t optional_ssid, *pssid;
    
    system_debug_printf(SYSTEM_DEBUG_DEBUG, "start scan\r\n");

	extparam.number_of_probes_per_channel = number_of_probes_per_channel;
	extparam.scan_active_dwell_time_per_channel_ms = ms_per_channel;
	extparam.scan_passive_dwell_time_per_channel_ms = ms_per_channel;
	extparam.scan_home_channel_dwell_time_between_channels_ms = 50;
    if ((ssid!=NULL) && (ssid[0]!=0)) {
        optional_ssid.len = strlen(ssid);
        memcpy(optional_ssid.val, ssid, optional_ssid.len);
        pssid = &optional_ssid;
    } else {
        pssid = NULL;
    }
    
    free_ap_list();
    ret = wiced_wifi_scan(WICED_SCAN_TYPE_ACTIVE, WICED_BSS_TYPE_ANY, 
                          pssid, NULL, NULL, &extparam, scan_results_handler, 
                          (wiced_scan_result_t**)&p_scan_buff, NULL);

    mico_rtos_get_semaphore(&scan_sem, STA_MODE_MAX_SCANTIME);
    system_debug_printf(SYSTEM_DEBUG_DEBUG, "state %d\r\n", sta_state);
    return ret;
}

void wlan_get_tx_power(int *min, int *max, int *cur)
{
    uint8_t dbm;
    
    wiced_wifi_get_tx_power(&dbm);

    *cur = dbm;
}
int wlan_set_tx_power(int level)
{
    return wiced_wifi_set_tx_power(level);
}

void wlan_get_mac_address(uint8_t *mac)
{
    wiced_get_mac_addr(mac);
}

static void* sta_link_events_handler( const wiced_event_header_t* event_header, const uint8_t* event_data, void* handler_user_data )
{
    if ( event_header->interface != (uint8_t) WICED_STA_INTERFACE )
    {
        return handler_user_data;
    }

    system_debug_printf(SYSTEM_DEBUG_DEBUG, "event %d, status %d, flags %x\r\n",
            event_header->event_type, event_header->status, event_header->flags);
    switch ( event_header->event_type )
    {
        case WLC_E_LINK:
            if ( ( event_header->flags & WLC_EVENT_MSG_LINK ) != 0 )
            {
                sta_link_up();
            }
            else
            {
                if (sta_state == CONNECTED) {
                    sta_down( );
                    sta_state = SLEEPING;
                    mico_rtos_set_semaphore(&wifimgr_sem);
                }
            }
            break;
        case WLC_E_ROAM:
            /* when roam attempt completed successfully, we will renew the existing link */
            /* otherwise ignore all roam events */
            /* please keep in mind that if roaming was successful wlan chip wont send any link down event to the host */
            /* driver */
            if ( event_header->status == WLC_E_STATUS_SUCCESS )
            {
#ifdef FAST_ROAM
				SetTimer(10, update_connected_ap_info);
#else
                wiced_network_link_renew_handler();
#endif
            } else if ( event_header->status == WLC_E_STATUS_NO_NETWORKS ) {

            }
            break;
        default:
            wiced_assert( "Received event which was not registered\r\n", 0 != 0 );
            break;
    }
    return handler_user_data;
}
#if 0
static void timer_thread_func(uint32_t arg)
{
    UNUSED_PARAMETER(arg);
    while(1) {
        mico_rtos_get_semaphore(&timer_sem, timer_thread_wait);
        mxchip_timer_tick();
    }
}
#endif
void mxchip_thread_init(void)
{
    mico_rtos_init_semaphore(&timer_sem, 1);
    mico_rtos_init_semaphore(&wifimgr_sem, 1);
    mico_rtos_init_semaphore(&scan_sem, 1);
#if 0
    mico_rtos_create_thread(NULL, WICED_DEFAULT_WORKER_PRIORITY, "mxchipTimer", timer_thread_func, 400, 0);
#endif
	if (health_monitor_enabled == 1) {
		mico_rtos_create_thread(NULL, WICED_DEFAULT_WORKER_PRIORITY, "HealthMon", health_monitor_thread, 1024, 0);
    }
	mico_rtos_create_thread(NULL, WICED_DEFAULT_WORKER_PRIORITY, "WifiMgr", wifimgr_thread, 2*1024, 0);
}

void mxchip_system_configure(struct mxchip_config_st config)
{
	if (config.need_health_monitor == 0)
		health_monitor_enabled = 0;
}

void wlan_init()
{
    memset(active_scan_ssid, 0, sizeof(active_scan_ssid));
    g_ap_entry.valid = 0;
    wiced_management_set_event_handler( link_events, sta_link_events_handler, 0 );
}

void wifi_reboot_only(void)
{
    system_debug_printf(SYSTEM_DEBUG_DEBUG, "wifi reboot\r\n");
    wiced_management_wifi_off( );
    wiced_management_wifi_on( );
    msleep(100);// wait 100 ms.
    wlan_init();
}

static void wifi_reboot_do(void)
{
    wifi_reboot_event(WIFI_ERROR_INIT);
}

void wifi_reboot(void)
{
    SetTimer(0, wifi_reboot_do);
}
#if 0
int SetTimer(unsigned long ms, void (*psysTimerHandler)(void))
{
	struct mxchip_timer *timer, *p;

	timer = malloc(sizeof(*timer));
	if (timer == NULL)
		return -1;

	timer->timeout = host_rtos_get_time() + ms;
	timer->handler = psysTimerHandler;
    timer->valid = 1;
	timer->next = NULL;
    if (timer_head == NULL)
        timer_head = timer;
    else {
        p = timer_head;
        while(p->next != NULL)
            p = p->next;
        p->next = timer;
    }
    mico_rtos_set_semaphore(&timer_sem);
    return 0;
}

/* find in the timer list, if find the same handler iqnore, else create new timer */
int SetTimer_uniq(unsigned long ms, void (*psysTimerHandler)(void))
{
	struct mxchip_timer *p;

    p = timer_head;
    while(p != NULL) {
        if (p->handler == psysTimerHandler) {
            p->timeout = host_rtos_get_time() + ms; // update time
            p->valid = 1;
            return 0;
        } else
            p = p->next;
    }
    
	return SetTimer(ms, psysTimerHandler);
}

/* Remove all timers which handler is psysTimerHandler */
int UnSetTimer(void (*psysTimerHandler)(void))
{
	struct mxchip_timer *p;

    p = timer_head;
    while (p != NULL) {
        if (p->handler == psysTimerHandler) {
            p->valid = 0;
		} 
        p = p->next;
    }
    
    return 0;
}

static void mxchip_timer_tick(void)
{
	struct mxchip_timer *p, *q;
    uint32_t next_time = NEVER_TIMEOUT, cur_time;
    
    q = timer_head;
    p = timer_head;
	while (p != NULL) {
        if (next_time > p->timeout)
            next_time = p->timeout;
		if (p->timeout < host_rtos_get_time()) {
            if (p == timer_head) {
                timer_head = timer_head->next;
                if (p->valid == 1)
                    mico_rtos_send_asynchronous_event( WICED_NETWORKING_WORKER_THREAD, (event_handler_t) p->handler, (void*) 0 );
                free(p);
                p = timer_head; // time_head may be changed by handler().
                continue;
            } else {
                q->next = p->next;
                if (p->valid == 1)
                    mico_rtos_send_asynchronous_event( WICED_NETWORKING_WORKER_THREAD, (event_handler_t) p->handler, (void*) 0 );
    			free(p);
                break;
            }
		}
        q = p;
		p = p->next;
	}

    cur_time = host_rtos_get_time();
    if (next_time <= cur_time)
        timer_thread_wait = 1;
    else
        timer_thread_wait = next_time - cur_time;
	return;

}
#endif
static void wifimgr_thread(uint32_t arg)
{
    uint32_t timeout = NEVER_TIMEOUT;
    int scan_state;
    
    UNUSED_PARAMETER(arg);
    
    while(1) {
        scan_state = 0;
        mico_rtos_get_semaphore(&wifimgr_sem, timeout);
		if (sta_state == ADVTRY) {
			if (try_adv_connect() != 0) { // join fail, retry later.
                sta_state = SLEEPING;
            } else {
                timeout = NEVER_TIMEOUT;
                sta_state = CONNECTED;
                set_sta_connection(1);
				continue;
            }
		}
        if (user_stop_sta == 1) { // user request to disconnect AP
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "wifimgr stop sta\r\n");
            sta_state = STOPPED;
            g_ap_entry.valid = 0;
            wiced_wifi_leave();
            sta_down();
			user_stop_sta = 0;
            continue;
        }
        if (sta_state == SLEEPING)
            scan_state |= 1;
        if (user_req_scan & USER_SCAN_FLAG)
            scan_state |= 2;
        if (user_req_scan & USER_ADV_SCAN_FLAG)
            scan_state |= 4;
        if (user_req_scan & CMD_SCAN_FLAG)
            scan_state |= 8;
        
        if (scan_state == 0)
            continue;
        
        user_req_scan = 0;
        if (scan_state > 1) {
            if (strlen(active_scan_ssid) > 0) {
                start_scan(active_scan_ssid);
                memset(active_scan_ssid, 0, sizeof(active_scan_ssid));
            } else
                start_scan(NULL);
        } else 
            start_scan(g_ap_entry.ssid);
        
        if (scan_state & 2) {
            get_scan_result_list(&gApList);
            scan_finished();
        } 
        
        if (scan_state & 4) {
            get_scan_result_adv_list(&gApAdvList);
            mico_rtos_delay_milliseconds(10);
            scan_adv_finished();
        } 

        system_debug_printf(SYSTEM_DEBUG_DEBUG, "state %d\r\n", sta_state);
        if (sta_state == SLEEPING) {
            sta_state = JOINING;
            if (select_ap_to_join() != WICED_SUCCESS) { // join fail, retry later.
                timeout = sta_retry_interval;
                sta_state = SLEEPING;
            } else {
                timeout = NEVER_TIMEOUT;
                sta_state = CONNECTED;
            }
        }
    }
}


static wiced_time_t last_no_bus_credit_time = 0;

void update_bus_credit_time(wiced_bool_t has_bus_credit)
{
    if (has_bus_credit == WICED_TRUE)
        last_no_bus_credit_time = 0;
    else if (last_no_bus_credit_time == 0)
        last_no_bus_credit_time = host_rtos_get_time();
}

extern void os_tick_(void);

static void health_monitor_thread(uint32_t arg)
{
    int sleep_time = 10;
    int fail_times = 0;
    uint32_t ip,gw,mask;
    ip_addr_t gwip, retgw;
    struct eth_addr *eth;
    char ssid[33], bssid[6];
    int len, ret;
    wiced_time_t curtime;
    UNUSED_PARAMETER(arg);

#define MAX_RETRY_TIMES 10

    while(1) {
        sleep(sleep_time);
		os_tick_();
        if (last_no_bus_credit_time != 0) {
            curtime = host_rtos_get_time();
            if (curtime > last_no_bus_credit_time + 60*1000) {// no bus credit more than 1 minute.
                wifi_reboot_event(WIFI_ERROR_NOBUS_CREDIT);
                system_debug_printf(SYSTEM_DEBUG_DEBUG, "healMon reboot, no bus credit time too long\r\n");
            }
        }

        if (sta_connected == 0) {
            sleep_time = 60;
            fail_times = 0;
            continue;
        }
        
        wiced_ip_get_ipaddr(WICED_STA_INTERFACE, &ip, &mask, &gw);
        gwip.addr = htonl(gw);
        if (etharp_find_addr(&IP_HANDLE(WICED_STA_INTERFACE), &gwip, &eth, &retgw) < 0) {
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "healMon query gateway %x %d\r\n", gwip.addr, fail_times);
            etharp_query(&IP_HANDLE(WICED_STA_INTERFACE), &gwip, NULL);
            sleep_time = 10;
            fail_times++;
            if (fail_times >= MAX_RETRY_TIMES) {
                wifi_reboot_event(WIFI_ERROR_NOGW);
                memset(ssid, 0, sizeof(ssid));
                ret = wiced_wifi_get_ap_name(ssid, &len, bssid);
                system_debug_printf(SYSTEM_DEBUG_DEBUG, "healMon reboot, get AP info ret %d\r\n", ret);
                system_debug_printf(SYSTEM_DEBUG_DEBUG, "AP Info: ssid %s, bssid %02x-%02x-%02x-%02x-%02x-%02x\r\n",
                        ssid, bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
                fail_times = 0;
            } 
        } else {
            sleep_time = 60;
            fail_times = 0;
        }
    }
}

int get_gateway_mac(char mac[6])
{
    uint32_t ip,gw,mask;
    ip_addr_t gwip, retgw;
    struct eth_addr *eth;

    memset(mac, 0, 6);
    wiced_ip_get_ipaddr(WICED_STA_INTERFACE, &ip, &mask, &gw);
    gwip.addr = htonl(gw);
    if (etharp_find_addr(&IP_HANDLE(WICED_STA_INTERFACE), &gwip, &eth, &retgw) < 0) {
        etharp_query(&IP_HANDLE(WICED_STA_INTERFACE), &gwip, NULL);
        msleep(100);
        if (etharp_find_addr(&IP_HANDLE(WICED_STA_INTERFACE), &gwip, &eth, &retgw) < 0)
            return 0;
    } 

    memcpy(mac, eth, 6);
    return 1;
}

int station_dhcp_mode (void)
{
    return sta_dhcp_mode;
}

void get_easylink_channel_list(inert_ap_list_t insert)
{
    int ret;
    //uint16_t chlist[] = { 1,2,3,4,5,6,7,8,9,10,11,12,13};
    wiced_scan_extended_params_t extparam = { 5, 120, 120, 10 };
    wiced_scan_result_t* p;
	
    stopscan();
    user_req_scan = EASYLINK_SCAN_FLAG;
    ret = wiced_wifi_scan(WICED_SCAN_TYPE_ACTIVE, WICED_BSS_TYPE_INFRASTRUCTURE, 
                          NULL, NULL, NULL, &extparam, scan_results_handler, 
                          (wiced_scan_result_t**)&p_scan_buff, NULL);

    if (ret != WICED_SUCCESS) {
        int i;
        
        system_debug_printf(SYSTEM_DEBUG_DEBUG, "wifi scan ret %d\r\n", ret);
        for(i=1;i<14;i++)
            insert("", NULL, i, 0);
        return;
    }
    ret = mico_rtos_get_semaphore(&scan_sem, 5000);
    if (p_result_head == NULL) {
        int i;
        
        system_debug_printf(SYSTEM_DEBUG_DEBUG, "get scan semphore ret %d\r\n", ret);
        for(i=1;i<14;i++)
            insert("", NULL, i, 0);
        return;
    } else {
		p = p_result_head;
	    while(p != NULL) {
	        insert((char*)p->SSID.val, p->BSSID.octet, p->channel, (int)p->signal_strength);
	        p = p->next;
	    }
	}
}

static void send_minus_cb( wiced_scan_result_t ** result_ptr, void * user_data )
{
    return;
}

int mico_send_minus(char *ssid, int len)
{
	int ret;
    uint16_t chlist[] = { 6, 0};
    wiced_scan_extended_params_t extparam = { 5, 20, 20, 1 };
	wiced_ssid_t optional_ssid;
	uint32_t channel;
	
	if (wiced_wifi_get_channel(&channel) == WICED_SUCCESS) {
		chlist[0] = channel;
	}
	optional_ssid.len = len;
	memcpy(optional_ssid.val, ssid, 32);
    ret = wiced_wifi_scan(WICED_SCAN_TYPE_ACTIVE, WICED_BSS_TYPE_INFRASTRUCTURE, 
                          &optional_ssid, NULL, chlist, &extparam, send_minus_cb, 
                          (wiced_scan_result_t**)&p_scan_buff, NULL);

	msleep(20);

	return ret;
}


#ifdef FAST_ROAM
int Get_connected_ap_info(int *dBm, uint8_t bssid[6])
{
    if (sta_connected == 0)
		return 0;
	
    wiced_wifi_get_rssi(dBm);
    memcpy(bssid, connected_ap_bssid, 6);
	
    return 0;
}
#endif
/*-----------------------------------------------------------*/
void wifistate_Command( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv )
{
    char ssid[33], bssid[6];
    int len, dBm, channel;

    if (g_ap_entry.valid)
        cmd_printf("Connect AP info: \r\n\tSSID: %s\r\n\tkey: %s\r\n\tBSSID: %02x-%02x-%02x-%02x-%02x-%02x\r\n\tretry interval %d\r\n\r\n", 
        g_ap_entry.ssid, g_ap_entry.key, g_ap_entry.bssid[0],g_ap_entry.bssid[1],
        g_ap_entry.bssid[2],g_ap_entry.bssid[3],g_ap_entry.bssid[4],
        g_ap_entry.bssid[5], sta_retry_interval);
    switch(sta_state) {
    case STOPPED:
        cmd_printf("Station is STOPPED\r\n");
        break;
    case SLEEPING:
        cmd_printf("Station is sleeping\r\n");
        break;
    case SCANING:
        cmd_printf("Station is scanning for AP\r\n");
        break;
    case JOINING:
        cmd_printf("Station is trying to join AP\r\n");
        break;
    case CONNECTED:
        cmd_printf("Station is CONNECTED\r\n");
        wiced_wifi_get_rssi((int32_t*)&dBm);
        wiced_wifi_get_ap_name(ssid, &len, bssid);
        wiced_wifi_get_channel((uint32_t*)&(channel));
        cmd_printf("Connected AP info: \r\n\tSSID: %s\r\n\tChannel: %d \r\n\tRSSI: %d\r\n", ssid, channel, dBm);
        cmd_printf("\tBSSID: %02x-%02x-%02x-%02x-%02x-%02x\r\n", bssid[0],
            bssid[1],bssid[2],bssid[3],bssid[4],bssid[5]);
        break;
    default:
        cmd_printf("Unknown WIFI state %d\r\n", sta_state);
        break;
    }
}
void wifidebug_Command( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv )
{
	if (!strcasecmp(argv[1], "on")) {
        wifimgr_debug_enable(1);
		cmd_printf("Enable wifidebug\r\n");
	} else if (!strcasecmp(argv[1], "off")) {
    	wifimgr_debug_enable(0);
		cmd_printf("Disable wifidebug\r\n");
	}
}

void wifiscan_Command( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv )
{
    wiced_scan_result_t *p;
    int i = 0;
    
    cmd_printf("Waiting for scan results...\r\n");
    user_req_scan |= CMD_SCAN_FLAG;
    
    mico_rtos_set_semaphore(&wifimgr_sem);

    mico_rtos_get_semaphore(&scan_sem, 2*1000); // wait 2 seconds
    p = p_result_head;
    if (p != NULL) {
        cmd_printf("  # Type  BSSID             RSSI  Chan Security    SSID\r\n");
        cmd_printf("------------------------------------------------------------\r\n");
    } else
        cmd_printf("Scan Failed\r\n");
    while(p != NULL) {
        cmd_printf( "%3d %5s ", i++, ( p->bss_type == WICED_BSS_TYPE_ADHOC ) ? "Adhoc" : "Infra" );
        cmd_printf( "%02X:%02X:%02X:%02X:%02X:%02X ", p->BSSID.octet[0], p->BSSID.octet[1], p->BSSID.octet[2], p->BSSID.octet[3], p->BSSID.octet[4], p->BSSID.octet[5] );
        cmd_printf( " %d ", p->signal_strength );
        cmd_printf( " %2d  ", p->channel );
        cmd_printf( "%-10s ", ( p->security == WICED_SECURITY_OPEN ) ? "Open" :
                                     ( p->security == WICED_SECURITY_WEP_PSK ) ? "WEP" :
                                     ( p->security == WICED_SECURITY_WPA_TKIP_PSK ) ? "WPA TKIP" :
                                     ( p->security == WICED_SECURITY_WPA_AES_PSK ) ? "WPA AES" :
                                     ( p->security == WICED_SECURITY_WPA2_AES_PSK ) ? "WPA2 AES" :
                                     ( p->security == WICED_SECURITY_WPA2_TKIP_PSK ) ? "WPA2 TKIP" :
                                     ( p->security == WICED_SECURITY_WPA2_MIXED_PSK ) ? "WPA2 Mixed" :
                                     "Unknown" );
        cmd_printf( " %-32s ", p->SSID.val );
        cmd_printf( "\r\n" );
        p = p->next;
    }
}

ip_addr_t dns_getserver(u8_t numdns);

void ifconfig_Command( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv )
{
    uint32_t ip, netmask, gateway, dnsServer;
    char *p;
    ip_addr_t addr;
    
    if (sta_connected) {
        wiced_ip_get_ipaddr(WICED_STA_INTERFACE, &ip, &netmask, &gateway);
        addr.addr = htonl(ip);
        p = ipaddr_ntoa(&addr);
        cmd_printf("Station Interface: \r\n    IP Addr: %s\r\n", p);
    	addr.addr = htonl(netmask);
        p = ipaddr_ntoa(&addr);
        cmd_printf("    Netmask: %s\r\n", p);
    	addr.addr = htonl(gateway);
        p = ipaddr_ntoa(&addr);
        cmd_printf("    Gateway: %s\r\n", p);
        if ((addr.addr = dns_getserver(0).addr)) {
            p = ipaddr_ntoa(&addr);
            cmd_printf("    DNS1   : %s\r\n", p);
        }
        if ((addr.addr = dns_getserver(1).addr)) {
            p = ipaddr_ntoa(&addr);
            cmd_printf("    DNS2   : %s\r\n", p);
        }
    } 

    if (uap_connected) {
        wiced_ip_get_ipaddr(WICED_AP_INTERFACE, &ip, &netmask, &gateway);
        addr.addr = htonl(ip);
        p = ipaddr_ntoa(&addr);
        cmd_printf("SoftAP Interface: \r\n    IP address: %s\r\n", p);
    	addr.addr = htonl(netmask);
        p = ipaddr_ntoa(&addr);
        cmd_printf("    Netmask: %s\r\n", p);
    }
}

void arp_display( char *pcWriteBuffer, int xWriteBufferLen);
void arp_clean(void);
void arp_Command( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv )
{
    if (argc != 2) {
        cmd_printf("Usage: arp show/clean.\r\n");
        return;
    }

	if (!strcasecmp(argv[1], "show")) {
		arp_display(pcWriteBuffer, xWriteBufferLen);
	} else if (!strcasecmp(argv[1], "clean")) {
		arp_clean();
        cmd_printf("ARP Clean Done\r\n");
	} else {
        cmd_printf("Usage: arp show/clean.\r\n");
    }
}

void ping_Command( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv )
{
    wiced_ip_address_t  ping_target_ip;
    uint32_t ip;
    wiced_result_t status;
    uint32_t elapsed_ms;
    int i;
    
    if (argc != 2) {
        cmd_printf("Usage: ping <IP>.\r\n");
        return;
    }

    ip = ipaddr_addr(argv[1]);
    ping_target_ip.version = WICED_IPV4;
    ping_target_ip.ip.v4 = htonl(ip);

    for(i=0;i<5;i++) {
        if (i > 0)
            msleep(1000);
        status = wiced_ping( WICED_STA_INTERFACE, &ping_target_ip, 900, &elapsed_ms );
        if ( status == WICED_SUCCESS )
        {
            cmd_printf("ping reply from %s, delay %ld ms\r\n", argv[1], (long)elapsed_ms);
        }
        else if ( status == WICED_TIMEOUT )
        {
            cmd_printf("ping timeout\r\n" );
        }
        else
        {
            cmd_printf("ping error\r\n" );
        }
    }
}

void dns_show(char *pcWriteBuffer, int xWriteBufferLen);
void dns_clean(void);
//dns show/get/clean
void dns_Command( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv )
{
    char ipaddr[32];
    err_t err;
	ip_addr_t addr;
	
    if (argc != 2) {
        cmd_printf("Usage: dns show/clean/<domain_name>.\r\n");
        return;
    }

    if (!strcasecmp(argv[1], "show")) {
		dns_show(pcWriteBuffer, xWriteBufferLen);
	} else if (!strcasecmp(argv[1], "clean")) {
		dns_clean();
        cmd_printf("DNS Clean Done\r\n");
	} else {
		err = netconn_gethostbyname(argv[1], &addr);
		strcpy(ipaddr, ipaddr_ntoa((ip_addr_t*)&(addr)));
	    if (ERR_OK == err)
            cmd_printf("%s's IP address is %s\r\n", argv[1], ipaddr);
        else
            cmd_printf("Can't get %s's IP address\r\n", argv[1]);
    }
}

void task_Command( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv )
{
    cmd_printf("%-32s Status Prio    Stack   TCB\r\n", "Name");
    cmd_printf("---------------------------------------------------------------");
    if (xWriteBufferLen >= 256)
        vTaskList(pcWriteBuffer);
    else {
        char buf[256];
        
        vTaskList(buf);
        strncpy(pcWriteBuffer, buf, xWriteBufferLen);
    }
}

void tcp_dump(char *pcWriteBuffer, int xWriteBufferLen);
void udp_dump(char *pcWriteBuffer, int xWriteBufferLen);
void socket_show(char *pcWriteBuffer, int xWriteBufferLen);
void socket_show_Command( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv )
{
    socket_show(pcWriteBuffer, xWriteBufferLen);
    xWriteBufferLen -= strlen(pcWriteBuffer);
    pcWriteBuffer += strlen(pcWriteBuffer);
    
    tcp_dump(pcWriteBuffer, xWriteBufferLen);
    
    xWriteBufferLen -= strlen(pcWriteBuffer);
    pcWriteBuffer += strlen(pcWriteBuffer);
    udp_dump(pcWriteBuffer, xWriteBufferLen);
}

struct _mallinfo {
  int num_of_chunks;  /* number of free chunks */
  int total_memory;  /* maximum total allocated space */
  int allocted_memory; /* total allocated space */
  int free_memory; /* total free space */
};
struct _mallinfo *mico_memory_info();


void memory_show_Command( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv )
{
    struct _mallinfo *p;

    p = mico_memory_info();
    cmd_printf("number of chunks %d\r\n", p->num_of_chunks);
    cmd_printf("total memory %d\r\n", p->total_memory);
    cmd_printf("free memory %d\r\n", p->free_memory);
    //p_bufstr = pcWriteBuffer;
    //buflen = xWriteBufferLen;
    //cmd_printf("malloc list: \r\n");
    //get_malloc_list(debug_mem);
    //xWriteBufferLen = buflen;
    //pcWriteBuffer = p_bufstr;
    //cmd_printf("free list: \r\n");
    //get_free_list(debug_mem);
}

void memory_dump_Command( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv )
{
    uint32_t i;
    uint8_t *pstart;
    uint32_t start, length;
    
    if (argc != 3) {
        cmd_printf("Usage: memdump <addr> <length>.\r\n");
        return;
    }

    start = strtoul(argv[1], NULL, 0);
    length = strtoul(argv[2], NULL, 0);
    pstart = (uint8_t*)start;
    
    for(i=0; i<length;i++) {
        cmd_printf("%02x ", pstart[i]);
        if (i % 0x10 == 0x0F) {
            cmd_printf("\r\n");
            
        }
    }
}

void memory_set_Command( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv )
{
    uint8_t *pstart, value;
    uint32_t start;
    int i;
    
    if (argc < 3) {
        cmd_printf("Usage: memset <addr> <value 1> [<value 2> ... <value n>].\r\n");
        return;
    }
    start = strtoul(argv[1], NULL, 0);
    value = strtoul(argv[2], NULL, 0);
    pstart = (uint8_t*)start;
    *pstart = value;
    cmd_printf("Set 0x%08x %d bytes\r\n", start, argc-2);
    for(i=2;i<argc;i++) {
        value = strtoul(argv[i], NULL, 0);
        pstart[i-2] = value;
    }
}


int mico_wlan_custom_ie_add(wlan_if_t wlan_if, uint8_t *custom_ie, uint32_t len)
{
    uint8_t ret = wiced_wifi_manage_custom_ie(  wlan_if == Soft_AP ? WICED_AP_INTERFACE : WICED_STA_INTERFACE,
                                                WICED_ADD_CUSTOM_IE, 
                                                custom_ie, 
                                                custom_ie[3], 
                                                &custom_ie[4], 
                                                len - 4, 
                                                VENDOR_IE_BEACON|VENDOR_IE_PROBE_RESPONSE);
    return ret == WICED_SUCCESS ? kNoErr : kGeneralErr;
}

int mico_wlan_custom_ie_delete(wlan_if_t wlan_if, uint8_t op, uint8_t *option_data, uint32_t len)
{
    if(op != 1)
    {
        return kUnsupportedErr;
    }

    uint8_t *custom_ie = option_data;
    uint8_t ret = wiced_wifi_manage_custom_ie(  wlan_if == Soft_AP ? WICED_AP_INTERFACE : WICED_STA_INTERFACE,
                                            WICED_REMOVE_CUSTOM_IE, 
                                            custom_ie, 
                                            custom_ie[3], 
                                            &custom_ie[4], 
                                            len - 4, 
                                            VENDOR_IE_BEACON|VENDOR_IE_PROBE_RESPONSE);
    return ret == WICED_SUCCESS ? kNoErr : kGeneralErr;
}

int dns_client_add_server_address( wiced_ip_address_t address )
{
	ip_addr_t dns_addr;
	
    ip4_addr_set_u32(&dns_addr, htonl(address.ip.v4));
    dns_setserver(0, &dns_addr);
    return 0;
}

int dns_client_remove_all_server_addresses( void )
{
    dns_setserver(0, NULL);
	dns_setserver(1, NULL);
    return WICED_SUCCESS;
}
