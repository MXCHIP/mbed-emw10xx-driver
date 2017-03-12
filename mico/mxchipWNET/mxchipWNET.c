
#include "wiced_management.h"
#include "wwd_management.h"
#include "wifimgr.h"
#include "smart_config.h"
#include "wiced_wifi.h"
#include "mxchipWNET.h"

#include "mico_rtos.h"
#include "lwip/err.h"

#include "lwip/ip_addr.h"
#include "lwip/dns.h"


#define IPPROTO_IP      0


#if defined MXCHIP_3166
#define LIBRARY_VER "31660002.050"

#define HC128_CHECK
#elif defined MXCHIP_3238
#define LIBRARY_VER "32380002.050"
#define _check_os_d(x) 

#elif defined MXCHIP_3239
#define LIBRARY_VER "32390002.050"
#define HC128_CHECK

#else
#define _check_os_d(x) 

#ifdef SMALL_MEMP
#define LIBRARY_VER "31620002.035-smallmemp"
#elif defined (TCP_RX_PBUF_COPY)
#define LIBRARY_VER "31621002.050"
#else
#define LIBRARY_VER "31620002.049"
#endif
#endif

enum {
    WIFI_CHIP_ID_43362 = 1,
    WIFI_CHIP_ID_43341,
    WIFI_CHIP_ID_43438,

};

static int ps_enabled = 0;
static int wifi_power_is_down = 0;

static int mxchip_inited = 0;

extern int errno;

extern int station_dhcp_mode (void);

char* system_lib_version(void)
{
    return LIBRARY_VER;
}

WEAK void system_version(char *str, int maxlen)
{
    snprintf(str, maxlen, "Unknown");
}

WEAK void dma_rx_complete_handler(void)
{
}

WEAK void dma_tx_complete_handler(void)
{
}

/* User provide callback functions:  */
WEAK void WatchDog(void)
{
}
WEAK void WifiStatusHandler(int status)
{
}

WEAK void RptConfigmodeRslt(network_InitTypeDef_st *nwkpara)
{
}

WEAK void NetCallback(net_para_st *pnet)
{
}

WEAK void ApListCallback(ScanResult *pApList)
{
}

WEAK void ApListAdvCallback(ScanResult_adv *pApAdvList)
{
}


WEAK void socket_connected(int fd)
{

}

WEAK void easylink_user_data_result(int datalen, char*data)
{

}

WEAK void connected_ap_info(apinfo_adv_t *ap_info, char *key, int key_len)
{
}

WEAK void wifi_reboot_event(int type)
{
}

WEAK void join_fail(int type)
{
}

WEAK void mico_rtos_stack_overflow(char *taskname)
{
    printf("Stack Overflow Detected in task %s\r\n",taskname);
}

WEAK void mac_report_cb(uint8_t *mac, int rssi)
{
    //printf("mac %02x-%02x-%02x-%02x-%02x-%02x\r\n", mac[0],
    //    mac[1],mac[2],mac[3],mac[4],mac[5]);
}

WEAK int alink_get_tpsk(uint8_t *ie,  char *tpsk)
{
    return -1;
}

WEAK void tcp_input_cb(void)
{
}

WEAK void mxchip_mfg_test(void)
{
}

/* type = 0: fine
  * type = 1: stuck
  */
WEAK void wifi_tx_state(int type)
{
}

WEAK void easylink_wifi_info(uint8_t*bssid, int rssi)
{
}

WEAK void probe_request_rx_cb(uint8_t *header, int length)
{

}


#ifdef HC128_CHECK
void hc128_crypto(uint8_t *input, int msglen, uint8_t *output, uint8_t *key, uint8_t *iv);


static int _os_d = 0;
#define P_VER   2
#define PLAIN_HEAD "MiCO"

#define OTP_START_ADDRESS 0x1FFF7800
#define BLOCK_12_OFFSET   0x140


static char ali_key[20], ali_secret[16];

/*
 * Get the unique ID of STM32F412 Devices
 */
void cpuidGetId(uint8_t * mcuID)
{
    int i=0;
    for(i=0;i<12;i++)
    {
        mcuID[i] = (*(volatile char*)(0x1FFF7A10+i));
    }

}

typedef unsigned int   word32;

enum {
  MD5             =  0,      /** hash type unique */
  MD5_BLOCK_SIZE  = 64,
  MD5_DIGEST_SIZE = 16,
  MD5_PAD_SIZE    = 56
};

/* MD5 digest */
typedef struct Md5 {
    word32  buffLen;   /* in bytes          */
    word32  loLen;     /* length in bytes   */
    word32  hiLen;     /* length in bytes   */
    word32  digest[MD5_DIGEST_SIZE / sizeof(word32)];
    word32  buffer[MD5_BLOCK_SIZE  / sizeof(word32)];
} Md5;

static void md5_hex(unsigned char *input, int len, unsigned char *output)
{
    Md5 ctx;

    InitMd5(&ctx);
    Md5Update(&ctx, (const unsigned char*)input, len);
    Md5Final(&ctx, (unsigned char*)output);
}

static inline int check_id(void)
{
	/*60 bytes otp:
	  * 12 bytes: F412 chipid
	  * 16 bytes: mxchip chipid
	  * 32 bytes: ali key&sec
	  */
	uint8_t OTP_plain[60]={0};

	uint8_t key[16], iv[16], tmp[12];
	uint8_t buf_chipID[16]={0};
	uint8_t *p_otp = (uint8_t*)(OTP_START_ADDRESS + BLOCK_12_OFFSET);

	memset(buf_chipID, 0, sizeof(buf_chipID));
	cpuidGetId(buf_chipID);

	hc128_crypto( p_otp, 60, OTP_plain, buf_chipID, buf_chipID );

	ali_key[0]='0';
	memcpy(&ali_key[1], &OTP_plain[28], 16);
	memcpy(ali_secret, &OTP_plain[44], 16);

	_os_d = 1;
	memset(iv, 0, sizeof(iv));
	memcpy(iv, &OTP_plain[12], 4);
	memcpy(&iv[4], PLAIN_HEAD, 4);
	md5_hex(iv, 8, key);

	hc128_crypto(&OTP_plain[16], 12, tmp, key, iv);

	if (memcmp(tmp, "MiCO", 4) != 0) {
		return 0;
	}
	if (memcmp(&tmp[8], &tmp[4], 4) != 0)  {
		return 0;
	}

	_os_d = 0;
	return 1;
}

#define _check_os_d(x) do {if (_os_d)malloc(x+100);} while(0)

char *get_ali_key(void)
{
	return ali_key;
}

char *get_ali_secret(void)
{
	return ali_secret;
}

#endif



OSStatus mxchipInit(void)
{
    int ret;
	uint8_t mac[6];
	
    if (mxchip_inited == 1)
        return kNoErr;
    
    ret = wiced_init();
    if (ret != kNoErr) {
        wifi_reboot_event(1); // wifi can't be initiliazed.
        return kGeneralErr;
    }
    wlan_init();
    mxchip_thread_init();
	wlan_get_mac_address(mac);
#ifdef HC128_CHECK
	check_id();
#endif	

	_check_os_d(__LINE__);
    mxchip_inited = 1;
	return kNoErr;
}

/* MXCHIP NORMAL API  END*/
//extern char *inet_ntoa (struct in_addr addr);

int getNetPara(IPStatusTypedef * pnetpara, WiFi_Interface iface)
{
	uint8_t mac[6];
	char macstr[14];
	uint32_t ip, netmask, gateway;
	int if_default;
	struct in_addr temp_addr;
	ip_addr_t dnsServer;
    
    if (wifi_power_is_down == 1)
        return kGeneralErr;
    _check_os_d(__LINE__);
	if (iface == 0)// 0=softAP
		if_default = 1;//WICED_AP_INTERFACE;
	else
		if_default = 0;//WICED_STA_INTERFACE;
	memset(pnetpara, 0, sizeof(net_para_st));
	wlan_get_mac_address(mac);

	sprintf(macstr, "%02x%02x%02x%02x%02x%02x", mac[0],
			mac[1], mac[2], mac[3], mac[4], mac[5]);
	memcpy(pnetpara->mac, macstr, 12);
	
	wiced_ip_get_ipaddr(if_default, &ip, &netmask, &gateway);
	temp_addr.s_addr = htonl(ip);
    strcpy( pnetpara->ip, inet_ntoa( temp_addr ) );
    temp_addr.s_addr = htonl(netmask);
    strcpy( pnetpara->mask, inet_ntoa( temp_addr ) );
    temp_addr.s_addr = htonl(gateway);
    strcpy( pnetpara->gate, inet_ntoa( temp_addr ) );
    pnetpara->dhcp = station_dhcp_mode(); // 
	dnsServer = dns_getserver(0);
    strcpy( pnetpara->dns, ipaddr_ntoa( &dnsServer ) );
    sprintf(pnetpara->broadcastip, "255.255.255.255");
    
	return kNoErr;
}

void station_up(void)
{
    net_para_st netpara;
    net_para_st *pnetpara = &netpara;
    uint8_t mac[6];
    char macstr[14];
    uint32_t ip, netmask, gateway;
	ip_addr_t dnsServer;
    struct in_addr temp_addr;

    if (mxchip_inited == 0)// mftest mode
        return ;
    _check_os_d(__LINE__);
	memset(pnetpara, 0, sizeof(net_para_st));
	wlan_get_mac_address(mac);

	sprintf(macstr, "%02x%02x%02x%02x%02x%02x", mac[0],
			mac[1], mac[2], mac[3], mac[4], mac[5]);
	memcpy(pnetpara->mac, macstr, 12);

    wiced_ip_get_ipaddr(0, &ip, &netmask, &gateway);
    temp_addr.s_addr = ntohl(ip);
    strcpy( pnetpara->ip, inet_ntoa( temp_addr ) );
    temp_addr.s_addr = ntohl(netmask);
    strcpy( pnetpara->mask, inet_ntoa( temp_addr ) );
    temp_addr.s_addr = ntohl(gateway);
    strcpy( pnetpara->gate, inet_ntoa( temp_addr ) );
    pnetpara->dhcp = station_dhcp_mode(); //
    dnsServer = dns_getserver(0);
    strcpy( pnetpara->dns, ipaddr_ntoa( &dnsServer ) );
    sprintf(pnetpara->broadcastip, "255.255.255.255");
    NetCallback(pnetpara);
    sta_up();
}

void wifi_status_callback(int connection)
{
	_check_os_d(__LINE__);
    if (mxchip_inited == 0)// mftest mode
        return ;
    switch(connection) {
    case WIFI_STATE_STA_UP:
        if (ps_enabled == 1)
            wiced_wifi_enable_powersave_with_throughput(100);
        WifiStatusHandler(NOTIFY_STATION_UP);
        break;
    case WIFI_STATE_STA_DOWN:
        WifiStatusHandler(NOTIFY_STATION_DOWN);
        break;
    case WIFI_STATE_UAP_UP:
        WifiStatusHandler(NOTIFY_AP_UP);
        break;
    case WIFI_STATE_UAP_DOWN:
        WifiStatusHandler(NOTIFY_AP_DOWN);
        break;
    default:
        return;
    }
}


void mxchipStartScan(void)
{
    if (wifi_power_is_down == 1)
        return;
    _check_os_d(__LINE__);
	user_scan(0);
}

void mxchipStartAdvScan(void)
{
    if (wifi_power_is_down == 1)
        return ;
    _check_os_d(__LINE__);
	user_scan(1);
}

int mxchip_active_scan(char*ssid, int is_adv)
{
    if (wifi_power_is_down == 1)
        return kGeneralErr;
    _check_os_d(__LINE__);
	user_active_scan(ssid, is_adv);
    return kNoErr;
}

int StopScan(void)
{
	_check_os_d(__LINE__);
    return kNoErr;
}

int StartScan(int interval)
{
    if (wifi_power_is_down == 1)
        return kGeneralErr;
    _check_os_d(__LINE__);
    
    if(user_scan(0) != 0)
        return kGeneralErr;
    else
        return kNoErr;
}


int SelectSupport(void)
{
    return kNoErr;
}


void enable_ps_mode(int unit_type, int unitcast_ms, int multicast_ms)
{
	_check_os_d(__LINE__);
	ps_enabled = 1;
    wiced_wifi_enable_powersave_with_throughput(100);
}

void disable_ps_mode(void)
{
	_check_os_d(__LINE__);
    wiced_wifi_disable_powersave();
	ps_enabled = 0;
}

void ps_enable(void)
{
	_check_os_d(__LINE__);
    ps_enabled = 1;
    wiced_wifi_enable_powersave_with_throughput(100);
}

void ps_disable(void)
{
	_check_os_d(__LINE__);
    disable_ps_mode();
}


// 
/* wifi_strength 0~100, rssi of current AP. */
int CheckNetLink(LinkStatusTypeDef *ap_state)
{
    int32_t dBm;
    int ssidlen;
	_check_os_d(__LINE__);
	memset(ap_state, 0, sizeof(LinkStatusTypeDef));
    if (wifi_power_is_down == 1){
        ap_state->is_connected = 0;
        return 0;
    }
    ap_state->is_connected = get_sta_connection();
    wiced_wifi_get_rssi(&dBm);
    
    // dBm to Quality:
    ap_state->signal_strength = (int16_t)dBm;

    wiced_wifi_get_ap_name( (char *)ap_state->ssid, &ssidlen, (char *)ap_state->bssid);
    wiced_wifi_get_channel((uint32_t*)&(ap_state->channel));

    return 0;
}

int is_uap_mode(void)
{
	return 0;// TODO: uap_enabled;
}

int is_ps_enabled(void)
{
	return ps_enabled;
}

void user_RptConfigmodeRslt(char *ssid, char *key)
{
    network_InitTypeDef_st nwkpara;
	_check_os_d(__LINE__);
    if (ssid == NULL) { // fail
        RptConfigmodeRslt(NULL); 
        return;
    }
    nwkpara.wifi_mode = 1;
    strncpy(nwkpara.wifi_key, key, 64);
    strncpy(nwkpara.wifi_ssid, ssid, 32);
    nwkpara.dhcpMode = 1;
    RptConfigmodeRslt(&nwkpara); 
}
void report_ap_info(char *ssid);

void smart_config_done(char *ssid, char *key, int mode)
{
    network_InitTypeDef_st nwkpara;
	_check_os_d(__LINE__);
    if (ssid == NULL) { // fail
        RptConfigmodeRslt(NULL); 
        return;
    }
    nwkpara.wifi_mode = 1;
    strncpy(nwkpara.wifi_key, key, 64);
    strncpy(nwkpara.wifi_ssid, ssid, 32);
    nwkpara.dhcpMode = 1;
    nwkpara.wifi_retry_interval = mode; 
    RptConfigmodeRslt(&nwkpara); 
	report_ap_info(ssid);
}

void easy_link_success_callback(char *ssid, char *key)
{
    user_RptConfigmodeRslt(ssid, key);
	_check_os_d(__LINE__);
}

int OpenEasylink(int timeout)
{
    if (wifi_power_is_down == 1)
        return kGeneralErr;
    
    wlan_disconnect();
	_check_os_d(__LINE__);
    mxchip_easy_link_start(timeout);
    return 0;
}

int CloseEasylink(void)
{
    easy_link_stop();
    return 0;
}

int OpenAlink(int timeout)
{
    if (wifi_power_is_down == 1)
        return kGeneralErr;
    
    wlan_disconnect();
    alink_config_start(timeout);
    return 0;
}

int CloseAlink(void)
{
    alink_config_stop();
    return 0;
}

int OpenEasylink2(int timeout)
{
    if (wifi_power_is_down == 1)
        return kGeneralErr;
    
    wlan_disconnect();
    easylink2_start(0, timeout);
    return 0;
}

int CloseEasylink2(void)
{
    easylink2_stop();
    return 0;
}

int OpenEasylink2_withdata(int timeout)
{
    if (wifi_power_is_down == 1)
        return kGeneralErr;
    
    wlan_disconnect();
    easylink2_start( 1, timeout );
    return 0;
}


int OpenAirkiss(int timeout)
{
    airkiss_start(timeout);
    return 0;
}

int CloseAirkiss(void)
{
    Airkiss_stop();
    return 0;
}

int ChgWIFIWorkMode(char flag)
{
    if (flag == 'b' || flag=='g' || flag=='n')
        return 0;// don't set work mode
    else
        return -1;
}


void scan_report(ScanResult *pApList)
{
    ApListCallback(pApList);
}

void os_tick_(void)
{
	_check_os_d(__LINE__);
}

/* AES APIs*/
#define BLOCK_SIZE 16

void Aes128_EcbEncode(uint8_t *key,  char* srcString, int srcLen, char* dstString, int dstLen)
{
    
}

void Aes128_EcbDecode(uint8_t *key,  char* srcString, int srcLen, char* dstString, int dstLen)
{
    
}

int wifi_power_down(void)
{
    if (wifi_power_is_down == 1)
        return 0;
    
    wlan_disconnect();
    msleep(100); // delay 100ms to wait IP down, may hardfault if no delay 
    wiced_management_wifi_off( );
    wifi_power_is_down = 1;
    return 0;
}

int wifi_power_up(void)
{
    if (wifi_power_is_down == 0)
        return 0;
    wifi_power_is_down = 0;
    wiced_management_wifi_on( );
    wlan_init();
    return 0;
}




#ifdef HC128_CHECK
int wlan_driver_version( char* version, uint8_t length )
{
	int ret;
	char *p;
	
	if (_os_d)
	{
		sprintf(version, "NULL");
		return 0;
	}

	ret = wiced_wifi_get_wifi_version(version, length);
	p = strstr(version, "FWID");
	if (p)
		p[0] = 0;
    return ret;
}

#else
int wlan_driver_version( char* version, uint8_t length )
{
	char *p;
	int ret = wiced_wifi_get_wifi_version(version, length);

	p = strstr(version, "FWID");
	if (p)
		p[0] = 0;
	
    return ret;
}
#endif

int wlan_set_max_station_num(int number)
{
    if ((number > 0) && (number < 5))
        return wiced_wifi_set_maxassoc(number);
    else
        return 0;
}

#ifdef __IAR_SYSTEMS_ICC__
WEAK struct mxchip_mallinfo* mico_memory_info(void)
{
    static struct mxchip_mallinfo info;
    struct mallinfo iar_info = __iar_dlmallinfo();

    info.num_of_chunks = iar_info.ordblks;
    info.total_memory = iar_info.usmblks;
    info.allocted_memory = iar_info.uordblks;
    info.free_memory = iar_info.fordblks;
    return &info;
}
#endif



