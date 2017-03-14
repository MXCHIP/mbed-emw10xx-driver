#ifndef __WIFI_MGR_H__
#define __WIFI_MGR_H__

#include "mico_wlan.h"

enum {
    AP_CLIENT_MODE = 0,
    AD_HOC_MODE = 1,
    AP_SERVER_MODE = 2,
};

enum {
    WIFI_STATE_STA_UP,
    WIFI_STATE_STA_DOWN,
    WIFI_STATE_UAP_UP,
    WIFI_STATE_UAP_DOWN,
};

enum {
    WIFI_MODE_UAP,
    WIFI_MODE_STA,
};

typedef enum {
    SEC_TYPE_NONE = 0,
    SEC_TYPE_WEP,
    SEC_TYPE_WPA_TKIP,
    SEC_TYPE_WPA_AES,
    SEC_TYPE_WPA2_TKIP,
    SEC_TYPE_WPA2_AES,
    SEC_TYPE_WPA2_MIXED,
} MXCHIP_SEC_TYPE_E;



typedef struct _mxchip_wifi_config_ 
{ 
    char wifi_mode;    // SoftAp(0)��sta(1)  
    char wifi_ssid[32]; 
    char wifi_key[64]; 
    char local_ip_addr[16]; 
    char net_mask[16]; 
    char gateway_ip_addr[16]; 
    char dnsServer_ip_addr[16]; 
    char dhcpMode;       // disable(0), client mode(1), server mode(2) 
    char address_pool_start[16]; 
    char address_pool_end[16]; 
    int wifi_retry_interval;//sta reconnect interval, ms
} mxchip_wifi_config_t; 

typedef  struct  _ap_info_  
{  
    char ssid[32];  
    char ApPower;  // min:0, max:100
}ap_info_t; 


typedef  struct  _ap_list_info_  
{  
  char ApNum;       //AP number
  ap_info_t * ApList; 
} ap_list_info_t;  

typedef  struct  _ap_info_adv_  
{  
    char ssid[32];  
    int16_t signal_strength;  /**< Receive Signal Strength Indication in dBm. <-90=Very poor, >-30=Excellent */
    char bssid[6];
    char channel;
    MXCHIP_SEC_TYPE_E security;
}ap_info_adv_t; 

typedef  struct  _ap_list_info_adv_  
{  
  char ApNum;       //AP number
  ap_info_adv_t * ApList; 
} ap_list_info_adv_t;  


typedef struct _mxchip_wifi_adv_config_ 
{ 
    apinfo_adv_t ap_info;
    char key[64];
    int key_len;
    char local_ip_addr[16]; 
    char net_mask[16]; 
    char gateway_ip_addr[16]; 
    char dnsServer_ip_addr[16]; 
    char dhcpMode;       // disable(0), client mode(1), server mode(2) 
    char address_pool_start[16]; 
    char address_pool_end[16]; 
    int wifi_retry_interval;//sta reconnect interval, ms
} mxchip_wifi_adv_config_t; 

struct mxchip_config_st {
	uint8_t need_health_monitor; // 0=disable health monitor. default = 1
	uint8_t reserved[15];
};

typedef void (*inert_ap_list_t)(char *ssid, uint8_t *bssid, uint8_t channel, int rssi );
void get_easylink_channel_list(inert_ap_list_t insert);
void mxchip_system_configure(struct mxchip_config_st config);

void wlan_init( void );
void mxchip_thread_init(void);
void sta_up( void );
int user_scan(int advance_scan);
int user_active_scan(char *ssid, int advance_scan);
int get_sta_connection(void);

int dns_client_add_server_address( wiced_ip_address_t address );
int dns_client_remove_all_server_addresses( void );


#endif // __WIFI_MGR_H__ END
