/*
  * Smart config: support Easylink, Airkiss, 
  *
  */
#include "smart_config.h"
#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wpointer-sign"
#pragma GCC diagnostic ignored "-Wformat"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wimplicit-function-declaration"
#endif /* ifdef __GNUC__ */


#include <stdlib.h>
#include "wiced.h"
#include "wiced_wifi.h"
#include "wwd_network_interface.h"
#include "wwd_buffer_interface.h"
#include "../mxchipwnet/mxchip_debug.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define SC_OFFSET_LEN 64 // ip(20)+udp(8)+mac_header(24)+fcs(4)+LLC(8)

#define TYPE_SUBTYPE_MASK 0xfc 
#define TYPE_MASK     0x0c
#define FLAG_MASK            0x03
#define BROADCAST_MASK       0xff

#define RETRY_MASK            0x0010

#define DATA_ZERO 0x0000

#define BEACON_TYPE           0x80
#define PROBE_RESPONSE_TYPE   0x50
#define PROBE_REQUEST_TYPE    0x40
#define ACTION_TYPE           0xD0
#define MANAGEMENT_TYPE       0x00
#define DATA_TYPE             0x08 // 

#define PKT_STA_TO_AP         0x01
#define PKT_AP_FWD            0x02

#define PROTECT_FRAM          0x40

#define PKT_START_FLAG1 0x5AA
#define PKT_START_FLAG2 0x5AB
#define PKT_START_FLAG3 0x5AC
#define PKT_SEQ_FLAG1   0x500
#define PKT_SEQ_FLAG2   0x540
#define SPKT_START_FLAG1 0xAA
#define SPKT_START_FLAG2 0xAB
#define SPKT_START_FLAG3 0xAC
#define SPKT_SEQ_FLAG1   0xB0
#define SPKT_SEQ_FLAG2   0xF0

#define MAX_DATA_LENGTH 256
#define FROM_DS                   2
#define TO_DS                     1
#define DS_BITMASK               (FROM_DS|TO_DS) // [FROM DS][TO DS] 

#define ALINK_START_FLAG 0x4E0
#define ALINK_DATA_MIN   0x100
#define ALINK_DATA_MAX   0x4DE
#define ALINK_SEQ_FLAG1  0x3E1
#define ALINK_SEQ_FLAG2  0x3EF
#define ALINK_DATA_MASK  0x7F
#define ALINK_DATA_SHIFT 7

#define SSID_START 0x01
#define SSID_FLAG  0x7E
#define SSID_PASS  "mxchip_easylink_minus"


#define MAX_SCAN_TIME 50 // 50ms for each channel
#define MAX_GET_SSID_TIME 10000 // 4 seconds to get ssid & key

/******************************************************
 *                   Enumerations
 ******************************************************/
enum {
    SC_STOPPED = 0,
    SC_SCANING,  // data length from 1~4, get the channel and offset length
    SC_NEWFILTER,
    SC_MAGIC, //magic code
    SC_RECVING,  // LOOP( prefix code, data)
    SC_DONE,    // success
    SC_FAIL,
};

enum {
    PKT_STATE_NONE = 0,
    PKT_STATE_APFWD = 1,
    PKT_STATE_STA = 2,
    DATA_SET = 1,
    DATA_VALID = 2,
};

typedef enum{
  /* MICO system defined notifications */
  CONFIG_BY_NONE,
  CONFIG_BY_EASYLINK_V2,         
  CONFIG_BY_EASYLINK_PLUS,        
  CONFIG_BY_EASYLINK_MINUS,          
  CONFIG_BY_AIRKISS,             
  CONFIG_BY_SOFT_AP,  
  CONFIG_BY_WAC,   
  CONFIG_BY_ALINK,
} mico_config_source_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/
#define ARRAY_LEN(x) sizeof(x)/sizeof(x[0])
#define GET_BIT(x, bit)     (x[bit / 32] & (1 << (bit&0x1F)))
#define SET_BIT(x, bit)     (x[bit / 32] |= (1 << (bit&0x1F)))
#define RESET_BIT(x, bit)   (x[bit / 32] &= ~(1 << (bit&0x1F)))

typedef void (*inert_ap_list_t)(char *ssid, uint8_t *bssid, uint8_t channel, int rssi );

#define MAX_AP_TBL_NUM 32

/* Bit mask define the configure mode */
enum {
    MODE_EASYLINK = 1,
    MODE_ALINK = 2,
    MODE_AIRKISS = 4,
    MODE_PROBEREQ = 8,
    MODE_EASYLINKV2 = 0x10,
    
    MODE_AUTO = 0xFF,
};

enum {
    PKT_TYPE_DATA = 0,
    PKT_TYPE_EL_START,
    PKT_TYPE_EL_INDEX,
    PKT_TYPE_AL_START,
    PKT_TYPE_AL_INDEX,
    PKT_TYPE_AK_START,
    PKT_TYPE_ELV2,
};


enum {
    STA_SEND = 0,
    AP_FWD,

    INVALID_DIR,
    BOTH_DIR,
};
#define PKT_START_FLAG_MIN 1
#define PKT_START_FLAG_MAX 4

/******************************************************
 *                    Structures
 ******************************************************/
struct pkt_info_st {
    uint8_t  bssid[6];
    uint8_t  initiator[6];
    uint32_t last_data;
    uint32_t last_seq;
    int      last_index;
    int      valid;
    int      protect_type;
    int      offset;
    int      pktnum;
};

#pragma pack(1)
#ifdef EXTRA_HEADER
#define RADIO_RSSI_INDEX 8
#define RADIO_NOISE_INDEX 9

typedef struct bcm_header_rx {
    uint8_t   it_version;
    uint8_t   it_pad;
    uint16_t  it_len;
    uint32_t  it_present;
    

    
    int8_t rssi;
    int8_t noise;
    uint8_t pad[2];
} bcm_header_rx_t;
#endif

typedef struct
{
#ifdef EXTRA_HEADER
	int8_t radio[12];
#endif
    uint8_t type;
    uint8_t flags;
    uint16_t duration;
    uint8_t address1[6];
    uint8_t address2[6];
    uint8_t address3[6];
    uint16_t seq;
    uint8_t data[1];
} ieee80211_header_t;

typedef struct
{
    char    ssid[32];
    uint8_t bssid[6];
    uint8_t channel;
    uint8_t crc8;
	int rssi;
} ap_tbl_t;

typedef struct 
{
    uint8_t data;
    uint8_t state; // 0=none, 1=set, 2=valid
} data_element_t;

#pragma pack()

/******************************************************
 *               Function Declarations
 ******************************************************/
static int total_data_validate(void);

void get_easylink_channel_list(inert_ap_list_t insert);
/* return 0=success.
  * ie point to ali IE.
  * tpsk point to a 64 bytes buffer, save the tpsk result.
  */
int alink_get_tpsk(uint8_t *ie,  char *tpsk);

static void (*sniffer_callback)(uint8_t*data, int len) = NULL;

static void fill_ssid(char*apssid, int mode);

/******************************************************
 *               Variable Definitions
 ******************************************************/
/* Setup a filter to catch beacon frames */
static const uint8_t tods_bcast_mask[]   = {TYPE_MASK, FLAG_MASK,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xff,0xff,0xff,0xff,0xff,0xff};
static const uint8_t tods_bcast_match[]  = {DATA_TYPE, PKT_STA_TO_AP,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xff,0xff,0xff,0xff,0xff,0xff};
static const uint8_t frmds_bcast_mask[]  = {TYPE_MASK, FLAG_MASK,0,0,0xff,0xff,0xff,0xff,0xff,0xff};
static const uint8_t frmds_bcast_match[] = {DATA_TYPE, PKT_AP_FWD,0,0,0xff,0xff,0xff,0xff,0xff,0xff};
static const uint8_t type_subtype_mask[] = {TYPE_SUBTYPE_MASK};
static const uint8_t type_mask[]         = {TYPE_MASK};

static const uint8_t beacon_match[]      = {BEACON_TYPE};
static const uint8_t data_match[]        = {DATA_TYPE};
static const uint8_t management_match[]  = {MANAGEMENT_TYPE};

static const uint8_t probe_rsp_match[]   = {PROBE_RESPONSE_TYPE};
static const uint8_t probe_req_match[]   = {PROBE_REQUEST_TYPE};
static const uint8_t action_match[]      = {ACTION_TYPE};

static const wiced_packet_filter_pattern_t tods_brast_pattern[1] =
{
    {
        .offset = 0,
        .mask_size = sizeof(tods_bcast_mask),
        .mask    = (uint8_t*)tods_bcast_mask,
        .pattern = (uint8_t*)tods_bcast_match,
    },

};

static const wiced_packet_filter_pattern_t frmds_brast_pattern[1] =
{
    {
        .offset = 0,
        .mask_size = sizeof(frmds_bcast_mask),
        .mask    = (uint8_t*)frmds_bcast_mask,
        .pattern = (uint8_t*)frmds_bcast_match,
    },

};
static const wiced_packet_filter_pattern_t beacon_pattern[1] =
{
    {
        .offset = 0,
        .mask_size = sizeof(type_subtype_mask),
        .mask    = (uint8_t*)type_subtype_mask,
        .pattern = (uint8_t*)beacon_match,
    },

};
static const wiced_packet_filter_pattern_t probe_rsp_pattern[1] =
{
    {
        .offset = 0,
        .mask_size = sizeof(type_subtype_mask),
        .mask    = (uint8_t*)type_subtype_mask,
        .pattern = (uint8_t*)probe_rsp_match,
    },

};

static const wiced_packet_filter_pattern_t probe_req_pattern[1] =
{
    {
        .offset = 0,
        .mask_size = sizeof(type_subtype_mask),
        .mask    = (uint8_t*)type_subtype_mask,
        .pattern = (uint8_t*)probe_req_match,
    },

};

static const wiced_packet_filter_pattern_t data_pattern[1] =
{
    {
        .offset = 0,
        .mask_size = sizeof(type_mask),
        .mask    = (uint8_t*)type_mask,
        .pattern = (uint8_t*)data_match,
    },

};
static const wiced_packet_filter_pattern_t management_pattern[1] =
{
    {
        .offset = 0,
        .mask_size = sizeof(type_mask),
        .mask    = (uint8_t*)type_mask,
        .pattern = (uint8_t*)management_match,
    },

};

static const wiced_packet_filter_pattern_t action_pattern[1] =
{
    {
        .offset = 0,
        .mask_size = sizeof(type_subtype_mask),
        .mask    = (uint8_t*)type_subtype_mask,
        .pattern = (uint8_t*)action_match,
    },

};


static const wiced_packet_filter_settings_t tods_filter =
{
    .rule = WICED_PACKET_FILTER_RULE_POSITIVE_MATCHING,
    .pattern_count = 1,
    .pattern_list = (wiced_packet_filter_pattern_t*)tods_brast_pattern,
};

static const wiced_packet_filter_settings_t frmds_filter =
{
    .rule = WICED_PACKET_FILTER_RULE_POSITIVE_MATCHING,
    .pattern_count = 1,
    .pattern_list = (wiced_packet_filter_pattern_t*)frmds_brast_pattern,
};

static const wiced_packet_filter_settings_t beacon_filter =
{
    .rule = WICED_PACKET_FILTER_RULE_POSITIVE_MATCHING,
    .pattern_count = 1,
    .pattern_list = (wiced_packet_filter_pattern_t*)beacon_pattern,
};

static const wiced_packet_filter_settings_t probe_rsp_filter =
{
    .rule = WICED_PACKET_FILTER_RULE_POSITIVE_MATCHING,
    .pattern_count = 1,
    .pattern_list = (wiced_packet_filter_pattern_t*)probe_rsp_pattern,
};
static const wiced_packet_filter_settings_t probe_req_filter =
{
    .rule = WICED_PACKET_FILTER_RULE_POSITIVE_MATCHING,
    .pattern_count = 1,
    .pattern_list = (wiced_packet_filter_pattern_t*)probe_req_pattern,
};

static const wiced_packet_filter_settings_t data_filter =
{
    .rule = WICED_PACKET_FILTER_RULE_POSITIVE_MATCHING,
    .pattern_count = 1,
    .pattern_list = (wiced_packet_filter_pattern_t*)data_pattern,
};

static const wiced_packet_filter_settings_t management_filter =
{
    .rule = WICED_PACKET_FILTER_RULE_POSITIVE_MATCHING,
    .pattern_count = 1,
    .pattern_list = (wiced_packet_filter_pattern_t*)management_pattern,
};

static const wiced_packet_filter_settings_t action_filter =
{
    .rule = WICED_PACKET_FILTER_RULE_POSITIVE_MATCHING,
    .pattern_count = 1,
    .pattern_list = (wiced_packet_filter_pattern_t*)action_pattern,
};

static const uint8_t cooee_upper_address_mask[]  = {0xFF, 0xFF, 0xFF, 0xFF};
static const uint8_t cooee_upper_address_match[] = {0x01, 0x00, 0x5e, 0x7e};

static const uint8_t cooee_beacon_address_mask[]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const uint8_t cooee_beacon_address_match[] = {0x01, 0x00, 0x5e, 0x76, 0x00, 0x00};

static const uint8_t mcast_mask[]  = {0xFF, 0xFF, 0xFF};
static const uint8_t mcast_match[] = {0x01, 0x00, 0x5e};

static const wiced_packet_filter_pattern_t mcast1_pattern =
{
    .offset       = 4,
    .mask_size    = sizeof(mcast_mask),
    .mask         = (uint8_t*)cooee_upper_address_mask,
    .pattern      = (uint8_t*)cooee_upper_address_match,
};

static const wiced_packet_filter_pattern_t mcast2_pattern =
{
    .offset       = 16,
    .mask_size    = sizeof(mcast_mask),
    .mask         = (uint8_t*)mcast_mask,
    .pattern      = (uint8_t*)mcast_match,
};

static const wiced_packet_filter_pattern_t a1_pattern =
{
    .offset       = 4,
    .mask_size    = 4,
    .mask         = (uint8_t*)cooee_upper_address_mask,
    .pattern      = (uint8_t*)cooee_upper_address_match,
};

static const wiced_packet_filter_pattern_t a3_pattern =
{
    .offset       = 16,
    .mask_size    = 4,
    .mask         = (uint8_t*)cooee_upper_address_mask,
    .pattern      = (uint8_t*)cooee_upper_address_match,
};

static const wiced_packet_filter_pattern_t beacon_a1_pattern =
{
    .offset       = 4,
    .mask_size    = 4,
    .mask         = (uint8_t*)cooee_beacon_address_mask,
    .pattern      = (uint8_t*)cooee_beacon_address_match,
};

static const wiced_packet_filter_pattern_t beacon_a3_pattern =
{
    .offset       = 16,
    .mask_size    = 4,
    .mask         = (uint8_t*)cooee_beacon_address_mask,
    .pattern      = (uint8_t*)cooee_beacon_address_match,
};

static const wiced_packet_filter_settings_t a1_settings =
{
    .rule = WICED_PACKET_FILTER_RULE_POSITIVE_MATCHING,
    .pattern_count = 1,
    .pattern_list  = (wiced_packet_filter_pattern_t*)&a1_pattern,
};

static const wiced_packet_filter_settings_t a3_settings =
{
    .rule = WICED_PACKET_FILTER_RULE_POSITIVE_MATCHING,
    .pattern_count = 1,
    .pattern_list  = (wiced_packet_filter_pattern_t*)&a3_pattern,
};

static const wiced_packet_filter_settings_t mcast1_settings =
{
    .rule = WICED_PACKET_FILTER_RULE_POSITIVE_MATCHING,
    .pattern_count = 1,
    .pattern_list  = (wiced_packet_filter_pattern_t*)&mcast1_pattern,
};

static const wiced_packet_filter_settings_t mcast2_settings =
{
    .rule = WICED_PACKET_FILTER_RULE_POSITIVE_MATCHING,
    .pattern_count = 1,
    .pattern_list  = (wiced_packet_filter_pattern_t*)&mcast2_pattern,
};

static const wiced_packet_filter_settings_t beacon_a1_settings =
{
    .rule = WICED_PACKET_FILTER_RULE_POSITIVE_MATCHING,
    .pattern_count = 1,
    .pattern_list  = (wiced_packet_filter_pattern_t*)&beacon_a1_pattern,
};

static const wiced_packet_filter_settings_t beacon_a3_settings =
{
    .rule = WICED_PACKET_FILTER_RULE_POSITIVE_MATCHING,
    .pattern_count = 1,
    .pattern_list  = (wiced_packet_filter_pattern_t*)&beacon_a3_pattern,
};

static int sc_state = SC_STOPPED;
static int sc_channel = 0;

static struct pkt_info_st pkt_tbl[2]; // 0=sta to AP, 1=AP forward

static unsigned char pkt_buffer[MAX_DATA_LENGTH];
static unsigned char pkt_state[MAX_DATA_LENGTH];

//static unsigned char *realbssid = NULL;
static int pkt_num;

static mico_semaphore_t  *p_sem = NULL;
static uint8_t channel_tbl[14];
static int max_channel = 13, max_channel_index = 0;

/* added for AWS & easylink */
static int detect_easylink_in_monitor = 0; /* user monitor mode detected easylink mode */
static int _easylink_start=0;

static int sc_loop_num=0;
static int thread_created=0;
static int received_byte_count;
static ap_tbl_t ap_tbl[MAX_AP_TBL_NUM];
static uint8_t ap_num = 0;
static uint8_t current_channel;
static int easylink_break = 0;
static int mgmt_smart_type = 0;
static char *apssid;
static int mac_monitor = 0;
static char tpsk[64], tssid[32];
static uint8_t remote_ip[6];
static data_element_t magic_tbl[5][4];
static data_element_t prefix_tbl[4];
static data_element_t data_tbl[6];
static int valid_dir;
static int prefix_valid = 0;
static int total_data_len;
static int data_index=0;
static uint8_t valid_ssid=MAX_AP_TBL_NUM;
static uint8_t *p_minus_buffer=NULL, minus_pkt_num, minus_got_pkt[7], minus_pkt_len;
static int monitor_ignore = 0;
static int config_mode, work_mode; // user configure mode, choosed work mode.
static const unsigned char CRC8Table[]={
  0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
  157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
  35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
  190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
  70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
  219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
  101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
  248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
  140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
  17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
  175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
  50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
  202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
  87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
  233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
  116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};

static int es_switch_interval = MAX_SCAN_TIME, es_recv_time=MAX_GET_SSID_TIME;

static uint8_t *es_key = NULL;
static int es_key_len = 0;

void set_easylink_systime(int switch_interval, int receive_time)
{
    es_switch_interval = switch_interval;
    if (es_switch_interval < 40)
        es_switch_interval = 40;
    es_recv_time = receive_time;
}

/******************************************************
 *               Function Definitions
 ******************************************************/
extern void smart_config_done(char *ssid, char *key, int mode);
extern void easylink_user_data_result(int datalen, char*data);
extern void wifi_reboot_only(void);
extern void msleep(uint32_t ms);
extern void easylink_wifi_info(uint8_t*bssid, int rssi);

void autoconfig_start(int seconds, int mode);

static int easy_link_start_cmd(void);

static void reset_pkt_buffer(void)
{
    memset(pkt_buffer, 0, sizeof(pkt_buffer));
    memset(pkt_state, 0, sizeof(pkt_state));
    received_byte_count = 0;
}

static void reset_recv_state(void)
{
    int i;

    {
        if (pkt_buffer[0] > 0) {
            for(i=0;i<pkt_buffer[0];i++) 
                system_debug_printf(SYSTEM_DEBUG_DEBUG, "%02x ", pkt_buffer[i]);
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "\r\n");
        }
    }
    memset(pkt_tbl, 0, sizeof(pkt_tbl));
    memset(pkt_buffer, 0, sizeof(pkt_buffer));
    memset(pkt_state, 0, sizeof(pkt_state));
    received_byte_count = 0;
    pkt_num = 0;
    apssid = NULL;
    valid_dir = INVALID_DIR;
    memset(magic_tbl, 0, sizeof(magic_tbl));
    total_data_len = 0;
    prefix_valid = 0;
    sc_state = SC_SCANING;
}

static char *find_ssid_by_bssid(uint8_t *bssid)
{
    int i;

    for(i=0; i<MAX_AP_TBL_NUM; i++) {
        if (memcmp(bssid, ap_tbl[i].bssid, 6) == 0)
            return ap_tbl[i].ssid;
    }

    return NULL;
}

static void reset_without_dataclean(void)
{
    int i;
    
    memset(pkt_tbl, 0, sizeof(pkt_tbl));
    for(i=0; i<MAX_DATA_LENGTH; i++) {
        if (pkt_state[i] != PKT_STATE_NONE)
            pkt_state[i] = PKT_STATE_APFWD;
    }
    pkt_num = 0;
    valid_dir = INVALID_DIR;

    memset(magic_tbl, 0, sizeof(magic_tbl));
    total_data_len = 0;
    prefix_valid = 0;
}

static void easy_link_fail(void)
{
    smart_config_done(NULL, NULL, CONFIG_BY_NONE);
}

void report_ap_info(char *ssid)
{
	int i, rssi = -999;
	
	for(i=0; i<ap_num; i++) {
        if (strcmp(ssid, ap_tbl[i].ssid) == 0) {
            if (rssi < ap_tbl[i].rssi)
				rssi = ap_tbl[i].rssi;
        }
    }
	easylink_wifi_info(ap_tbl[i].bssid, rssi);
}

static void Airkiss_success(void)
{
    char num;
    int i,j, key_len = (prefix_tbl[0].data<<4) + prefix_tbl[1].data;

    memset(tssid, 0, sizeof(tssid));
    memset(tpsk, 0, sizeof(tpsk));

    system_debug_printf(SYSTEM_DEBUG_DEBUG, "key len %d\r\n", key_len);
    for(i=0; i<key_len;i++ ) {
        tpsk[i] = pkt_buffer[i];
    }

    num = pkt_buffer[i++];

    j=0;
    while(i<total_data_len) {
       tssid[j++] = pkt_buffer[i++]; 
    }

    smart_config_done(tssid, tpsk, CONFIG_BY_AIRKISS);
    easylink_user_data_result(1, &num);
}

static void easy_link_success(void)
{
    char *data;
    int totlen, ssidlen, keylen, datalen;

    sc_state = SC_STOPPED;
    memset(tssid, 0, sizeof(tssid));
    memset(tpsk, 0, sizeof(tpsk));
    totlen = pkt_buffer[0];
    ssidlen = pkt_buffer[1];
    keylen = pkt_buffer[2];
    datalen = totlen - ssidlen - keylen - 5;
    memcpy(tssid, &pkt_buffer[3], ssidlen);
    memcpy(tpsk, &pkt_buffer[3+ssidlen], keylen);

    data = (char*)&pkt_buffer[3 + ssidlen + keylen];

	if (es_key_len) {
		arc4_decrypt(tssid, tssid, es_key, es_key_len, ssidlen);
		arc4_decrypt(tpsk, tpsk, es_key, es_key_len, keylen);
	}
    smart_config_done(tssid, tpsk, CONFIG_BY_EASYLINK_PLUS);
    easylink_user_data_result(datalen, data);
}

static void decode_chinese(uint8_t *in, uint8_t in_len, uint8_t *out) 
{ 
  uint8_t bit[32 * 8]; 
  uint8_t out_len = (in_len * 6) / 8; 
  uint8_t i, j; 
   
  //char to bit stream 
  for (i = 0; i < in_len; i++) { 
    for (j = 0; j < 6; j++) { 
      bit[i * 6 + j] = (in[i] >> j) & 0x01; 
    } 
  } 
   
  out[out_len] = '\0'; /* NULL-terminated */ 
  for (i = 0; i < out_len; i++) { 
    for (j = 0, out[i] = 0; j < 8; j++) { 
      out[i] |= bit[i * 8 + j] << j; 
      //info("%02x ", out[i]); 
    } 
  } 
}

static void alink_success(void)
{
    char *pbuf;
    int ssidlen, keylen, datalen, flag, i;
    
    sc_state = SC_STOPPED;
    memset(tssid, 0, sizeof(tssid));
    memset(tpsk, 0, sizeof(tpsk));
    pbuf = pkt_buffer;
    /* 
          pbuf[] hold the all things:  
          total_len, flag, [ssid_len], passwd_len, ssid, passwd, checksum 
        */ 
    //totlen = pbuf[0]; /* len @ [0] */   
    flag = pbuf[1];   /* flag @ [1] */ 
    pbuf += 2;    /* 2B for total_len, flag */ 
    if (flag & 1) {
        ssidlen = pbuf[0]; 
        keylen = pbuf[1]; 
        pbuf += 2; /* 2B for ssid_len, passwd_len */ 
        snprintf((char *)tssid, ssidlen + 1, "%s", pbuf); 
        if (!(flag & (1 << 5))) {//ascii 
          for (i = 0; i < ssidlen; i++) 
            tssid[i] += 32; 
        } else {//chinese format 
          decode_chinese(pbuf, ssidlen, tssid); 
        } 
        pbuf += ssidlen; /* ssid */    
    } else {
        keylen = pbuf[0];   
        pbuf += 1; /* 1B for passwd_len */ 
    }
    snprintf((char *)tpsk, keylen + 1, "%s", pbuf); 
    for (i = 0; i < keylen; i++) 
      tpsk[i] += 32; 
    smart_config_done(tssid, tpsk, CONFIG_BY_ALINK);
}

static void easylinkv2_success(void)
{
    char *data = (char*)&pkt_buffer[2];

    memset(tssid, 0, sizeof(tssid));
    memset(tpsk, 0, sizeof(tpsk));
    
    memcpy(tssid, data, pkt_buffer[0]);
    data += pkt_buffer[0];
    memcpy(tpsk, data, pkt_buffer[1]);
    data += pkt_buffer[1];

	if (es_key_len) {
		arc4_decrypt(tssid, tssid, es_key, es_key_len, pkt_buffer[0]);
		arc4_decrypt(tpsk, tpsk, es_key, es_key_len, pkt_buffer[1]);
	}
    smart_config_done(tssid, tpsk, CONFIG_BY_EASYLINK_V2);
    if ((pkt_buffer[0] + pkt_buffer[1]) % 2)
        data++;
    easylink_user_data_result(data[0], &data[2]);

}

int get_sniffer_channel(void)
{
	if(sc_loop_num > 2) { 
        current_channel++;
        if (current_channel > max_channel) {
            sc_loop_num = 0;
            sc_channel = 0;
            current_channel = channel_tbl[sc_channel++];
        }
    } else {
        if(sc_channel >= max_channel_index) {
            sc_channel = 0;
            sc_loop_num++;
        }
        if(sc_loop_num > 2)
           current_channel = 1;
        else
           current_channel = channel_tbl[sc_channel++];
    }
	
	return current_channel;
}

static void switch_channle(void)
{
    int retrynum;

    if (pkt_num>1)
        msleep(100);
    
    if (p_minus_buffer != NULL) {
        msleep(500);
    }
    if (SC_SCANING != sc_state)
        return;

    reset_without_dataclean();

    // receive easylink pkt, switch channel from the scanned AP channel list.
    // 3 times switch channel based on the scanned AP channel, 
    // 1 time  switch channel from 1 to max_channel.
    get_sniffer_channel();
	
    work_mode = 0;
    system_debug_printf(SYSTEM_DEBUG_DEBUG, "channel %d\r\n", current_channel);
    //realbssid = NULL;
    retrynum = 0;
    monitor_ignore = 1;
    while (WICED_SUCCESS != wiced_wifi_set_channel(current_channel)) {
        retrynum++;
        if (retrynum > 3) {
            wifi_reboot_only();
            easy_link_start_cmd();
            monitor_ignore = 0;
            return;
        }
        msleep(10);
    }
    monitor_ignore = 0;
}

static void get_ssid_key_end(void)
{
    sc_state = SC_SCANING;
    pkt_num = 0;
    switch_channle();
    msleep(es_switch_interval);
}
static void print_buffer(void)
{
    int i;

    system_debug_printf(SYSTEM_DEBUG_DEBUG, "\r\nRX %d\r\ndata:", received_byte_count);
    for(i=0;i<received_byte_count;i++) {
        system_debug_printf(SYSTEM_DEBUG_DEBUG, "%x ", pkt_buffer[i]);
        
    }
    system_debug_printf(SYSTEM_DEBUG_DEBUG, "\r\nstate:");
    for(i=0;i<received_byte_count;i++) {
        system_debug_printf(SYSTEM_DEBUG_DEBUG, "%d ", pkt_state[i]);
    }
    system_debug_printf(SYSTEM_DEBUG_DEBUG, "\r\n");
}

void smart_insert_tbl(char *ssid, uint8_t *bssid, uint8_t channel, int rssi)
{
    int i, j, len;
    uint8_t crc8 = 0;

    if (bssid && (ap_num < MAX_AP_TBL_NUM)) {
        for(i=0; i<ap_num; i++) {
            if (memcmp(bssid, ap_tbl[i].bssid, 6) == 0)// already inserted, return;
                return;
        }
        
        strncpy(ap_tbl[i].ssid, ssid, 32);
        memcpy(ap_tbl[i].bssid, bssid, 6);
        ap_tbl[i].channel = channel;
		ap_tbl[i].rssi = rssi;
        ap_num++;
        
        len = strlen(ssid);
        if (len > 32)
            len = 32;
        if (len > 0) {
            crc8 = 0;
            for(j=0; j<len; j++) {
                crc8 = CRC8Table[crc8^ssid[j]];
            }
            ap_tbl[i].crc8 = crc8;
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "ssid %s, CRC %02x\r\n", 
                ap_tbl[i].ssid, ap_tbl[i].crc8);
        }
    }
    
    for(i=0;i<max_channel_index;i++) {
        if (channel == channel_tbl[i])
            break;
    }
    if (i==max_channel_index) {
        channel_tbl[i] = channel;
        max_channel_index++;
    }
}

static void easy_link_main_thread( uint32_t timeout )
{
    uint32_t end_time, cur_time;
    wiced_result_t ret = WICED_ERROR;
    mico_semaphore_t  easylink_sem;
    int choose_dir = INVALID_DIR;

    wiced_network_suspend();
    mico_rtos_init_semaphore(&easylink_sem, 1);
    if (easylink_break) {
        sc_state = SC_FAIL;
        goto QUIT;
    }
    system_debug_printf(SYSTEM_DEBUG_DEBUG, "Smart Configue start...\r\n");
    ap_num = 0;
    get_easylink_channel_list(smart_insert_tbl);
    {
        int i;
        
        system_debug_printf(SYSTEM_DEBUG_DEBUG, "Got AP Channel list: ");
        for(i=0; i<ap_num; i++) {
            system_debug_printf(SYSTEM_DEBUG_DEBUG, 
                "%02x-%02x-%02x-%02x-%02x-%02x %d\r\n", ap_tbl[i].bssid[0],
                ap_tbl[i].bssid[1],ap_tbl[i].bssid[2],ap_tbl[i].bssid[3],
                ap_tbl[i].bssid[4],ap_tbl[i].bssid[5],ap_tbl[i].channel);
        }
    }
	if (ap_num == 0) { // scan fail.
		int i;
		
		for(i=1;i<14;i++)
            smart_insert_tbl("", NULL, i, 0);
	}
    
    if (easylink_break) {
        sc_state = SC_FAIL;
        goto QUIT;
    }
    easy_link_start_cmd();
    if (easylink_break) {
        sc_state = SC_FAIL;
        goto QUIT;
    }
    
    p_sem = &easylink_sem;
    
    cur_time = mico_rtos_get_time();
    end_time = cur_time + timeout;
    if (end_time < cur_time)
        end_time = 0xFFFFFFFF;
    
    while(1) {
        if (easylink_break) {
            sc_state = SC_FAIL;
            goto QUIT;
        }
        cur_time = mico_rtos_get_time();
        if (cur_time > end_time) {// timeout, should call callback function.
            sc_state = SC_FAIL;
            goto QUIT;
        }
        if (sc_state == SC_SCANING) {
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "scanning ");
            switch_channle();
            msleep(es_switch_interval);
        } else if ((sc_state == SC_NEWFILTER) || (sc_state == SC_MAGIC) || (sc_state == SC_RECVING)) {
            if (sc_state == SC_NEWFILTER) {
                msleep(100);
            }
            if (pkt_tbl[AP_FWD].valid) {
                if (pkt_tbl[STA_SEND].valid) {
                    if (pkt_tbl[AP_FWD].pktnum > pkt_tbl[STA_SEND].pktnum) {
                        choose_dir = AP_FWD;
                    } else {
                        choose_dir = STA_SEND;
                    }
                } else {
                    choose_dir = AP_FWD;
                }
            } else {
                choose_dir = STA_SEND;
            }
            
            if (work_mode & MODE_AIRKISS)
                work_mode = MODE_AIRKISS;
            else if (work_mode & MODE_EASYLINK) {
                work_mode = MODE_EASYLINK;
                sc_state = SC_RECVING;
            }
            
            if (pkt_tbl[STA_SEND].valid && pkt_tbl[AP_FWD].valid) {
                if (memcmp(pkt_tbl[STA_SEND].initiator, pkt_tbl[AP_FWD].initiator, 6) != 0) {
                    system_debug_printf(SYSTEM_DEBUG_DEBUG, "PKTTBL MISMATCH! Choose: %02x-%02x-%02x-%02x-%02x-%02x %02x-%02x-%02x-%02x-%02x-%02x\r\n", 
                        pkt_tbl[choose_dir].initiator[0],pkt_tbl[choose_dir].initiator[1],pkt_tbl[choose_dir].initiator[2],
                        pkt_tbl[choose_dir].initiator[3],pkt_tbl[choose_dir].initiator[4],pkt_tbl[choose_dir].initiator[5],
                        pkt_tbl[choose_dir].bssid[0],pkt_tbl[choose_dir].bssid[1],pkt_tbl[choose_dir].bssid[2],
                        pkt_tbl[choose_dir].bssid[3],pkt_tbl[choose_dir].bssid[4],pkt_tbl[choose_dir].bssid[5]);
                } else {
                    if (work_mode != MODE_AIRKISS)
                        choose_dir = BOTH_DIR;
                }
            }
            reset_pkt_buffer();
            valid_dir = choose_dir;
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "Choose DIR %d, %d-%d, mode %x\r\n", choose_dir, 
                pkt_tbl[STA_SEND].pktnum, pkt_tbl[AP_FWD].pktnum, work_mode);
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "recv, wait semphore %p \r\n", &easylink_sem);
            if (apssid) {
                fill_ssid(apssid, work_mode);
            }
            
            ret = mico_rtos_get_semaphore(&easylink_sem, es_recv_time);
            if(ret == WICED_SUCCESS) {
                system_debug_printf(SYSTEM_DEBUG_DEBUG, "recv sem \r\n");
                print_buffer();
                goto QUIT;
            } else {
                system_debug_printf(SYSTEM_DEBUG_DEBUG, "recv sem timeout\r\n");
                print_buffer();
                get_ssid_key_end();
            }
        } else {
            goto QUIT;
        }
        if (p_sem == NULL) {
            goto QUIT;
        }
    }

QUIT:
    wiced_network_resume();
    if (mgmt_smart_type == 0) {
        if (sc_state == SC_DONE) {
            if (work_mode == MODE_EASYLINKV2) {
                easylinkv2_success();
            } else if (work_mode == MODE_ALINK) {
                alink_success();
            } else if (work_mode == MODE_EASYLINK) {
                easy_link_success();
            } else if (work_mode == MODE_AIRKISS) {
                Airkiss_success();
            }
        } else{
            SetTimer(10, easy_link_fail);
        }
    } else {
        
        if (mgmt_smart_type == MODE_EASYLINK) {
            smart_config_done(tssid, tpsk, CONFIG_BY_EASYLINK_MINUS);
            easylink_user_data_result(5, remote_ip);
        } else {
            smart_config_done(tssid, tpsk, CONFIG_BY_ALINK);
        }
    }
    sc_state = SC_STOPPED;
    p_sem = NULL;
    mico_rtos_deinit_semaphore(&easylink_sem);
    wiced_wifi_disable_monitor_mode();
    monitor_normal_filter_remove();
    wiced_wifi_remove_packet_filter(5);
    wiced_wifi_remove_packet_filter(6);
    wiced_wifi_remove_packet_filter(7);
    wiced_wifi_remove_packet_filter(8);
    thread_created = 0;
	if (es_key) {
		free(es_key);
		es_key = NULL;
		es_key_len = 0;
	}
    mico_rtos_delete_thread(NULL);
    return ;
}

int monitor_normal_filter(void)
{
	int ret;
	
	ret = wiced_wifi_add_packet_filter(0, &tods_filter);
    if (ret != WICED_SUCCESS)
        goto OUT;
    
    ret = wiced_wifi_enable_packet_filter(0);
    if (ret != WICED_SUCCESS)
        goto OUT;

    ret = wiced_wifi_add_packet_filter(1, &frmds_filter);
    if (ret != WICED_SUCCESS)
        goto OUT;
    
    ret = wiced_wifi_enable_packet_filter(1);
    if (ret != WICED_SUCCESS)
        goto OUT;

    ret = wiced_wifi_add_packet_filter(2, &beacon_filter);
    if (ret != WICED_SUCCESS)
        goto OUT;
    
    ret = wiced_wifi_enable_packet_filter(2);
    if (ret != WICED_SUCCESS)
        goto OUT;

    ret = wiced_wifi_add_packet_filter(3, &probe_rsp_filter);
    if (ret != WICED_SUCCESS)
        goto OUT;
    
    ret = wiced_wifi_enable_packet_filter(3);
    if (ret != WICED_SUCCESS)
        goto OUT;


    ret = wiced_wifi_add_packet_filter(4, &probe_req_filter);
    if (ret != WICED_SUCCESS)
        goto OUT;
    
    ret = wiced_wifi_enable_packet_filter(4);
    if (ret != WICED_SUCCESS)
        goto OUT;
OUT:
	return ret;
}

int instert_action_filter(int fd)
{
	
	int ret;
	
	ret = wiced_wifi_add_packet_filter(fd, &action_filter);
    if (ret != WICED_SUCCESS)
        goto OUT;
    
    ret = wiced_wifi_enable_packet_filter(fd);
    if (ret != WICED_SUCCESS)
        goto OUT;

OUT:
	return ret;
}

int monitor_normal_filter_remove()
{
	wiced_wifi_remove_packet_filter(0);
    wiced_wifi_remove_packet_filter(1);
    wiced_wifi_remove_packet_filter(2);
    wiced_wifi_remove_packet_filter(3);
    wiced_wifi_remove_packet_filter(4);
    
    return 0;
}

static int easy_link_start_cmd(void)
{
    wiced_result_t ret;

    memset(remote_ip, 0, sizeof(remote_ip));
    memset(tssid, 0, sizeof(tssid));
    memset(tpsk, 0, sizeof(tpsk));
    mgmt_smart_type = 0;
    max_channel = get_max_channel();
    if (config_mode & (MODE_EASYLINK|MODE_ALINK|MODE_AIRKISS)) {
        ret = monitor_normal_filter();
    }
    if (config_mode & MODE_EASYLINKV2) {
        wiced_wifi_add_packet_filter( 5, &a1_settings );
        wiced_wifi_add_packet_filter( 6, &a3_settings );
        wiced_wifi_add_packet_filter( 7, &beacon_a1_settings );
        wiced_wifi_add_packet_filter( 8, &beacon_a3_settings );
        wiced_wifi_enable_packet_filter( 5 );
        wiced_wifi_enable_packet_filter( 6 );
        wiced_wifi_enable_packet_filter( 7 );
        wiced_wifi_enable_packet_filter( 8 );
    }
    ret = wiced_wifi_enable_monitor_mode();
    if (ret != WICED_SUCCESS)
        goto OUT;

    sc_state = SC_SCANING;
    sc_channel = 0;
    
    memset(pkt_buffer, 0, sizeof(pkt_buffer));
    reset_recv_state();
    switch_channle();
    msleep(es_switch_interval);
OUT:

    return ret;
}

void easy_link_stop(void)
{
    mico_semaphore_t  *p = p_sem;

    easylink_break = 1;
    if (p_sem) {
        p_sem = NULL;
        mico_rtos_set_semaphore(p);
        
        while(thread_created) // wait real stopped
            msleep(10);
    }
    //easylink_break = 0;
}

void mxchip_easy_link_start(int seconds)
{
    autoconfig_start(seconds, MODE_AUTO);
}

void alink_config_start(int seconds)
{
    autoconfig_start(seconds, MODE_AUTO);
}

void airkiss_start(int seconds)
{
    autoconfig_start(seconds, MODE_AUTO);
}

void airkiss_stop()
{
    easy_link_stop();
}

void Airkiss_stop()
{
    easy_link_stop();
}

void easylink2_start(int with_data, int timeout)
{
    autoconfig_start(timeout, MODE_AUTO);
}

void easylink2_stop(void)
{
    easy_link_stop();
}

/* Auto choose config mode.*/
void autoconfig_start(int seconds, int mode)
{
    if (p_sem) {
        return;
    }

    if (thread_created != 0)
        return;

    easylink_break = 0;
    thread_created = 1;
    config_mode = mode;
    if (WICED_SUCCESS != mico_rtos_create_thread(NULL, WICED_NETWORK_WORKER_PRIORITY,  
                             "autoconf", easy_link_main_thread, 1248, (mico_thread_arg_t)(seconds*1000)))
        thread_created = 0;
}

void alink_config_stop(void)
{
    easy_link_stop();
}

int is_valid_channel(uint8_t *bssid, uint8_t channel)
{
    int i;

    for(i=0; i<ap_num; i++) {
        if (memcmp(bssid, ap_tbl[i].bssid, 6) == 0) {
            if (channel == ap_tbl[i].channel) {
                return 1;
            } else {
                return 0;
            }
        }
    }
    return 1;
}

static int check_sum_validate(void)
{
    int i;
    uint16_t sum = 0;

    for(i=0; i<pkt_buffer[0]; i++) {
        if (pkt_state[i] == PKT_STATE_NONE)
            received_byte_count--;
    }
    if (received_byte_count < pkt_buffer[0]) {
        print_buffer();
        return 0 ;
    }
    
    for(i=0; i<pkt_buffer[0]-2; i++) 
        sum += pkt_buffer[i];
    if (work_mode == MODE_EASYLINK) {
        if (sum>>8 != pkt_buffer[i++]) {
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "cksum err, sum %x, pkt_buffer[%d] %x\r\n", sum, i-1, pkt_buffer[i-1]);
            reset_pkt_buffer();
            return 0;
        }
        if ((sum&0x0ff) != pkt_buffer[i]) {
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "cksum err, sum %x, pkt_buffer[%d] %x\r\n", sum&0xff, i, pkt_buffer[i]);
            reset_pkt_buffer();
            return 0;
        }
    } else {
        if (((sum >> 7)&0x7f) != pkt_buffer[i++]) {
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "cksum err, sum %x, pkt_buffer[%d] %x\r\n", sum, i-1, pkt_buffer[i-1]);
            reset_pkt_buffer();
            return 0;
        }
        if ((sum&0x07f) != pkt_buffer[i]) {
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "cksum err, sum %x, pkt_buffer[%d] %x\r\n", sum&0xff, i, pkt_buffer[i]);
            reset_pkt_buffer();
            return 0;
        }
    }
    return 1;
}

unsigned char CRC8_Table(unsigned char *p, char counter)
{
    unsigned char crc8 = 0;
    for( ; counter > 0; counter--){
        crc8 = CRC8Table[crc8^*p];
        p++;
    }
    return(crc8);
}

static int data_validate(data_element_t *p, int len)
{
    uint8_t crc8 = 0;
    int i, index, ret = 0;;

    if (len < 3)
        goto EXIT;
    if (len > 6)
        goto EXIT;
    
    if (p[0].state == PKT_STATE_NONE)
        goto EXIT;
    if (p[1].state == PKT_STATE_NONE)
        goto EXIT;
    
    index = p[1].data*4;
    if ((len != 6) && (index + len - 2 != total_data_len)) // len < 6 must the last data pkt
        goto EXIT;
    
    if (index >= MAX_DATA_LENGTH)
        goto EXIT;

    if ((pkt_state[index+(len-3)] == DATA_VALID) && //last data is valid
        (pkt_state[index] == DATA_VALID)){// first data is valid
        ret = 1;
        goto EXIT;
    }
    
    for(i=1; i<len; i++) {
        if (p[i].state == PKT_STATE_NONE)
            goto EXIT;
        
        crc8 = CRC8Table[crc8^p[i].data];
    }

    crc8 &= 0x7F;

    if (crc8 != p[0].data) 
        goto EXIT;

    ret = 1;    
    for(i=2; i<len; i++) {
        pkt_state[index+i-2] = DATA_VALID;
        pkt_buffer[index+i-2] = p[i].data;
    }

    total_data_validate();
EXIT:
    
    return ret;
}

static int magic_code_valide(void)
{
    int i, j, maxnum, value;

    for(i=0; i<4; i++) {
        maxnum = magic_tbl[0][i].state;
        value = magic_tbl[0][i].data;
        for(j=1; j<5; j++) {
            if (magic_tbl[j][i].state > maxnum) {
                maxnum = magic_tbl[j][i].state;
                value = magic_tbl[j][i].data;
            }
        }
        if (maxnum == 0) {
            if (i != 0) {
                return 0;
            } else
                value = 0;//airkiss may not report 0 if total length less than 0x10
        }
        magic_tbl[0][i].data = value;
        magic_tbl[0][i].state = DATA_VALID;
    }

    valid_ssid = MAX_AP_TBL_NUM;
    for(i=0; i<MAX_AP_TBL_NUM; i++) {
        if (ap_tbl[i].ssid[0] == '\0') 
            break;

        if (ap_tbl[i].crc8 == (magic_tbl[0][2].data<<4) + magic_tbl[0][3].data) {
            valid_ssid = i;
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "ssid %s\r\n", 
                ap_tbl[valid_ssid].ssid);
            break;
        }
    }
    if (magic_tbl[0][0].data > 7) {
        system_debug_printf(SYSTEM_DEBUG_DEBUG, "totlen high %d --> 0\r\n", 
            magic_tbl[0][0].data);
        magic_tbl[0][0].data = 0;
    }
    total_data_len = (magic_tbl[0][0].data << 4) + magic_tbl[0][1].data;

    system_debug_printf(SYSTEM_DEBUG_DEBUG, "total data len %d\r\n", total_data_len);

    return 1;
}

static void insert_magic_code(int index, uint8_t value)
{
    int i;

    for(i=0;i<5; i++) {
        if (magic_tbl[i][index].state == 0) {
            magic_tbl[i][index].state = 1;
            magic_tbl[i][index].data = value;
            return;
        } else {
            if (magic_tbl[i][index].data == value) {
                magic_tbl[i][index].state++;
                return;
            }
        }
    }
}

static int prefix_validate(void)
{
    int i;
    uint8_t len, crc8;    
    int pswlen, ssidlen;
    
    for(i=0; i<4; i++) {
        if (prefix_tbl[i].state == 0)
            return 0;
    }
    len = (prefix_tbl[0].data<<4) + prefix_tbl[1].data;
    crc8 = (prefix_tbl[2].data<<4) + prefix_tbl[3].data;
    if (crc8 != CRC8Table[0^len]) {
        return 0;
    }

    system_debug_printf(SYSTEM_DEBUG_DEBUG, "prefix valid, key len %d\r\n", len);
    
    prefix_valid = 1;
    pswlen = (prefix_tbl[0].data<<4) + prefix_tbl[1].data;
    if (valid_ssid < MAX_AP_TBL_NUM) {
        ssidlen = strlen(ap_tbl[valid_ssid].ssid);
        if (ssidlen > 32)
            ssidlen = 32;

        len = (pswlen + 1 + ssidlen);
        if (total_data_len != len) {
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "total len %d===>%d\r\n", 
                total_data_len ,len);
            total_data_len = len;
        }
        system_debug_printf(SYSTEM_DEBUG_DEBUG, "ssid set index %d, len %d\r\n", 
                pswlen+1, ssidlen);
        for(i=0; i<ssidlen; i++) {
            pkt_state[i+pswlen+1] = DATA_VALID;
            pkt_buffer[i+pswlen+1]  = ap_tbl[valid_ssid].ssid[i];
        }
    }
    total_data_validate();
    return 1;
}



static int total_data_validate(void)
{
    int i;

    for(i=0;i<total_data_len;i++) {
        if (pkt_state[i] != DATA_VALID) {
            return 0;
        }
    }

    {
        system_debug_printf(SYSTEM_DEBUG_DEBUG, "success: %d \r\n", total_data_len);
        for(i=0;i<total_data_len;i++) {
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "%02x ", pkt_buffer[i]);
        }
        system_debug_printf(SYSTEM_DEBUG_DEBUG, "\r\n");
    }
    sc_state = SC_DONE;

    if (p_sem)
        mico_rtos_set_semaphore(p_sem);
    return 1;
}

static void mac_monitor_thread( uint32_t arg )
{
    int channel = 1;
	int interval = arg;
	int ret, i;

	if(interval < 100)
		interval = 100;
	else if (interval > 1000)
		interval = 1000;

	wiced_network_suspend();
    wiced_wifi_enable_monitor_mode();
    
    while(mac_monitor) {
		i = 0;
		while(1) {
	        ret = wiced_wifi_set_channel(channel);
			if (ret == WICED_SUCCESS) 
				break;
			i++;
			if (i>3) {
				wifi_reboot_event(4);
				break;
			}
			wifi_reboot_only();
			wiced_wifi_enable_monitor_mode();
		}
		channel++;
        if (channel == 14)
            channel = 1;
        msleep(interval);
    }

    wiced_wifi_disable_monitor_mode();
	wiced_network_resume();
    mico_rtos_delete_thread(NULL);
    return ;
}

void start_mac_detect_by_monitor(int switch_interval)
{
    mac_monitor = 1;
    wiced_rtos_create_thread(NULL, WICED_NETWORK_WORKER_PRIORITY,  
                             "maccapture", mac_monitor_thread, 1024, (void*)switch_interval);
}

void stop_mac_detect_by_monitor(void)
{
    mac_monitor = 0;
}
#ifdef MXCHIP_LIBRARY  

#define DOT11_MNG_VS_ID             221 /* d11 management Vendor Specific IE */
#define DOT11_MNG_SSID_ID           0 

#define ALI_OUI                     "\xD8\x96\xE0"   /** A Li OUI D8-96-E0*/

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


static wiced_bool_t wlu_is_ali_ie( uint8_t** wpaie, uint8_t** tlvs, uint32_t* tlvs_len )
{
    uint8_t* ie = *wpaie;

    /* If the contents match the ALI_OUI */
    if ( ( ie[1] >= (uint8_t) 6 ) &&
         ( memcmp( &ie[2], ALI_OUI, (size_t) 3 ) == 0 ) )
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

static int minus_data_decode(void)
{
    int i, j, k, len;
    uint8_t *p = p_minus_buffer;

    /* remove 0x7E */
    for(j=0, i=0;i<minus_pkt_len;i++) {
        if (p[i] == SSID_FLAG) {
            i++;
            if (p[i] == 0x01) {
                p[j++] = SSID_FLAG;
            } else if (p[i] == 0x02) {
                p[j++] = 0;
            } else {
                system_debug_printf(SYSTEM_DEBUG_DEBUG, "Invalid 0x7E\r\n");
                free(p_minus_buffer);
                p_minus_buffer = NULL;
                minus_pkt_num = 0;
                minus_pkt_len = 0;
                memset(minus_got_pkt, 0, sizeof(minus_got_pkt));
                return 0;
            }
        } else {
            p[j++] = p[i];
        }
    }
    /* transfer to HEX */
    len = j-1;// left the last byte.
    if (j > 8)
        k = 7;
    else {
        k = len;
    }
    for(j=0,i=0; i<len; i++) {
        pkt_buffer[j++] = p[i] | ((p[k]<<(7-(i%8))) & 0x80);
        if ((i % 8) == 6) {
            i++;
            k += 8;
            if (len < k)
                k = len;
        } 
    }
    
    /* ARC4 decrypt */
    arc4_decrypt(pkt_buffer, p, SSID_PASS, strlen(SSID_PASS), j);

    p[j] = 0;

    remote_ip[0]='#';
    //memcpy(&remote_ip[1], p, 4);
    remote_ip[1] =  p[3];
    remote_ip[2] =  p[2];
    remote_ip[3] =  p[1];
    remote_ip[4] =  p[0];
    p+=4;
    len = p[0];
    p++;
    memcpy(tssid, p, len);
    p+=len;
    memcpy(tpsk, p, j-5-len);
    free(p_minus_buffer);
    p_minus_buffer = NULL;
    minus_pkt_num = 0;
    minus_pkt_len = 0;
    memset(minus_got_pkt, 0, sizeof(minus_got_pkt));
    return 1;
}

/*return 1 if success*/
static int parse_ssid_ie(uint8_t *ssid, int len)
{
    int i, j, k;
    uint8_t flag = ssid[0], version, crc8, seq=ssid[1], tmp;
    uint8_t buf[32], rc4_data[32], *p;

    if ((len > 31) || (len < 3))
        return 0;
    system_debug_printf(SYSTEM_DEBUG_DEBUG, "Got easylink minus %d\r\n", len);

    version = (flag & 0x70) >> 4;
    if (version != 1) {// only support version 1 right now.
        system_debug_printf(SYSTEM_DEBUG_DEBUG, "Invalid version %d\r\n", version);
        return 0;
    }

    /*crc8 check*/
    crc8 = 0;
    for(i=1; i<len; i++) {
        crc8 = CRC8Table[crc8^ssid[i]];
    }
    if ((crc8 & 0x0F) != (flag & 0x0F)) {
        system_debug_printf(SYSTEM_DEBUG_DEBUG, "Invalid CRC %02x-%02x\r\n", crc8, flag);
        return 0;
    }
    
    tmp = (seq&0x0F0) >> 4;
    if (tmp > 7)
        return 0;
    seq = (seq & 0x0F) - 1; // tmp is total pkt number, seq is the current pkt number.
    if (seq >= tmp)
        return 0;
    
    len -= 2;// remove crc and seq.
    if (p_minus_buffer == NULL) { // first packet
        memset(minus_got_pkt, 0, sizeof(minus_got_pkt));
        p_minus_buffer = malloc(tmp * 29);
        minus_pkt_num = tmp;
    } else {
        if (tmp != minus_pkt_num) {// mismatch, remove older
            free(p_minus_buffer);
            minus_pkt_num = tmp;
            memset(minus_got_pkt, 0, sizeof(minus_got_pkt));
            p_minus_buffer = malloc(tmp * 29);
            minus_pkt_num = tmp;
        }
    }
    
    if (p_minus_buffer == NULL)
        return 0;

    
    if (minus_got_pkt[seq] == 0) {
        minus_got_pkt[seq] = 1;
        memcpy(&p_minus_buffer[seq*29], &ssid[2], len);
    }
    if (seq == (minus_pkt_num - 1))
        minus_pkt_len = seq * 29 + len;

    for(i=0;i<minus_pkt_num;i++) {
        if (minus_got_pkt[i] == 0)
            return 0;
    }
    
    return minus_data_decode();
}

#endif

/*PKT_TYPE_EL_START = 1,
    PKT_TYPE_EL_INDEX,
    PKT_TYPE_AL_START,
    PKT_TYPE_AL_INDEX,
    PKT_TYPE_AK_START,*/
static int pkt_type(int len)
{
    if (config_mode & MODE_EASYLINK) {
        if (((len == SPKT_START_FLAG1) || (len == SPKT_START_FLAG2) || (len == SPKT_START_FLAG3)) ||
			((len == PKT_START_FLAG1) || (len == PKT_START_FLAG2) || (len == PKT_START_FLAG3))) {
            work_mode |= MODE_EASYLINK;
            return PKT_TYPE_EL_START;
        } else if (((len > SPKT_SEQ_FLAG1) && (len <= SPKT_SEQ_FLAG2))||
        		    ((len > PKT_SEQ_FLAG1) && (len <= PKT_SEQ_FLAG2))) {
            work_mode |= MODE_EASYLINK;
            return PKT_TYPE_EL_INDEX;
        }
    } 
    
    if (config_mode & MODE_AIRKISS) {
        if ((len <= PKT_START_FLAG_MAX) && (len >= PKT_START_FLAG_MIN)) {
            work_mode = MODE_AIRKISS;
            return PKT_TYPE_AK_START;
        }
    }

    if (config_mode & MODE_ALINK) {
        if (len == ALINK_START_FLAG) {
            if (work_mode == 0)
                work_mode = MODE_ALINK;
            return PKT_TYPE_AL_START;
        } else if ((len >= ALINK_SEQ_FLAG1) && (len < ALINK_SEQ_FLAG2)) {
            if (work_mode == 0)
                work_mode = MODE_ALINK;
            return PKT_TYPE_AL_INDEX;
        }
    }

    return PKT_TYPE_DATA;
}

// return 0: goto EXIT;
// return 1: OK
static int easylink_parse(int len, int dir, int seq_num, int alink)
{
    int delta, last_offset;
    int offset;
    
    if (alink == 0) {
        if (len < 0xB0) {
            if ((len == 0xAA) || (len == 0xAB) || (len == 0xAC))
                len += 0x500;
            else
                goto EXIT;
        }
        if (len < 0x100)
            len += 0x450;
    }
    if (((alink == 0) && ((len == PKT_START_FLAG1) || (len == PKT_START_FLAG2) ||
        (len == PKT_START_FLAG3))) || ((alink == 1) && (len == ALINK_START_FLAG))){
        pkt_tbl[dir].last_index = 0;
        pkt_tbl[dir].last_data = len;
        pkt_tbl[dir].last_seq = seq_num;
        pkt_tbl[dir].pktnum++;
        goto EXIT;
    } else if (((alink == 0) && ((len > PKT_SEQ_FLAG1) &&
        (len <= PKT_SEQ_FLAG2))) || ((alink == 1) && 
        ((len >= ALINK_SEQ_FLAG1) && (len < ALINK_SEQ_FLAG2)))) {
        pkt_tbl[dir].last_index = 0;
        pkt_tbl[dir].last_data = len;
        pkt_tbl[dir].last_seq = seq_num;
        pkt_tbl[dir].pktnum++;
        goto EXIT;
    }
    if (alink == 0) {
        // length validation
        if (len < 0x100)
            goto EXIT;
        if (len > PKT_SEQ_FLAG2)
            goto EXIT;
    } else {
        if (len < ALINK_DATA_MIN)
            goto EXIT;
        if (len > ALINK_DATA_MAX)
            goto EXIT;
    }
    
    // sequence number validation
    delta = seq_num - pkt_tbl[dir].last_seq;
    if (delta <= 0) 
        goto EXIT;
    if (alink == 0) {
        if (delta > 4) // lost too many packets, must use start flag to restart receiving process.
            goto EXIT;
    } else {
        if (delta > 8)
            goto EXIT;
    }

    if (alink == 0)
        offset = (len >> 8) - 1;
    else
        offset = (len >> 7) - 2;
    
    if (((alink == 0) && ((pkt_tbl[dir].last_data == PKT_START_FLAG1) ||
        (pkt_tbl[dir].last_data == PKT_START_FLAG2) ||
        (pkt_tbl[dir].last_data == PKT_START_FLAG3))) ||
        ((alink == 1) && (pkt_tbl[dir].last_data == ALINK_START_FLAG))){ // last pkt is start flag
        //offset -= 1;
    } else if (((alink == 0) && (pkt_tbl[dir].last_data > PKT_SEQ_FLAG1) &&
        (pkt_tbl[dir].last_data <= PKT_SEQ_FLAG2)) || 
        ((alink == 1)&& ((pkt_tbl[dir].last_data >= ALINK_SEQ_FLAG1) &&
        (pkt_tbl[dir].last_data <= ALINK_SEQ_FLAG2)) )){
        if (alink == 0)
            offset = (pkt_tbl[dir].last_data - PKT_SEQ_FLAG1)*4 + offset;
        else
            offset = (pkt_tbl[dir].last_data - ALINK_SEQ_FLAG1 + 1)*8 + offset;
    } else {
        
        if (alink == 0)
            last_offset = (pkt_tbl[dir].last_data >> 8) - 1;
        else
            last_offset = (pkt_tbl[dir].last_data >> 7) - 2;
        if (offset > last_offset)
            delta = offset-last_offset;
        else {
            if (alink == 0)
                delta = offset+4-last_offset;
            else
                delta = offset+8-last_offset;
        }
        offset = delta + pkt_tbl[dir].last_index;
    }

	if (offset >= MAX_DATA_LENGTH) {
		goto EXIT;
	}
	
    if (pkt_state[offset] == PKT_STATE_NONE ) {
        if (alink == 0)
            pkt_buffer[offset] = (uint8_t)(len & 0xff);
        else
            pkt_buffer[offset] = (uint8_t)(len & 0x7f);
        if (dir == 0)
            pkt_state[offset] = PKT_STATE_STA;
        else
            pkt_state[offset] = PKT_STATE_APFWD;
        received_byte_count ++;
        if (pkt_state[0] > 0) {
            if (offset > pkt_buffer[0]) {
                system_debug_printf(SYSTEM_DEBUG_DEBUG, "len %d, offset %d\r\n", pkt_buffer[0], offset);
            }
        }
    } else {
        if (dir == 0) {
            if (alink == 0)
                pkt_buffer[offset] = (uint8_t)(len & 0xff);
            else
                pkt_buffer[offset] = (uint8_t)(len & 0x7f);
            pkt_state[offset] = PKT_STATE_STA;
        } else {
            if (pkt_state[offset] == PKT_STATE_APFWD) {
                if (alink == 0)
                    pkt_buffer[offset] = (uint8_t)(len & 0xff);
                else
                    pkt_buffer[offset] = (uint8_t)(len & 0x7f);
            }
        }
    }

    if (pkt_buffer[0] != 0) {
        if (received_byte_count>=pkt_buffer[0]) {
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "got all %d pkt\r\n", received_byte_count);
            if (check_sum_validate() == 1) {
                sc_state = SC_DONE;
                if (p_sem) {
                    system_debug_printf(SYSTEM_DEBUG_DEBUG, "put sem %p\r\n", p_sem);
                    mico_rtos_set_semaphore(p_sem);
                } else {
                    system_debug_printf(SYSTEM_DEBUG_DEBUG, "sem NULL\r\n");
                }
            } else {
                
                goto EXIT;
            }
        }
    }

    // update last pkt information
    pkt_tbl[dir].last_index = offset; 
    pkt_tbl[dir].last_data = len;
    pkt_tbl[dir].last_seq = seq_num;

    return 1;
    
EXIT: 
    return 0;
}


void easylinkv2_parse(int index, uint8_t *dest, int dir)
{
    int i;

    if (valid_dir == INVALID_DIR)
        goto EXIT;
    if ((valid_dir != BOTH_DIR) && (dir != valid_dir))
        goto EXIT;
    
    index -= 20;// udp more 20 bytes offset
    if (( index < 0 ) || ( index >= 512 ))
        return;
    index *= 2;
    if (index > MAX_DATA_LENGTH) {
		return;
    }
    if ( pkt_state[index] == PKT_STATE_NONE )
    {
        pkt_buffer[index] = dest[4];
        pkt_buffer[index + 1] = dest[5];
        if (dir == STA_SEND) {
            pkt_state[index] = PKT_STATE_STA;
            pkt_state[index+1] = PKT_STATE_STA;
        } else {
            pkt_state[index] = PKT_STATE_APFWD;
            pkt_state[index+1] = PKT_STATE_APFWD;
        }
        received_byte_count += 2;

        /* Check if we have all the data we need */
        if ( pkt_buffer[0] != 0 )
        {
            int a;
            int datalen;
            unsigned char *data = &pkt_buffer[2];

            datalen = pkt_buffer[0] + pkt_buffer[1];
            if (datalen % 2)
                datalen++;
            
            data += datalen;

            datalen += data[0]+2;
            if (datalen % 2)
                datalen += 3;
            else
                datalen += 2;
            for (a = 0; a < datalen; a+=2)
            {
                if (pkt_state[a] == 0)
                {
                    return;
                }
            }

            system_debug_printf(SYSTEM_DEBUG_DEBUG, "Easylink V2 success: ssid %d, key %d, total %d\r\n",
                pkt_buffer[0], pkt_buffer[1], datalen);
            sc_state = SC_DONE;
            mico_rtos_set_semaphore(p_sem);
        }
    }
EXIT:    
    return ;

}

static int airkiss_parse(int len, int dir)
{
    uint8_t hi;

    if (valid_dir == INVALID_DIR)
        goto EXIT;
    if ((valid_dir != BOTH_DIR) && (dir != valid_dir))
        goto EXIT;
    if (sc_state == SC_NEWFILTER) {
        if (len > 4) {
            sc_state = SC_MAGIC;//SC_MAGIC;
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "Magic state\r\n");
        } else {
            goto EXIT;
        }
    }
    
    hi = (len>>4) & 0x1F;
    if (sc_state == SC_MAGIC) { // magic code
        if (hi<4) {
            insert_magic_code(hi, (uint8_t)(len&0x0F));
        } else {
            if (1 == magic_code_valide()) {
                sc_state = SC_RECVING;
                system_debug_printf(SYSTEM_DEBUG_DEBUG, "Recv state\r\n");
            }
        }
        goto EXIT;
    } 
    
    if (hi<4)
        goto EXIT;
    
    if ((hi>=4) && (hi < 8)) { // prefix
        if (prefix_valid == 1)
            goto EXIT;
        hi -= 4;
        prefix_tbl[hi].data = (uint8_t)(len & 0x0F);
        prefix_tbl[hi].state = 1;
        if (hi == 3) {
            prefix_validate();
        }

        if (data_index > 2) {
            data_validate(data_tbl, data_index);
            memset(data_tbl, 0, sizeof(data_tbl));
            data_index = 0;
        }
    } else { // data
        if (hi < 0x10) { // sequence CRC / sequence number
            if (data_index > 1) {
				if (data_tbl[1].state == DATA_SET) {
	                if (data_tbl[1].data*4 + data_index-2 == total_data_len) {
						system_debug_printf(SYSTEM_DEBUG_DEBUG, "last pkt %d, index %d, total %d\r\n",
							data_tbl[1].data, data_index, total_data_len);
						data_validate(data_tbl, data_index);
	                }
	            }
                memset(data_tbl, 0, sizeof(data_tbl));
                data_index = 0;
            }
            data_tbl[data_index].state = DATA_SET;
            data_tbl[data_index].data = (uint8_t)(len & 0x7F);
            data_index++;
        } else {
            if (data_index < 2) {
                memset(data_tbl, 0, sizeof(data_tbl));
                data_index = 0;
                goto EXIT;
            }
            data_tbl[data_index].state = DATA_SET;
            data_tbl[data_index].data = (uint8_t)(len & 0xFF);
            data_index++;
            if (data_index == 6){
                data_validate(data_tbl, 6);
                memset(data_tbl, 0, sizeof(data_tbl));
                data_index = 0;
            } else if (data_tbl[1].state == DATA_SET) {
                if (data_tbl[1].data*4 + data_index-2 == total_data_len) {
					system_debug_printf(SYSTEM_DEBUG_DEBUG, "last pkt %d, index %d, total %d\r\n",
							data_tbl[1].data, data_index, total_data_len);
                    data_validate(data_tbl, data_index);
            }
        } 
    }
    }
    
EXIT: 
    return 0;
}

static int wifi_mgmt_parse(uint8_t *header, int len, int type)
{
    int     i;
    uint8_t *parse, *ie, *ali_ie;
    uint32_t     parse_len;

    if (PROBE_REQUEST_TYPE != type) { // beacon / probe response
        len -= 36;
        if (len <= 0)
            goto EXIT;
        parse = (uint8_t*)header + 36; // IE buffer header
        parse_len = len;
        while ( ( ie = wlu_parse_tlvs( parse, parse_len, (uint32_t) DOT11_MNG_VS_ID ) ) != 0 )
        {
            if ( wlu_is_ali_ie( &ie, &parse, &parse_len ) == WICED_TRUE )
            {
                ali_ie = ie;
                parse = (uint8_t*)header + 36; // IE buffer header
                parse_len = len;
                if ( ( ie = wlu_parse_tlvs( parse, parse_len, (uint32_t) DOT11_MNG_SSID_ID ) ) != 0 )
                {
                    memset(tssid, 0, sizeof(tssid));
                    memcpy(tssid, &ie[2], ie[1]);
                
                    if (0 == alink_get_tpsk(ali_ie, tpsk)) {
                        if (p_sem) {
                            mico_rtos_set_semaphore(p_sem);
                        }
                        mgmt_smart_type = MODE_ALINK;
                        sc_state = SC_DONE;
                    }
                }
                break;
            }
        }
        goto EXIT;
    }else {
        len -= 24;
        if (len <= 0)
            goto EXIT;
        parse = (uint8_t*)header + 24; // IE buffer header
        parse_len = len;
        if ( ( ie = wlu_parse_tlvs( parse, parse_len, (uint32_t) DOT11_MNG_SSID_ID ) ) != 0 )
        {
            len = ie[1]; // ssid len 
            ie += 2; // move to ssid
            if (len <= 2)
                goto EXIT;
            if (ie[0] != SSID_START) 
                goto EXIT;
            if (parse_ssid_ie(&ie[1], len-1)) {
                if (p_sem) {
                    mico_rtos_set_semaphore(p_sem);
                }
                mgmt_smart_type = MODE_EASYLINK;
                sc_state = SC_DONE;
            }
        }
        goto EXIT;
    }

EXIT:
    return 0; 
}

static void fill_ssid(char*apssid, int mode)
{
    int i, len;

    if (mode == MODE_EASYLINK) {
        len = strlen(apssid);
        pkt_buffer[1] = len;
        memcpy(&pkt_buffer[3], apssid, len);
        for(i=0;i<len;i++) {
            if (pkt_state[i+3] == PKT_STATE_NONE) {
                pkt_state[i+3] = PKT_STATE_APFWD;
                received_byte_count++;
            }
        }
    } 
}

static int smart_cfg_process( ieee80211_header_t* header, int len )
{
    int offset = SC_OFFSET_LEN, dir=0;
    unsigned int seq_num, delta;
    uint8_t* iv = NULL, *bssid, *initiator = NULL, *dest;
    int protect_type = 0, last_offset;
    int datatype;
	int rssi;
#ifdef MXCHIP_LIBRARY      
    int i;
    uint8_t         *parse, *ie, *ali_ie;
    int             parse_len;
#endif

#define IS_TKIP(data)  (data[1] == ((data[0] | 0x20) & 0x7f)) // copy from wireshark 1.10
#define IS_CCMP(data)  (data[2] == 0)

    if (mac_monitor == 1) {
#ifdef EXTRA_HEADER
		rssi = header->radio[RADIO_RSSI_INDEX];
#else
		rssi = 0;
#endif	
		if (PROBE_REQUEST_TYPE == header->type)
			initiator = header->address2;
		else {
			if ( (header->flags&DS_BITMASK) == TO_DS ) // STA send
		    {
		        initiator = header->address2;
		    }
		    else if ( (header->flags&DS_BITMASK) == FROM_DS )// AP forward
		    {
		        initiator = header->address3;
		    }
		}
		if (initiator)
        	mac_report_cb(initiator, rssi);
        goto EXIT;
    }

    if ((header->type & TYPE_MASK) == 0) {
        wifi_mgmt_parse((uint8_t*)header, len, header->type);
        goto EXIT;
    }
    
    seq_num = header->seq >> 4;
    
    if ( (header->flags&DS_BITMASK) == TO_DS ) // STA send
    {
        dir = STA_SEND;
        bssid = header->address1;
        initiator = header->address2;
        dest = header->address3;
    }
    else if ( (header->flags&DS_BITMASK) == FROM_DS )// AP forward
    {
        dir = AP_FWD;
        bssid = header->address2;
        initiator = header->address3;
        dest = header->address1;
    } else {
        goto EXIT;
    }

    
    
    if (is_valid_channel(bssid, current_channel) == 0) {
        goto EXIT;
    }

    if ((sc_state == SC_SCANING) || (valid_dir == INVALID_DIR)) {
        iv = header->data;
            
        if (header->type &0x80) {
            offset += 2; // QOS type
            iv += 2; // QOS field 2Bytes.
        }
        if (header->flags&PROTECT_FRAM) {
            if ((iv[3] & 0x20) == 0) {
                offset += 8;    // WEP
            } else if (IS_TKIP(iv)) {
            	if (IS_CCMP(iv)) // CCMP's IV may increased as a TKIP packet. drop this packet if can't distinguish tkip and ccmp.
					goto EXIT;
                offset += 20;   // TKIP
            } else if (IS_CCMP(iv)) {
                offset += 16;   // CCMP
            } else
                goto EXIT; // CCMP/TKIP
        }

        len -= offset; // the real UDP payload length.
        pkt_tbl[dir].pktnum++;
        pkt_num++;
        if ((config_mode & MODE_EASYLINKV2) &&
            ((memcmp(dest, cooee_upper_address_match, sizeof(cooee_upper_address_match)) == 0) ||
            (memcmp(dest, cooee_beacon_address_match, sizeof(cooee_beacon_address_match)) == 0))){
            datatype = PKT_TYPE_ELV2;
            work_mode = MODE_EASYLINKV2;
        } else {
            datatype = pkt_type(len);
        }
        if (datatype == PKT_TYPE_DATA) {
            goto EXIT;
        }

        if (pkt_tbl[dir].valid == 0) {
            pkt_tbl[dir].last_data = len;
            memcpy(pkt_tbl[dir].bssid, bssid, 6);
            memcpy(pkt_tbl[dir].initiator, initiator, 6);
            pkt_tbl[dir].offset = offset;
            pkt_tbl[dir].valid = 1;
            
            apssid = find_ssid_by_bssid(pkt_tbl[dir].bssid);
            
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "ssid %s\r\n", apssid);
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "DIR %d, Channel %d, datatype %d\r\n", dir, current_channel, datatype);
			if (datatype == PKT_TYPE_ELV2)
				sc_state = SC_NEWFILTER;
            goto EXIT;
        }

        if (memcmp(pkt_tbl[dir].initiator, initiator, 6) != 0) { // src address wrong?
            goto EXIT;
        }
        if (memcmp(pkt_tbl[dir].bssid, bssid, 6) != 0) { // bssid wrong?
            goto EXIT;
        }

        if (pkt_tbl[dir].offset != offset) {// offset must wrong
            pkt_tbl[dir].valid = 0;
            pkt_tbl[dir].last_data = 0;
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "wrong type\r\n");
            goto EXIT;
        }
        
        if ((sc_state == SC_SCANING) /*&& (pkt_tbl[dir].last_data != len) */)
        { // switch to filter state
            sc_state = SC_NEWFILTER;
            
            system_debug_printf(SYSTEM_DEBUG_DEBUG, "filter\r\n");
        }
        
        goto EXIT;
    }

    if (pkt_tbl[dir].valid == 0)
        goto EXIT;
    
    if (memcmp(pkt_tbl[dir].initiator, initiator, 6) != 0) { // src address wrong?
        goto EXIT;
    }
    if (memcmp(pkt_tbl[dir].bssid, bssid, 6) != 0) { // bssid wrong?
        goto EXIT;
    }
    len -= pkt_tbl[dir].offset;

    if (work_mode == MODE_EASYLINKV2) {
        if ((memcmp(dest, cooee_upper_address_match, sizeof(cooee_upper_address_match)) == 0))
            easylinkv2_parse(len, dest, dir);
    } else if (work_mode == MODE_EASYLINK) {
        easylink_parse(len, dir, seq_num, 0);
    } else if (work_mode == MODE_ALINK) {
        easylink_parse(len, dir, seq_num, 1);
    } else if (work_mode == MODE_AIRKISS) {
        airkiss_parse(len, dir);
    } 
    
EXIT:
    
    return WICED_SUCCESS;
}

wiced_result_t host_network_process_raw_packet( wiced_buffer_t buffer, wiced_interface_t interface )
{
    int len;
    ieee80211_header_t* header;
    
    if (monitor_ignore)
        goto EXIT;
    
    len = host_buffer_get_current_piece_size(buffer);
    header = (ieee80211_header_t*) host_buffer_get_current_piece_data_pointer( buffer );


	if (PROBE_REQUEST_TYPE == header->type) {
		probe_request_rx_cb(header, len);
	} 
    if ((sniffer_callback != NULL) && (detect_easylink_in_monitor == 0)) {
        sniffer_callback((uint8_t*)header, len);
        goto EXIT;
    }
    
    if ((mac_monitor == 1) || ((sc_state != SC_STOPPED) && (sc_state != SC_DONE) && (sc_state != SC_FAIL) )) {
        smart_cfg_process( header, len );
    }

EXIT:    
    host_buffer_release(buffer, WICED_NETWORK_RX);
    return WICED_SUCCESS;
}

/* make compiler happy */
wiced_result_t airkiss_process_raw_packet( wiced_buffer_t buffer)
{
    return WICED_SUCCESS;
}

/* */
int OpenEasylink_withKey(int timeout, uint8_t *key, int key_len)
{
	wlan_disconnect();
	
	if ((key_len > 0) && (key!= NULL)) {
		es_key_len = key_len;
		es_key = (uint8_t*)malloc(key_len);
		if (es_key == NULL)
			return -1;
		
		memcpy(es_key, key, key_len);
	}
	
	autoconfig_start(timeout, MODE_EASYLINK|MODE_EASYLINKV2);

	return 0;
}

int OpenAutoConfig(int timeout, int mode)
{
    wlan_disconnect();
    autoconfig_start(timeout, mode);
    return 0;
}

int CloseAutoConfig(void)
{
    alink_config_stop();
    return 0;
}

void set_sniffer_channel(int32_t nchannel)
{
    wiced_wifi_set_channel(nchannel);
}

int set_promiscuous_mode(void (*promiscuous_cb)(uint8_t *data,
						 int len)) 
{
    sniffer_callback = promiscuous_cb;
    wiced_wifi_set_channel(1);
    wiced_wifi_enable_monitor_mode();
    return 0;
}

/*WIFI monitor mode APIs*/
static int max_filter_fd=-1;
typedef void (*monitor_cb_t)(uint8_t*data, int len);
enum {
	WLAN_FILTER_RX_BEACON,
	WLAN_FILTER_RX_PROBE_REQ,
	WLAN_FILTER_RX_PROBE_RES,
	WLAN_FILTER_RX_ACTION,
	WLAN_FILTER_RX_MANAGEMENT,
	WLAN_FILTER_RX_DATA,
	WLAN_FILTER_RX_MCAST_DATA,

	WLAN_FILTER_MAX,
};

int mico_wlan_monitor_rx_type(int type)
{
	int ret;
	int fd;
	const wiced_packet_filter_settings_t *p;

	fd = type;
	switch(type) {
	case WLAN_FILTER_RX_BEACON:
		p = &beacon_filter;
		break;
	case WLAN_FILTER_RX_PROBE_REQ:
		p = &probe_req_filter;
		break;
	case WLAN_FILTER_RX_PROBE_RES:
		p = &probe_rsp_filter;
		break;
	case WLAN_FILTER_RX_ACTION:
		p = &action_filter;
		break;
	case WLAN_FILTER_RX_MANAGEMENT:
		p = &management_filter;
		break;
	case WLAN_FILTER_RX_DATA:
		p = &data_filter;
		break;
	case WLAN_FILTER_RX_MCAST_DATA:
		wiced_wifi_add_packet_filter( fd, &tods_filter );
		wiced_wifi_enable_packet_filter( fd++ );
        wiced_wifi_add_packet_filter( fd, &frmds_filter );
		wiced_wifi_enable_packet_filter( fd++ );
		wiced_wifi_add_packet_filter( fd, &mcast1_settings );
		wiced_wifi_enable_packet_filter( fd++ );
        wiced_wifi_add_packet_filter( fd, &mcast2_settings );
		wiced_wifi_enable_packet_filter( fd++ );
		max_filter_fd = fd;
		return 0;
		
	default :
		return 0;
	}
	
	wiced_wifi_add_packet_filter( fd, p );
    wiced_wifi_enable_packet_filter( fd );
	if (fd > max_filter_fd)
		max_filter_fd = fd;
	return 0;
}

int mico_wlan_start_monitor(int mode)
{
	monitor_ignore = 0;
	mico_wlan_monitor_rx_type(WLAN_FILTER_RX_MANAGEMENT);
	mico_wlan_monitor_rx_type(WLAN_FILTER_RX_MCAST_DATA);
	return wiced_wifi_enable_monitor_mode();
}

int mico_wlan_stop_monitor(void)
{
	int fd;
	sniffer_callback = NULL;
	monitor_ignore = 1;
	for(fd = 0; fd < max_filter_fd; fd++)
		wiced_wifi_remove_packet_filter(fd);
	max_filter_fd = -1;
	return wiced_wifi_disable_monitor_mode();
}

int mico_wlan_set_channel(int channel)
{
	if (detect_easylink_in_monitor == 1)
		return 0;
	
	return wiced_wifi_set_channel(channel);
}

void mico_wlan_register_monitor_cb(monitor_cb_t fn)
{
	sniffer_callback = fn;
}


static void easylink_aws_thread( uint32_t arg )
{
	int ret;
	mico_semaphore_t _easylinksem;
	int choose_dir;
	
	detect_easylink_in_monitor = 1;
	msleep(1000);
			
    if (pkt_tbl[AP_FWD].valid) {
        if (pkt_tbl[STA_SEND].valid) {
            if (pkt_tbl[AP_FWD].pktnum > pkt_tbl[STA_SEND].pktnum) {
                choose_dir = AP_FWD;
            } else {
                choose_dir = STA_SEND;
            }
        } else {
            choose_dir = AP_FWD;
        }
    } else {
        choose_dir = STA_SEND;
    }
    

    valid_dir = choose_dir;
	if (work_mode & MODE_EASYLINKV2)
		work_mode = MODE_EASYLINKV2;
	else
		work_mode = MODE_EASYLINK;
    system_debug_printf(SYSTEM_DEBUG_DEBUG, "Choose DIR %d, %d-%d, mode %x, %d\r\n", choose_dir, 
        pkt_tbl[STA_SEND].pktnum, pkt_tbl[AP_FWD].pktnum, (work_mode), mico_get_time());
	
	mico_rtos_init_semaphore(&_easylinksem, 1);
	p_sem = &_easylinksem;
	ret = mico_rtos_get_semaphore(&_easylinksem, MAX_GET_SSID_TIME);
	p_sem = NULL;
	if (ret == 0) {

		printf("easylink success, callback\r\n");
		if (work_mode == MODE_EASYLINKV2) {
            easylinkv2_success();
        }  else if (work_mode == MODE_EASYLINK) {
            easy_link_success();
        } 
	} else {
		
	}
	printf("easylink  callback done\r\n");
	mico_rtos_deinit_semaphore(&_easylinksem);

	_easylink_start = 0;
	detect_easylink_in_monitor = 0;
	system_debug_printf(SYSTEM_DEBUG_DEBUG, "set easylink in monitor to 0\r\n");
	mico_rtos_delete_thread(NULL);
}

/* callback used by AWS, switch to easylink mode */
void monitor_switch_easylink(uint8_t bssid[6],
		uint8_t src[6], uint8_t channel,
		int len, char ssid[33])
{
	if (_easylink_start == 0) {
		_easylink_start = 1;

		memset(pkt_buffer, 0, MAX_DATA_LENGTH);
		reset_recv_state();
		
		config_mode = MODE_EASYLINK|MODE_EASYLINKV2;
		work_mode = 0;
		sc_state = SC_SCANING;
		ap_num = 0;
		mico_rtos_create_thread(NULL, 5, "easylink", easylink_aws_thread, 1400, 0);
	}
}

