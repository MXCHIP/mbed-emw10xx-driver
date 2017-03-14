#ifndef __MXCHIPWNET_H__
#define __MXCHIPWNET_H__
#include "stdlib.h"
#include "string.h"
#include "mico_wlan.h"

//#define AF_INET 2
//#define SOCK_STREAM 1
//#define SOCK_DGRM 2
//#define IPPROTO_TCP 6
//#define IPPROTO_UDP 17
//#define SOL_SOCKET   1
//#define INADDR_ANY   0
//#define INADDR_BROADCAST 0xFFFFFFFF


#define u8 unsigned char
#define u16 unsigned short
#define u32 unsigned int

//typedef ULONG       time_t;

typedef enum
{
    IRQ_TRIGGER_RISING_EDGE  = 0x1, /* Interrupt triggered at input signal's rising edge  */
    IRQ_TRIGGER_FALLING_EDGE = 0x2, /* Interrupt triggered at input signal's falling edge */
    IRQ_TRIGGER_BOTH_EDGES   = IRQ_TRIGGER_RISING_EDGE | IRQ_TRIGGER_FALLING_EDGE,
} gpio_irq_trigger_t;

typedef void (*gpio_irq_handler_t)( void* arg);

typedef struct  _component_baseinfo_st{ 
  char protocol_ver[32]; 
  char hw_ver[32]; 
  char sw_ver[32]; 
  char product_id[32]; 
  char product_date[32]; 
 } COMPONENT_BASEINFO_st; 
 
typedef struct _component_WIFIinfo_st{ 
  COMPONENT_BASEINFO_st baseinfo; 
  char swlst[32];//eg:softap/wps 
  char securityt[32];//eg:wpa/wep/wpa2 
  char wkmode[32];//eg: b/g/n/a/bg/gn/bgn 
} COMPONENT_WIFIINFO_st; 
 
typedef struct _component_BTinfo_st{ 
  COMPONENT_BASEINFO_st baseinfo; 
  int MFIflg;//if has MFI with CP 1-has ,0-none  
} COMPONENT_BTINFO_st; 

typedef struct _component_NFCinfo_st{ 
    COMPONENT_BASEINFO_st baseinfo;  
    char nfc_model_no[32];//product model number  
    char nfc_id[32];//product id  
    char tag_type;//tag0:0��tag1:1��tag2:2��tag3:3  
    char nfc_mode;//passive��0��active��1��bi_direction��2��  
} COMPONENT_NFCINFO_st; 


//typedef long time_t; /* ʱ��ֵtime_t Ϊ�����͵ı���*/; 

typedef  struct  _timeval_st{ 
    long tv_sec; /*1970��1��1�����������*/ 
    long tv_hmsec; /* 1970��1��1������İٺ�����*/ 
}TIMEVAL_st; 

struct mxchip_mallinfo {
  int num_of_chunks;  /* number of free chunks */
  int total_memory;  /* maximum total allocated space */
  int allocted_memory; /* total allocated space */
  int free_memory; /* total free space */
};

 
typedef  struct  _ApList_str  
{  
    char ssid[32];  
    char ApPower;  // min:0, max:100
}ApList_str; 


typedef  struct  _UwtPara_str  
{  
  char ApNum;       //AP number
  ApList_str * ApList; 
} UwtPara_str;  


typedef  struct  _ApList_adv  
{  
    char ssid[32];  
    char ApPower;  // min:0, max:100
    char bssid[6];
    char channel;
    wlan_sec_type_t security;
}ApList_adv_t; 


typedef  struct  _UwtPara_adv  
{  
  char ApNum;       //AP number
  ApList_adv_t * ApList; 
} UwtPara_adv_t;  


typedef struct _uart_get_str 
{ 
    int BaudRate;    //The baud rate 
    char number;     //The number of data bits 
    char parity;     //The parity(0: none, 1:odd, 2:evn, default:0)
    char StopBits;      //The number of stop bits 
    char FlowControl;    //support flow control is 1 
}uart_get_str;

typedef struct _uart_set_str 
{ 
    char UartName[8];    // the name of uart 
    int BaudRate;    //The baud rate 
    char number;     //The number of data bits 
    char parity;     //The parity(default NONE) 
    char StopBits;      //The number of stop bits 
    char FlowControl;    //support flow control is 1 
}uart_set_str;

typedef struct _uart_str
{
	char baudrate;     //The baud rate, 0:9600, 1:19200, 2:38400, 3:57600, 4:115200, 5:230400, 6:460800, 7:921600 
	char databits;      //0:8, 1:9
	char parity;       //The parity(default NONE)  0:none, 1:even parity, 2:odd parity
	char stopbits;       //The number of stop bits ,  0:1, 1:0.5, 2:2, 3:1.5
} uart_str; 




typedef struct _sta_ap_state{
    int is_connected;
    int wifi_strength;
    u8  ssid[32];
    u8  bssid[6];
    int channel;
}sta_ap_state_t;

struct wifi_InitTypeDef
{
	u8 wifi_mode;		// adhoc mode(1), AP client mode(0), AP mode(2)
	u8 wifi_ssid[32];
	u8 wifi_key[32];
};

enum {
    WIFI_CHANEEL_1_11 = 0,
    WIFI_CHANEEL_1_13,
    WIFI_CHANEEL_1_14,
};


enum {
    WIFI_ERROR_INIT = 1,
    WIFI_ERROR_NOGW,
    WIFI_ERROR_NOBUS_CREDIT,
};

typedef enum {
	MXCHIP_SUCCESS = 0,
	MXCHIP_FAILED = -1,
	MXCHIP_8782_INIT_FAILED = -2,
	MXCHIP_SYS_ILLEGAL = -3,
    MXCHIP_WIFI_JOIN_FAILED = -4,

	MXCHIP_WIFI_UP = 1,
	MXCHIP_WIFI_DOWN,

    MXCHIP_UAP_UP,
    MXCHIP_UAP_DOWN,
} 	MxchipStatus;

/* Upgrade iamge should save this table to flash */
#pragma pack(1)
typedef struct _boot_table_t {
	u32 start_address; // the address of the bin saved on flash.
	u32 length; // file real length
	u8 version[8];
	u8 type; // B:bootloader, P:boot_table, A:application, D: 8782 driver
	u8 upgrade_type; //u:upgrade, 
	u8 reserved[6];
}boot_table_t;
#pragma pack()

typedef struct _lib_config_t {
    int wifi_channel; // 0:USA(1~11), 1:China(1~13), 2:JP(1~14)
}lib_config_t;


//typedef int (*timer_handler_t)( void* arg );

enum {
	SLEEP_UNIT_MS = 0,
	SLEEP_UNIT_BEACON = 1,
};

// upgraded image should saved in here
#define NEW_IMAGE_ADDR 0x08060000

#define FD_UART         16 // MEMP_NUM_NETCONN


typedef struct fd_set_ {
      unsigned char fd_bits [(FD_SETSIZE+7)/8];
    } fd_set_t;

typedef void * ssl_t;

ssl_t ssl_connect(int fd, int calen, char*ca);
ssl_t ssl_accept(int fd);
int ssl_send(ssl_t ssl, char *data, int len);
int ssl_recv(ssl_t ssl, char *data, int len);
int ssl_close(ssl_t ssl);

extern void lib_config(lib_config_t* conf);
extern void set_cert(const char *cert_pem, const char*private_key_pem);

extern int SelectSupport(void);

#if 0
extern int socket(int domain, int type, int protocol);
extern int setsockopt(int sockfd, int level, int optname,const void *optval, socklen_t optlen);
extern int getsockopt(int sockfd, int level, int optname,const void *optval, socklen_t *optlen);

extern int bind(int sockfd, const struct sockaddr_t *addr, socklen_t addrlen);
extern int connect(int sockfd, const struct sockaddr_t *addr, socklen_t addrlen);
extern int listen(int sockfd, int backlog);
extern int accept(int sockfd, struct sockaddr_t *addr, socklen_t *addrlen);
extern int select(int nfds, fd_set_t *readfds, fd_set_t *writefds, fd_set_t *exceptfds, struct timeval *timeout);
extern int send(int sockfd, const void *buf, unsigned int len, int flags);
extern int  sendto(int  sockfd,  const  void  *buf,  unsigned int  len,  int  flags,const  struct  sockaddr_t  *dest_addr, 
				socklen_t addrlen);
extern int recv(int sockfd, void *buf, unsigned int len, int flags);
extern int recvfrom(int  sockfd,  void  *buf,  unsigned int  len,  int  flags,struct  sockaddr_t  *src_addr,  socklen_t 
					*addrlen);
extern int read(int sockfd, void *buf, unsigned int len); 
extern int write(int sockfd, void *buf, unsigned int len); 
extern int close(int fd);
#endif
extern int GetUartPara (char *uart_name, uart_get_str * uart_para); 
extern int GetUartNum(char *uartname); 
extern int SetUartPara (uart_set_str *puartpara); 
extern int OpenUART(char*  uart_name); 

extern int mxchipInit(void);
extern int StartNetwork(network_InitTypeDef_st* pNetworkInitPara);
// callback function when Station connected, return the ap info and PMK for WPA security.
extern void connected_ap_info(apinfo_adv_t *ap_info, char *key, int key_len);
extern int StartAdvNetwork(network_InitTypeDef_adv_st* pNetworkInitPara);


extern int sta_disconnect(void);
extern int uap_stop(void);


extern void mxchipTick(void);

extern int StartScan(int interval);
extern int StopScan(void);

int ReallocIP(void);

int gethostbyname(const u8 * name, u8 * ip_addr, u8 ipLength);
u32 dns_request(char *hostname);// start a DNS request. return 0=start dns req, 0xffffffff=fail, other=the IP address of this DNS

extern int SetTimer(unsigned long ms, void (*psysTimerHandler)(void));


/* User provide watch dog callback function */
extern void WatchDog(void);

extern int FlashGetInfo(int *flashadd,int len);
extern int FlashRead(int flashadd,char *pbuf,int len);
extern int FlashWrite(int flashadd,char *pbuf,int len) ;
extern int FlashErase(int flashadd, int erase_bytelen); 
extern MxchipStatus newimage_write(int offset , int len , char *pbuf);

extern int sleep(int seconds);
extern int msleep(int mseconds);

/* Convert an ip address string in dotted decimal format to  its binary representation.*/
//extern u32 inet_addr(char *s);

/* Convert a binary ip address to its dotted decimal format. 
PARAMETER1 's':  location to place the dotted decimal string.  This must be an array of at least 16 bytes.
PARAMETER2 'x':  ip address to convert.

RETURN VALUE:  returns 's' .
*/
//extern char *inet_ntoa( char *s, u32 x );


extern void enable_ps_mode(int unit_type, int unitcast_ms, int multicast_ms);

extern void disable_ps_mode(void);

extern void system_reload(void);

int wlan_disconnect(void);


int OpenConfigmodeWPS(int timeout);
int CloseConfigmodeWPS(void);

int OpenEasylink(int timeout);
int CloseEasylink(void);


int OpenBT(int timeout);
int CloseBT(void);
int SetBTName(char *name);

int SetBTpin(char *pin, int enable);

int SetBTboundleID(char*boundleID);
int SetBTseedID(char* seedID);
int SetBTsdkProto(char*sdkproto);

int GetBTArg(char *name, char *pin, int *enable_pin, char *boundleID);
int SetBT_SearchAppPara(char *boundleID, char *seedId, char *SdkProtocolToken);

/* devlist return mac[4][6], return 4 mac address, last_dev_index is the last connected dev index (0~3)*/
int GetBTDevlist(char *devlist, u8 *last_dev_index);

int CheckComponentBT(COMPONENT_BTINFO_st *pst);


int CheckComponentWIFI(COMPONENT_WIFIINFO_st *pst);

int CheckComponentNFC(COMPONENT_NFCINFO_st * st);
int OpenConfigmodeNFC(int timeout);
int CloseConfigmodeNFC(void);


long user_time(TIMEVAL_st *t);

void ps_enable(void); 
void ps_disable(void); 

int rand(void); 

void set_tcp_keepalive(int num, int seconds);
void get_tcp_keepalive(int *num, int *seconds);
int get_tcp_clients(void);
void memory_status(int *total_free, int *max_len);
void get_malloc_list(void (*debug_mem)(u32 p, int len));
void malloc_list_mem(u8 *memory_tbl, int size);

// user provide callback function 
void WatchDog(void);
void WifiStatusHandler(int status);
void RptConfigmodeRslt(network_InitTypeDef_st *nwkpara);
void NetCallback(IPStatusTypedef *pnet);
void ApListCallback(ScanResult *pApList);


int wifi_power_down(void);
int wifi_power_up(void);
void wifi_reboot_event(int type);

int wlan_driver_version( char* version, u8 length );
int wlan_set_max_station_num(int number);

struct mxchip_mallinfo* mico_memory_info(void);

#endif

