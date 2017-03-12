#ifdef MICO
#define USE_SMART_CONFIG
#endif


void mxchip_easy_link_start(int seconds);
void easy_link_stop(void);
void alink_config_start(int seconds);
void alink_config_stop(void);
void easylink2_start(int with_data, int timeout);
void easylink2_stop(void);
void airkiss_start(int seconds);
void Airkiss_stop(void);
