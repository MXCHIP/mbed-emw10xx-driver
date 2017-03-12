#ifndef __MXCHIP_DEBUG_H__
#define __MXCHIP_DEBUG_H__

typedef int (*debug_printf)( char*msg, ... );

enum {
    SYSTEM_DEBUG_ERROR = 1,
    SYSTEM_DEBUG_DEBUG = 2,
    SYSTEM_DEBUG_INFO  = 3,
};

extern debug_printf pPrintffunc;
extern int debug_level;

#define system_debug_printf(level, ...) \
do {\
    if ((level <= debug_level) && (pPrintffunc != NULL))\
        pPrintffunc(__VA_ARGS__);\
}while(0)


#define cmd_printf(...) do{\
                                if (xWriteBufferLen > 0) {\
                                    snprintf(pcWriteBuffer, xWriteBufferLen, __VA_ARGS__);\
                                    xWriteBufferLen-=strlen(pcWriteBuffer);\
                                    pcWriteBuffer+=strlen(pcWriteBuffer);\
                                }\
                             }while(0)


void system_debug_enable(int level, debug_printf callback);



#endif
