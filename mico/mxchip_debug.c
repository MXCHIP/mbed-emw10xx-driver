#include <stdarg.h>

#include "mico_rtos.h"
#include "mxchip_debug.h"

debug_printf pPrintffunc = NULL;
int debug_level;


/* Enable library printf debug msg to callback function. 
 * debug level is from 0 to 7. 0 is the highest debug level. 
 * only system debug msg level is lower than level, the callbcak is called.
 */
void system_debug_enable(int level, debug_printf callback)
{
    debug_level = level;
    pPrintffunc = callback;
}




