#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include <AP_HAL/system.h>
#include <ch.h>
#include <stdarg.h>
#include "hwdef/boot/trace.h"

/*
  AP_HAL:: system services for the AM67/K3 board, backed by ChibiOS/RT time.

  The systick runs at CH_CFG_ST_FREQUENCY (1000 Hz -> 1 tick == 1 ms). The 32-bit
  helpers read the system time counter directly (chVTGetSystemTimeX, no lock);
  the 64-bit helpers use the wrap-free timestamp (CH_CFG_USE_TIMESTAMP).
*/

namespace AP_HAL
{

void init()
{
}

void panic(const char *errormsg, ...)
{
    va_list ap;

    trace_printf("AP-K3: PANIC: ");
    va_start(ap, errormsg);
    trace_vprintf(errormsg, ap);
    va_end(ap);
    trace_printf("\n");

    // No console guaranteed here; halt honestly.
    while (true) {
    }
}

uint32_t micros()
{
    return (uint32_t)TIME_I2US(chVTGetSystemTimeX());
}

uint32_t millis()
{
    return (uint32_t)TIME_I2MS(chVTGetSystemTimeX());
}

uint16_t micros16()
{
    return (uint16_t)(micros() & 0xFFFFU);
}

uint16_t millis16()
{
    return (uint16_t)(millis() & 0xFFFFU);
}

uint64_t micros64()
{
    return (uint64_t)chVTGetTimeStamp() * 1000000ULL / CH_CFG_ST_FREQUENCY;
}

uint64_t millis64()
{
    return (uint64_t)chVTGetTimeStamp() * 1000ULL / CH_CFG_ST_FREQUENCY;
}

} // namespace AP_HAL

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
