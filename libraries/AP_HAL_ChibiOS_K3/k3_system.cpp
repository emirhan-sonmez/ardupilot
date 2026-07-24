#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include <AP_HAL/system.h>

/*
  AP_HAL:: system services for the AM67/K3 board.

  M2 bring-up: the time sources are stubs returning 0. They exist to satisfy the
  link and are NOT functional. S3 wires them to the ChibiOS system time
  (chVTGetSystemTimeX + TIME_I2MS/TIME_I2US), which needs ch.h on the include
  path. panic() honestly halts (no console yet); S3 prints + reboots.
*/

namespace AP_HAL {

void init()
{
}

void panic(const char *errormsg, ...)
{
    (void)errormsg;
    // Honest halt: spin forever. Nothing to print to yet.
    while (true) { }
}

uint32_t micros()
{
    return 0;
}

uint32_t millis()
{
    return 0;
}

uint16_t micros16()
{
    return 0;
}

uint16_t millis16()
{
    return 0;
}

uint64_t micros64()
{
    return 0;
}

uint64_t millis64()
{
    return 0;
}

} // namespace AP_HAL

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
