#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include "Util.h"

using namespace ChibiOS_K3;

// TODO(S3+): back these with a real RTC / persistent time source.
extern "C" size_t k3_heap_remaining(void);

uint32_t Util::available_memory(void)
{
    return (uint32_t)k3_heap_remaining();
}

void Util::set_hw_rtc(uint64_t time_utc_usec)
{
    (void)time_utc_usec;
}

uint64_t Util::get_hw_rtc() const
{
    return 0;
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
