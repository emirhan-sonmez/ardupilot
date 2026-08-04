#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_K3_Namespace.h"

/*
  Util for the AM67/K3 ChibiOS backend.

  NOTE (S2): minimal implementation. The hardware RTC hooks are stubs for now;
  S3+ wires them to a real time source. Only the interface's pure-virtual
  methods are implemented here — the rest use AP_HAL::Util defaults.
*/
class ChibiOS_K3::Util : public AP_HAL::Util
{
public:
    /*
      Real free heap, not AP_HAL::Util's hardcoded 4096. AP_NavEKF3 refuses to
      start unless this reports enough room for its cores, so leaving the base
      implementation in place silently disables EKF3 on a board with 14 MB of
      DDR. See k3_heap_remaining() in k3_syscalls.c.
    */
    uint32_t available_memory(void) override;

    void set_hw_rtc(uint64_t time_utc_usec) override;
    uint64_t get_hw_rtc() const override;
};
