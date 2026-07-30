#pragma once

#include <stdint.h>

/*
  Q-34 A/B switch, 2026-07-30. 1 = the bench passthrough owns RCOutput
  channels 0-5 exclusively and AP_Motors' competing writes are dropped in the
  HAL; 0 = the old DR-012 "run last every tick" behaviour, which loses the
  shadow-register race once per PWM period (see RCOutput.h,
  set_exclusive_mask()). Kept as a compile-time switch so the fix can be
  A/B'd against the dancing baseline on hardware without reverting code.
*/
#ifndef PT_EXCLUSIVE_OUTPUTS
#define PT_EXCLUSIVE_OUTPUTS 1
#endif

namespace ChibiOS_K3 {
    // All six real channels, not just the four this module mixes: SRV_Channels
    // ::push() writes every channel every tick, so ch4/ch5 need the same
    // protection to stay at their safe boot idle. bench_passthrough_update()
    // re-asserts all six.
    constexpr uint32_t PT_EXCLUSIVE_MASK = PT_EXCLUSIVE_OUTPUTS ? 0x3FU : 0U;

    // Bench RC->PWM passthrough for the four quad-X outputs. Called every
    // main-loop tick from HAL_ChibiOS_K3::run(), after rcinDriver.update()
    // so fresh iBus data is available. See bench_passthrough.cpp for scope
    // and safety notes.
    void bench_passthrough_update();
}
