#pragma once

#include <stdint.h>

/*
  MASTER SWITCH, 2026-08-03. 0 = ArduCopter's own AP_Motors mixer drives the
  four outputs, i.e. real IMU/EKF3-based stabilisation reaches the pins;
  1 = the bench RC->PWM passthrough owns them instead.

  Default is now 0. The passthrough existed because Empty::Storage could not
  persist FRAME_CLASS/FRAME_TYPE/SERVOn_FUNCTION, so the vehicle's mixer could
  not be configured at all. M4 fixed that on 2026-08-02 -- a parameter now
  survives a power cycle -- so the reason for bypassing the vehicle is gone.

  Kept rather than deleted: the passthrough is the only path that produces
  motor output without arming, prearm checks or calibration, which makes it the
  right tool when something further up the stack is broken. Set to 1 and
  rebuild to get it back.

  DO NOT set the mask to 0 while leaving the module running. Both would then
  write the same compare registers and the DR-012 shadow-register race returns
  -- CMPA/CMPB load at CTR=ZERO once per 20 ms period, at a phase unrelated to
  the main loop, so each period independently latches whichever writer touched
  the shadow last. That is what the exclusive mask was built to end. The two
  settings below are derived from one switch for exactly this reason.
*/
#ifndef PT_ENABLE
#define PT_ENABLE 0
#endif

/*
  Q-34 A/B switch, 2026-07-30. Only meaningful while PT_ENABLE is 1. 1 = the
  passthrough owns the channels exclusively and AP_Motors' competing writes are
  dropped in the HAL; 0 = the old DR-012 "run last every tick" behaviour, which
  loses the shadow-register race once per PWM period (see RCOutput.h,
  set_exclusive_mask()). Kept so the fix can be A/B'd against the dancing
  baseline on hardware without reverting code.
*/
#ifndef PT_EXCLUSIVE_OUTPUTS
#define PT_EXCLUSIVE_OUTPUTS 1
#endif

namespace ChibiOS_K3
{
// Every real channel, not just the four this module mixes: SRV_Channels
// ::push() writes every channel every tick, so any spare channel needs the
// same protection to stay at its safe boot idle. Zero when the passthrough
// is disabled, which is what lets AP_Motors reach the pins at all.
constexpr uint32_t PT_EXCLUSIVE_MASK =
    (PT_ENABLE && PT_EXCLUSIVE_OUTPUTS) ? 0x0FU : 0U;

// Bench RC->PWM passthrough for the four quad-X outputs. Called every
// main-loop tick from HAL_ChibiOS_K3::run(), after rcinDriver.update()
// so fresh iBus data is available. See bench_passthrough.cpp for scope
// and safety notes.
void bench_passthrough_update();
}
