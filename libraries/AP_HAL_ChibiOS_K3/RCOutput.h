#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_K3_Namespace.h"

/*
  Temporary six-channel RCOutput for the AM67/J722S K3 backend.

  Channel -> peripheral/output -> Gemstone 40-pin header pin:
    ch0 -> EHRPWM0_A  -> GPIO5  -> pin 29   (scope-verified)
    ch1 -> EHRPWM1_A  -> GPIO6  -> pin 31
    ch2 -> EHRPWM1_B  -> GPIO13 -> pin 33
    ch3 -> ECAP0 APWM -> GPIO12 -> pin 32
    ch4 -> ECAP1 APWM -> GPIO16 -> pin 36
    ch5 -> ECAP2 APWM -> GPIO18 -> pin 12

  EHRPWM0_B (pin 8) is intentionally NOT used -- it is the UART1 console TX.

  Five peripherals back the six channels (EPWM1 drives ch1+ch2). Each
  peripheral's time base is clocked by a Linux-owned gate, so enable_ch() waits
  for that peripheral's counter to actually advance before programming it, and
  refuses to enable a channel whose peripheral clock never started.
*/
class ChibiOS_K3::RCOutput : public AP_HAL::RCOutput {
public:
    void     init() override;
    void     set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t chan) override;
    void     enable_ch(uint8_t chan) override;
    void     disable_ch(uint8_t chan) override;
    void     write(uint8_t chan, uint16_t period_us) override;
    uint16_t read(uint8_t chan) override;
    void     read(uint16_t *period_us, uint8_t len) override;
    void     cork() override {}
    void     push() override {}

    // Re-attempts enable_ch() for any channel not yet enabled. Cheap
    // (~5ms per still-dead peripheral, no busy loop) -- the Linux PWM
    // clock a peripheral depends on is commonly enabled well after this
    // firmware has already booted (see the ArduPilot iBus Port Handoff,
    // section 6), so a channel that failed at boot-time safe-init can
    // recover once that script runs, without needing a reboot. Call
    // periodically, not every loop tick.
    void     retry_pending();

    // Re-writes AQCTLA/AQCTLB on every enabled EPWM channel (ch0-2) every
    // call -- safe every tick, never touches CMPA/CMPB. Added 2026-07-30:
    // AQCTLA/B was otherwise written exactly once, at enable_ch() time,
    // and never revisited -- same written-once-never-reasserted shape as
    // the TBPRD/frequency bug fixed the same day. Closes that gap for
    // ECAP channels too where cheap to do so.
    void     reassert_outputs();

    /*
      Q-34 root-cause fix, 2026-07-30. Channels in this mask reject every
      write() and accept only write_exclusive(); one module owns the pin
      outright instead of out-racing the other writer.

      Why a mask and not write ordering: DR-012 tried to win the conflict by
      running bench_passthrough_update() after callbacks->loop(), so it was
      the last writer of each main-loop iteration. That cannot work. CMPA/CMPB
      are shadowed and load into the active compare at CTR=ZERO -- once per
      20ms PWM period, at a phase uncorrelated with the ~2.5ms main loop. The
      value that reaches the pin is whichever writer touched the shadow last
      before that load event, not whichever ran last in the iteration. With
      AP_Motors writing 1000us (SRV_Channels::push(), every tick, all six
      channels) and the passthrough writing its commanded value microseconds
      later, the shadow holds 1000 for the tail of loop() and the commanded
      value for the rest, so each PWM period independently latches one or the
      other. Fraction of wrong periods = (gap between the two writes) /
      (loop period), and the gap varies with which AP_Scheduler tasks ran that
      tick -- observed on the scope as bursts of correct duty mixed with
      bursts at exactly 5% (cmp=3125 of tbprd=62500 = 1000us).

      Single-threaded by construction: Scheduler::register_timer_process() is
      a no-op on this port, so every writer is on the main thread and a mask
      is sufficient. No locking needed.

      This is a bench stopgap, same scope as DR-012: the real handoff is
      stopping AP_Motors from owning these channels at all (SERVOn_FUNCTION=0
      via compiled-in hwdef defaults, since Empty::Storage cannot persist
      parameters). Not attempted here -- ArduCopter auto-assigns motor
      functions via SRV_Channels::set_default_function(), and
      default-vs-default precedence needs its own investigation.
    */
    void     set_exclusive_mask(uint32_t mask);
    void     write_exclusive(uint8_t chan, uint16_t period_us);

    // Count of write() calls dropped by the exclusive mask. Non-zero and
    // climbing at ~loop_rate x NUM_CH is the direct proof that a second
    // writer was competing for these pins.
    uint32_t foreign_writes_blocked() const { return _foreign_blocked; }

private:
    static const uint8_t  NUM_CH = 6;
    static const uint8_t  NUM_PERIPH = 5;      // EPWM0, EPWM1, ECAP0, ECAP1, ECAP2
    static const uint16_t PWM_MIN_US = 1000;   // test clamp
    static const uint16_t PWM_MAX_US = 2000;

    bool ensure_peripheral(uint8_t p);         // wait for clock, start once
    bool wait_for_timebase(uint8_t p, uint16_t max_tries);  // TBCTR/TSCTR advancing?
    void hw_set(uint8_t chan, uint16_t us);    // drive the right compare reg

    void     hw_write(uint8_t chan, uint16_t period_us);  // shared write body

    uint32_t _exclusive_mask = 0;
    uint32_t _foreign_blocked = 0;
    uint16_t _freq_hz = 50;
    uint16_t _pulse_us[NUM_CH]   = {0};
    bool     _ch_enabled[NUM_CH] = {false};
    bool     _p_started[NUM_PERIPH] = {false};
    bool     _p_failed[NUM_PERIPH]  = {false};
};
