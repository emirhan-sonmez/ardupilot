#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_K3_Namespace.h"

/*
  Four-channel RCOutput for the AM67/J722S K3 backend -- quad X, eHRPWM only.

  Channel -> peripheral/output -> Gemstone 40-pin header pin:
    ch0 -> EHRPWM0_A -> GPIO5  -> pin 29
    ch1 -> EHRPWM0_B -> GPIO14 -> pin 8
    ch2 -> EHRPWM1_A -> GPIO6  -> pin 31
    ch3 -> EHRPWM1_B -> GPIO13 -> pin 33

  WHY NO eCAP. Three eCAP channels (pins 32/36/12) were previously used to
  reach six outputs. The airframe is a quad and needs four, and eCAP is not
  equivalent hardware: a different IP block, a 125 MHz fck against eHRPWM's
  250 MHz (DR-002), and different shadow-load semantics -- ecap_start() drops
  the active compare to 0% immediately where ehrpwm_start() does not. Four
  motors on one peripheral type removes a whole class of asymmetry from the
  output path. The eCAP driver is left in the ChibiOS tree, unused.

  WHY PIN 8 IS AVAILABLE NOW. It was MAIN_UART1 TX. DR-016 moved MAVLink to
  the shared-memory rings, so RCInput uses only pin 10 (RX), and the stock
  overlay k3-am67a-t3-gem-o1-pwm-epwm0-gpio5-gpio14.dtbo reconfigures
  main_uart1 to an RX-only pin group as part of taking pad 0x01B0 for
  EHRPWM0_B. Nothing on this port transmits on UART1 any more.

  Two peripherals back the four channels: EPWM0 drives ch0+ch1 and EPWM1
  drives ch2+ch3, each pair sharing one time base. Each peripheral's time base
  is clocked by a Linux-owned gate, so enable_ch() waits for that peripheral's
  counter to actually advance before programming it, and refuses to enable a
  channel whose peripheral clock never started.

  NOTE ON MOTOR ORDER. ArduCopter's quad-X frame maps motor 1..4 to channels
  0..3, so the ESC leads must follow the pin order above, NOT the order used
  before this change (pins 29/31/33/32).
*/
class ChibiOS_K3::RCOutput : public AP_HAL::RCOutput
{
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

    // Re-writes the whole output configuration -- TBCTL prescale, TBPRD,
    // CMPCTL and AQCTLA/AQCTLB -- on every enabled channel, every call. Safe
    // every tick, never touches CMPA/CMPB, so a commanded pulse width is
    // never disturbed.
    //
    // Widened from AQCTL-only 2026-08-03. Linux owns the PWM clock gate and
    // r5f-setup.sh brings each peripheral up by exporting its sysfs pwm0 and
    // writing period/duty/enable -- and pwm0 is eHRPWM channel A, i.e. pins
    // 29 and 31. That write programs TBCTL, TBPRD, CMPA and AQCTLA with
    // Linux's own values, and it races this firmware at boot because both
    // remoteproc and gemstone-r5f-setup.service run during the same startup.
    // TBCTL/TBPRD/CMPCTL were previously written exactly once in
    // ehrpwm_start() and never revisited, so whichever side wrote last won
    // permanently. Reasserting all of it makes the R5F unconditionally the
    // last writer regardless of who got there first.
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

    /* Drive every channel to its disarmed pulse width, bypassing the exclusive
       mask. For the remoteproc shutdown path only: the kernel resets this core
       immediately after we acknowledge, and a stopped R5F leaves the PWM
       peripherals emitting the last commanded width forever with no watchdog
       to catch it. Safe from ISR context -- it only writes compare registers,
       takes no lock and cannot block. */
    void     park_all_disarmed();

    // Count of write() calls dropped by the exclusive mask. Non-zero and
    // climbing at ~loop_rate x NUM_CH is the direct proof that a second
    // writer was competing for these pins. Healthy value is loop_rate x 4
    // since the channel count dropped from 6.
    uint32_t foreign_writes_blocked() const
    {
        return _foreign_blocked;
    }

private:
    static const uint8_t  NUM_CH = 4;
    static const uint8_t  NUM_PERIPH = 2;      // EPWM0 (ch0+ch1), EPWM1 (ch2+ch3)
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
