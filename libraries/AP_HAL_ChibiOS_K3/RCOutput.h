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

private:
    static const uint8_t  NUM_CH = 6;
    static const uint8_t  NUM_PERIPH = 5;      // EPWM0, EPWM1, ECAP0, ECAP1, ECAP2
    static const uint16_t PWM_MIN_US = 1000;   // test clamp
    static const uint16_t PWM_MAX_US = 2000;

    bool ensure_peripheral(uint8_t p);         // wait for clock, start once
    bool wait_for_timebase(uint8_t p, uint16_t max_tries);  // TBCTR/TSCTR advancing?
    void hw_set(uint8_t chan, uint16_t us);    // drive the right compare reg

    uint16_t _freq_hz = 50;
    uint16_t _pulse_us[NUM_CH]   = {0};
    bool     _ch_enabled[NUM_CH] = {false};
    bool     _p_started[NUM_PERIPH] = {false};
    bool     _p_failed[NUM_PERIPH]  = {false};
};
