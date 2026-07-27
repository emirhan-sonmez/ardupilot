#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_K3_Namespace.h"

/*
  Minimal single-channel RCOutput for the AM67/J722S K3 backend.

  Mapping (M5, first slice):
    RCOutput channel 0  ->  AM67 EPWM0_A  ->  Gemstone 40-pin header pin 29.

  Channel 0 is a thin wrapper over the register-level am67_epwm driver in the
  ChibiOS AM67 port (the same driver scope-verified at 50 Hz / 1.0-2.0 ms).
  Channels 1..n have no hardware yet and are ignored. The EPWM time base is
  clocked by the Linux-owned epwm_tbclk gate, so enable_ch() waits until the
  counter is actually running before programming EPWM0.
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

private:
    static const uint8_t CH_EPWM0A = 0;   // channel 0 -> EPWM0_A -> pin 29

    // Block (bounded) until the Linux-owned tbclk gate is running (TBCTR moves).
    void wait_for_timebase();

    uint16_t _freq_hz = 50;               // default servo/ESC frame
    uint16_t _pulse_us = 0;               // last commanded high-time (0 = low)
    bool     _started = false;            // EPWM0 time base programmed by us
    bool     _enabled = false;            // channel 0 enabled
};
