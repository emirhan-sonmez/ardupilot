#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include "RCOutput.h"
#include <ch.h>              // chThdSleepMilliseconds
#include <am67_epwm.h>       // register-level EPWM0_A driver (ChibiOS AM67 port)
#include "hwdef/boot/trace.h" // RemoteProc trace buffer (independent of UART)

using namespace ChibiOS_K3;

void RCOutput::init()
{
    // Nothing to touch until enable_ch(): the EPWM time base is only clocked
    // once the Linux-owned epwm_tbclk gate is enabled (a Linux pwm channel is
    // enabled from user space).
    trace_printf("rcout: init()\n");
}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    if ((chmask & (1U << CH_EPWM0A)) == 0 || freq_hz == 0) {
        return;
    }
    _freq_hz = freq_hz;
    if (_started) {
        epwm0a_start(_freq_hz);              // re-program the time base
        epwm0a_set_pulse_us(_pulse_us);      // restore the current pulse
    }
}

uint16_t RCOutput::get_freq(uint8_t chan)
{
    return (chan == CH_EPWM0A) ? _freq_hz : 0;
}

void RCOutput::wait_for_timebase()
{
    // TBCTR only advances when the epwm_tbclk gate is running. Poll it, bounded
    // (~20 s) so a missing Linux channel-enable can't hang bring-up forever.
    trace_printf("rcout: wait_for_timebase enter\n");
    for (uint16_t tries = 0; tries < 40; tries++) {
        uint16_t a = epwm0a_read_tbctr();
        chThdSleepMilliseconds(5);
        uint16_t b = epwm0a_read_tbctr();
        if (a != b) {
            trace_printf("rcout: timebase running (a=%u b=%u tries=%u)\n",
                         (uint32_t)a, (uint32_t)b, (uint32_t)tries);
            return;                          // time base is running
        }
        if ((tries % 4U) == 0U) {
            trace_printf("rcout: TBCTR frozen a=%u b=%u tries=%u\n",
                         (uint32_t)a, (uint32_t)b, (uint32_t)tries);
        }
        chThdSleepMilliseconds(500);
    }
    trace_printf("rcout: wait_for_timebase TIMEOUT (counter never advanced)\n");
}

void RCOutput::enable_ch(uint8_t chan)
{
    trace_printf("rcout: enable_ch(%u) started=%u\n",
                 (uint32_t)chan, (uint32_t)_started);
    if (chan != CH_EPWM0A) {
        return;                              // only channel 0 is wired
    }
    if (!_started) {
        wait_for_timebase();
        epwm0a_start(_freq_hz);              // our prescale/period, 0% duty
        _started = true;
        trace_printf("rcout: epwm0a_start(%u) TBPRD=%u TBCTR=%u TBCTL=%x\n",
                     (uint32_t)_freq_hz, (uint32_t)epwm0a_read_tbprd(),
                     (uint32_t)epwm0a_read_tbctr(), (uint32_t)epwm0a_read_tbctl());
    }
    _enabled = true;
    epwm0a_set_pulse_us(_pulse_us);          // apply last commanded (0 => low)
    trace_printf("rcout: enable_ch done pulse=%u CMPA=%u AQCTLA=%x AQCSFRC=%x\n",
                 (uint32_t)_pulse_us, (uint32_t)epwm0a_read_cmpa(),
                 (uint32_t)epwm0a_read_aqctla(), (uint32_t)epwm0a_read_aqcsfrc());
}

void RCOutput::disable_ch(uint8_t chan)
{
    if (chan != CH_EPWM0A) {
        return;
    }
    _enabled = false;
    // Rest the output low (0% duty) but keep the counter running, so a later
    // enable_ch() doesn't stall waiting on a time base we froze ourselves.
    epwm0a_set_pulse_us(0);
}

void RCOutput::write(uint8_t chan, uint16_t period_us)
{
    if (chan != CH_EPWM0A) {
        return;
    }
    _pulse_us = period_us;
    if (_enabled && _started) {
        epwm0a_set_pulse_us(period_us);
        trace_printf("rcout: write(%u,%u) -> CMPA=%u TBPRD=%u\n",
                     (uint32_t)chan, (uint32_t)period_us,
                     (uint32_t)epwm0a_read_cmpa(), (uint32_t)epwm0a_read_tbprd());
    } else {
        trace_printf("rcout: write(%u,%u) IGNORED enabled=%u started=%u\n",
                     (uint32_t)chan, (uint32_t)period_us,
                     (uint32_t)_enabled, (uint32_t)_started);
    }
}

uint16_t RCOutput::read(uint8_t chan)
{
    return (chan == CH_EPWM0A) ? _pulse_us : 0;
}

void RCOutput::read(uint16_t *period_us, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        period_us[i] = (i == CH_EPWM0A) ? _pulse_us : 0;
    }
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
