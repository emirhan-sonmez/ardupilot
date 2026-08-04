#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include "RCOutput.h"
#include <ch.h>                // chThdSleepMilliseconds
#include <hal.h>               // AM67_* base addresses (board.h)
#include <am67_epwm.h>         // generic eHRPWM driver (ChibiOS AM67 port)
#include "hwdef/boot/trace.h"  // RemoteProc trace buffer (independent of UART)

using namespace ChibiOS_K3;

// Peripheral indices. Each eHRPWM backs two channels off one shared time base.
enum { P_EPWM0 = 0, P_EPWM1 = 1 };

struct periph_desc {
    uint32_t base;
};
static const periph_desc PERIPH[] = {
    { AM67_EPWM0_BASE },
    { AM67_EPWM1_BASE },
};

struct chan_desc {
    uint8_t periph;
    bool output_b;
};
static const chan_desc CHAN[] = {
    { P_EPWM0, false },   // ch0 EHRPWM0_A -> pin 29
    { P_EPWM0, true  },   // ch1 EHRPWM0_B -> pin 8
    { P_EPWM1, false },   // ch2 EHRPWM1_A -> pin 31
    { P_EPWM1, true  },   // ch3 EHRPWM1_B -> pin 33
};

void RCOutput::init()
{
    // NUM_CH/NUM_PERIPH are private, so these tables cannot be sized from them
    // directly at file scope. Check the agreement here instead, where the
    // members are visible: a table that disagrees with the counts indexes off
    // the end of the other one, silently, on a live output path.
    static_assert(sizeof(CHAN) / sizeof(CHAN[0]) == NUM_CH,
                  "CHAN table does not match NUM_CH");
    static_assert(sizeof(PERIPH) / sizeof(PERIPH[0]) == NUM_PERIPH,
                  "PERIPH table does not match NUM_PERIPH");
    trace_printf("rcout: init (4 channels, eHRPWM only)\n");
}

/*
  Root-cause fix, 2026-07-30: the only frequency ever hardware-verified on
  this board is 50 Hz (the traditional RC PWM rate every scope check, the
  bench passthrough's cmp<->us math, and standard analog ESCs all assume).
  ArduCopter's own AP_Motors/RC_SPEED init calls set_freq() during
  callbacks->setup() with an ESC-oriented rate (observed on hardware as
  400 Hz) -- a rate this port has never proven correct. Previously this
  was "fixed" after the fact from bench_passthrough.cpp's first tick, which
  left a real window (the whole of setup(), several seconds) where the
  physical pins carried the wrong frequency before anything corrected it.
  That is not acceptable on pins that may have a live ESC.

  Fix moved here instead: refuse any request that is not the
  hardware-verified rate. Nothing above this driver is trusted with
  frequency until a rate other than 50 Hz has actually been scope-verified
  on this hardware -- at that point, raise RCOUTPUT_VERIFIED_FREQ_HZ (and
  update this comment), do not just delete the check.
*/
constexpr uint16_t RCOUTPUT_VERIFIED_FREQ_HZ = 50;

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    if (freq_hz == 0) {
        return;
    }
    if (freq_hz != RCOUTPUT_VERIFIED_FREQ_HZ) {
        trace_printf("rcout: set_freq(%u) refused, only %u Hz is hardware-verified on this board\n",
                     (uint32_t)freq_hz, (uint32_t)RCOUTPUT_VERIFIED_FREQ_HZ);
        freq_hz = RCOUTPUT_VERIFIED_FREQ_HZ;
    }
    /*
      Idempotence guard, 2026-07-30. Re-running ehrpwm_start() on a peripheral
      already at this frequency is destructive, not free: it writes TBCTR = 0,
      and resetting the counter part-way through a period stretches or
      truncates that one period while the pulse width stays put, so the
      measured duty jumps for a cycle. A reset landing just after the CMPA
      match roughly doubles the period -> ~5-7% measured where 10% was
      commanded.

      Observed on hardware via the eCAP path (since removed), whose active
      compare went 125000 -> 0 across bench_passthrough's own set_freq(0x3F,
      50) call, a no-op by intent. Note the frequency clamp above runs first,
      so even a *refused* request (AP_Motors asks for 400 then 490 during
      setup) reached this loop and glitched every pin. Nothing above this
      driver should be able to disturb a running output by asking for the
      frequency it already has.
    */
    if (freq_hz == _freq_hz) {
        return;
    }

    _freq_hz = freq_hz;
    // Genuine frequency change: re-program the time base of any already-started
    // peripheral referenced by the mask, then restore that channel's commanded
    // pulse width -- the restart above resets the compare registers, and the
    // caller is entitled to assume set_freq() does not silently change duty.
    for (uint8_t ch = 0; ch < NUM_CH; ch++) {
        if ((chmask & (1U << ch)) == 0) {
            continue;
        }
        uint8_t p = CHAN[ch].periph;
        if (_p_started[p]) {
            ehrpwm_start(PERIPH[p].base, _freq_hz);
            if (_ch_enabled[ch]) {
                hw_set(ch, _pulse_us[ch]);
            }
        }
    }
}

uint16_t RCOutput::get_freq(uint8_t chan)
{
    return (chan < NUM_CH) ? _freq_hz : 0;
}

bool RCOutput::wait_for_timebase(uint8_t p, uint16_t max_tries)
{
    const periph_desc &d = PERIPH[p];
    // Counter must advance, else its clock gate never started. max_tries=16
    // (~5 s, the boot-time safe-init path) vs. max_tries=1 (~5ms, the cheap
    // periodic retry_pending() path) -- see ensure_peripheral().
    for (uint16_t tries = 0; tries < max_tries; tries++) {
        uint32_t a = ehrpwm_read_tbctr(d.base);
        chThdSleepMilliseconds(5);
        uint32_t b = ehrpwm_read_tbctr(d.base);
        if (a != b) {
            trace_printf("rcout: periph %u timebase running\n", (uint32_t)p);
            return true;
        }
        if ((tries + 1) < max_tries) {
            chThdSleepMilliseconds(300);
        }
    }
    return false;
}

bool RCOutput::ensure_peripheral(uint8_t p)
{
    if (_p_started[p]) {
        return true;
    }
    // First-ever attempt for this peripheral gets the full bounded wait
    // (boot-time safe-init, nothing else to do yet); a peripheral that
    // already failed once gets a cheap single-shot recheck instead of a
    // permanent latch -- the Linux PWM clock it depends on is commonly
    // enabled well after boot (ArduPilot iBus Port Handoff, section 6),
    // and the ChibiOS demo this was ported from recovers the same way via
    // its own pt_init() retry rather than requiring a reboot.
    const uint16_t max_tries = _p_failed[p] ? 1 : 16;
    if (!wait_for_timebase(p, max_tries)) {
        if (!_p_failed[p]) {
            trace_printf("rcout: periph %u CLOCK TIMEOUT (counter never advanced)\n",
                         (uint32_t)p);
        }
        _p_failed[p] = true;
        return false;
    }
    _p_failed[p] = false;
    ehrpwm_start(PERIPH[p].base, _freq_hz);
    _p_started[p] = true;
    trace_printf("rcout: periph %u started (freq=%u)\n",
                 (uint32_t)p, (uint32_t)_freq_hz);
    return true;
}

void RCOutput::retry_pending()
{
    for (uint8_t chan = 0; chan < NUM_CH; chan++) {
        if (!_ch_enabled[chan]) {
            enable_ch(chan);
        }
    }
}

void RCOutput::reassert_outputs()
{
    for (uint8_t chan = 0; chan < NUM_CH; chan++) {
        if (!_ch_enabled[chan]) {
            continue;
        }
        const chan_desc &c = CHAN[chan];
        ehrpwm_out_reassert(PERIPH[c.periph].base, c.output_b, _freq_hz);
    }
}

void RCOutput::hw_set(uint8_t chan, uint16_t us)
{
    const chan_desc &c = CHAN[chan];
    ehrpwm_out_set_pulse_us(PERIPH[c.periph].base, c.output_b, us);
}

void RCOutput::enable_ch(uint8_t chan)
{
    if (chan >= NUM_CH) {
        return;                              // ignore out-of-range safely
    }
    if (_ch_enabled[chan]) {
        return;                              // already enabled -- callers
        // (Plane's own servo output,
        // this port's retry_pending())
        // call this every cycle, and
        // re-running the body below
        // unconditionally, trace print
        // included, filled the 16 KiB
        // trace buffer within seconds.
    }
    const chan_desc &c = CHAN[chan];
    if (!ensure_peripheral(c.periph)) {
        // Bounded: retry_pending() calls this repeatedly for a channel
        // whose peripheral clock genuinely never comes up, which would
        // otherwise spam this line forever.
        static uint8_t fail_trace_count;
        if (fail_trace_count < NUM_CH) {
            fail_trace_count++;
            trace_printf("rcout: ch%u NOT enabled (periph %u clock failed)\n",
                         (uint32_t)chan, (uint32_t)c.periph);
        }
        return;                              // do not enable if clock failed
    }
    ehrpwm_out_enable(PERIPH[c.periph].base, c.output_b);
    _ch_enabled[chan] = true;

    uint16_t us = _pulse_us[chan];
    if (us < PWM_MIN_US) {
        us = PWM_MIN_US;
    }
    if (us > PWM_MAX_US) {
        us = PWM_MAX_US;
    }
    _pulse_us[chan] = us;
    hw_set(chan, us);
    trace_printf("rcout: ch%u enabled -> %u us\n", (uint32_t)chan, (uint32_t)us);
}

void RCOutput::disable_ch(uint8_t chan)
{
    if (chan >= NUM_CH) {
        return;
    }
    _ch_enabled[chan] = false;
    const chan_desc &c = CHAN[chan];
    ehrpwm_out_low(PERIPH[c.periph].base, c.output_b);
}

void RCOutput::set_exclusive_mask(uint32_t mask)
{
    _exclusive_mask = mask;
    trace_printf("rcout: exclusive mask=0x%x (write() from other modules now dropped)\n",
                 (uint32_t)mask);
}

void RCOutput::write(uint8_t chan, uint16_t period_us)
{
    if (chan >= NUM_CH) {
        return;
    }
    // Q-34: an exclusively-owned channel drops foreign writes outright --
    // including the _pulse_us[] cache, so read() keeps reporting the owner's
    // real commanded value rather than what the loser asked for. See the
    // set_exclusive_mask() comment in RCOutput.h for why ordering cannot
    // substitute for this.
    if ((_exclusive_mask & (1U << chan)) != 0) {
        _foreign_blocked++;
        return;
    }
    hw_write(chan, period_us);
}

void RCOutput::write_exclusive(uint8_t chan, uint16_t period_us)
{
    if (chan >= NUM_CH) {
        return;
    }
    hw_write(chan, period_us);
}

void RCOutput::park_all_disarmed()
{
    /* Only channels that were actually brought up. Writing a peripheral whose
       clock never started is what ensure_peripheral()/_p_failed[] exist to
       refuse, and this runs from an ISR on the way to a core reset -- the
       worst possible place to stall on a dead peripheral. */
    for (uint8_t ch = 0; ch < NUM_CH; ch++) {
        if (_ch_enabled[ch]) {
            hw_write(ch, PWM_MIN_US);
        }
    }
}

void RCOutput::hw_write(uint8_t chan, uint16_t period_us)
{
    if (period_us < PWM_MIN_US) {
        period_us = PWM_MIN_US;
    }
    if (period_us > PWM_MAX_US) {
        period_us = PWM_MAX_US;
    }
    _pulse_us[chan] = period_us;

    if (!_ch_enabled[chan]) {
        return;
    }
    hw_set(chan, period_us);

    // Diagnostics only, bounded. Once the vehicle main loop is running,
    // SRV_Channels writes every channel every iteration, so an unbounded
    // trace here floods the 16 KiB RemoteProc trace buffer within a few
    // loops and hides everything logged after it (trace.c stops accepting
    // once full). PWM behaviour above is unchanged.
    static uint8_t write_trace_count;
    if (write_trace_count >= 16) {          // ~4 loops x 4 channels
        return;
    }
    write_trace_count++;

    const chan_desc &c = CHAN[chan];
    // cmp_shadow, not the active compare: CMPCTL keeps CMPA/CMPB in shadow
    // mode (load at CTR=ZERO) and a read of the CMPA/CMPB address returns the
    // shadow. Classic eHRPWM exposes no separate active-compare address, so
    // this readback can only ever confirm our own last write -- it cannot
    // prove what the pin is doing. Do not treat it as pin proof.
    trace_printf("rcout: ch%u=%u us EPWM cmp_shadow=%u tbprd=%u\n",
                 (uint32_t)chan, (uint32_t)period_us,
                 (uint32_t)ehrpwm_read_cmp(PERIPH[c.periph].base, c.output_b),
                 (uint32_t)ehrpwm_read_tbprd(PERIPH[c.periph].base));
}

uint16_t RCOutput::read(uint8_t chan)
{
    return (chan < NUM_CH) ? _pulse_us[chan] : 0;
}

void RCOutput::read(uint16_t *period_us, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        period_us[i] = (i < NUM_CH) ? _pulse_us[i] : 0;
    }
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
