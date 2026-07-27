/*
  Safe UART-controlled ESC test for the AM67/J722S K3 backend.

  RCOutput channel 0 -> EPWM0_A -> Gemstone 40-pin header pin 29, 50 Hz.

  Arms at 1000 us and stays there until a valid single-character command is
  received on SERIAL0 (57600 8N1). Commands latch. Output is clamped to
  1000-2000 us in this test. Keys 0-9 step across the range:

    0 -> 1000   2 -> 1200   4 -> 1400   6 -> 1600   8 -> 1800
    1 -> 1100   3 -> 1300   5 -> 1500   7 -> 1700   9 -> 2000
    s -> 1000 (idle)        x -> 1000 (immediate safety)

  CR/LF/space are ignored; any other character leaves the output unchanged and
  prints an "invalid command" message. UART reads are non-blocking.

  Bench prerequisite (temporary Linux-assisted tbclk): enable a Linux pwm
  channel on 23000000.pwm first so the EPWM time base is clocked; enable_ch(0)
  waits for the running counter before programming EPWM0.

  SAFETY: keep a propeller off. This now spans full throttle (up to 2000 us).
*/
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
#include <AP_HAL_ChibiOS_K3/hwdef/boot/trace.h>
#define RC_TRACE(...) trace_printf(__VA_ARGS__)
#else
#define RC_TRACE(...) do {} while (0)
#endif

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Test-application limits (independent of any HAL clamp).
static const uint16_t ESC_MIN_US = 1000;
static const uint16_t ESC_MAX_US = 2000;

static uint16_t current_us = ESC_MIN_US;   // latched output, armed low

// Map a command character to a target pulse width. Returns false if the
// character is not a throttle/safety command (CR/LF/space handled separately).
// Keys 0-9 step 1000..2000 us (100 us apart, 9 lands on full 2000).
static bool map_command(char c, uint16_t &out_us)
{
    if (c >= '0' && c <= '9') {
        out_us = (uint16_t)(1000U + (uint16_t)(c - '0') * 100U);  // 9 -> 1900
        if (c == '9') {
            out_us = 2000;                                        // top out at 2000
        }
        return true;
    }
    switch (c) {
    case 's': out_us = 1000; return true;   // idle
    case 'x': out_us = 1000; return true;   // immediate safety
    default:  return false;
    }
}

static void apply_us(char c, uint16_t us)
{
    // Strictly clamp to the test range regardless of source.
    if (us < ESC_MIN_US) { us = ESC_MIN_US; }
    if (us > ESC_MAX_US) { us = ESC_MAX_US; }
    current_us = us;
    hal.rcout->write(0, current_us);
    hal.console->printf("Command %c -> RCOutput ch0 = %u us\r\n",
                        c, (unsigned)current_us);
    RC_TRACE("esc: cmd %c -> %u us\n", c, (unsigned)current_us);
}

void setup(void)
{
    RC_TRACE("esc: setup()\n");
    hal.console->printf("\r\nAP-K3 UART-controlled ESC test: ch0 -> EPWM0_A -> pin 29\r\n");
    hal.console->printf("cmds: 0..9 = 1000..2000 us (100 us steps, 9=2000); s/x = 1000\r\n");

    hal.rcout->init();
    hal.rcout->set_freq(1U << 0, 50);       // 50 Hz frame on channel 0
    hal.rcout->enable_ch(0);                 // waits for tbclk, starts EPWM0
    hal.rcout->write(0, current_us);         // arm immediately at 1000 us

    hal.console->printf("armed at %u us; waiting for command\r\n",
                        (unsigned)current_us);
    RC_TRACE("esc: armed %u us\n", (unsigned)current_us);
}

void loop(void)
{
    // Non-blocking read: -1 when no byte is available, so the main loop never
    // stalls waiting for input.
    int16_t ci = hal.console->read();
    if (ci < 0) {
        hal.scheduler->delay(2);
        return;
    }

    char c = (char)ci;
    if (c == '\r' || c == '\n' || c == ' ') {
        return;                              // ignore line endings / spaces
    }

    uint16_t us;
    if (map_command(c, us)) {
        apply_us(c, us);                     // latches current_us
    } else {
        hal.console->printf("invalid command '%c' -> RCOutput ch0 unchanged at %u us\r\n",
                            c, (unsigned)current_us);
        RC_TRACE("esc: invalid cmd, held %u us\n", (unsigned)current_us);
    }
}

AP_HAL_MAIN();
