/*
  Walking-channel diagnostic for the six-channel AM67/J722S K3 RCOutput.

  Channel -> peripheral/output -> Gemstone 40-pin header pin:
    ch0 -> EHRPWM0_A  -> pin 29     ch3 -> ECAP0 APWM -> pin 32
    ch1 -> EHRPWM1_A  -> pin 31     ch4 -> ECAP1 APWM -> pin 36
    ch2 -> EHRPWM1_B  -> pin 33     ch5 -> ECAP2 APWM -> pin 12

  All six start at 1000 us. Each channel in turn is raised to 1200 us for 2 s
  and then returned to 1000 us, walking ch0..ch5 and repeating. Every
  non-selected channel stays at 1000 us.

  Bench prerequisite (temporary Linux-assisted clocks): enable a Linux pwm
  channel on each of the five peripherals first (23000000/23010000.pwm and
  23100000/23110000/23120000.ecap) so their time bases are clocked; each
  RCOutput channel waits for its counter to advance and is skipped if that
  peripheral's clock never starts. Pin 8 (UART TX) is untouched.

  SAFETY: keep propellers off.
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

static const uint8_t  NUM_CH   = 6;
static const uint16_t IDLE_US  = 1000;
static const uint16_t WALK_US  = 1200;

void setup(void)
{
    RC_TRACE("walk: setup()\n");
    hal.console->printf("\r\nAP-K3 6-channel walking PWM diagnostic\r\n");

    hal.rcout->init();
    hal.rcout->set_freq((1U << NUM_CH) - 1, 50);   // ch0..ch5 @ 50 Hz

    // Set every output safely to 1000 us BEFORE enabling.
    for (uint8_t ch = 0; ch < NUM_CH; ch++) {
        hal.rcout->write(ch, IDLE_US);
    }
    // Enable each channel (waits for its peripheral clock; skips on timeout).
    for (uint8_t ch = 0; ch < NUM_CH; ch++) {
        hal.rcout->enable_ch(ch);
    }
    // Hold all at idle.
    for (uint8_t ch = 0; ch < NUM_CH; ch++) {
        hal.rcout->write(ch, IDLE_US);
    }
    hal.console->printf("all channels armed at %u us\r\n", (unsigned)IDLE_US);
    RC_TRACE("walk: armed all at %u us\n", (unsigned)IDLE_US);
}

static uint8_t sel = 0;

void loop(void)
{
    RC_TRACE("walk: select ch%u -> %u us\n", (unsigned)sel, (unsigned)WALK_US);
    hal.console->printf("ch%u -> %u us\r\n", (unsigned)sel, (unsigned)WALK_US);
    hal.rcout->write(sel, WALK_US);
    hal.scheduler->delay(2000);

    hal.rcout->write(sel, IDLE_US);
    RC_TRACE("walk: ch%u -> %u us\n", (unsigned)sel, (unsigned)IDLE_US);
    hal.scheduler->delay(200);

    sel = (uint8_t)((sel + 1) % NUM_CH);
}

AP_HAL_MAIN();
