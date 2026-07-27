/*
  Minimal single-channel RCOutput test for the AM67/J722S K3 backend.

  Cycles channel 0 through 1000 / 1500 / 2000 us at 50 Hz, every 3 seconds:
    RCOutput channel 0 -> EPWM0_A -> Gemstone 40-pin header pin 29.

  Bench prerequisite (Linux-assisted tbclk): the EPWM0 time base is clocked by
  the Linux-owned epwm_tbclk gate, so enable a Linux pwm channel first
  (keeps fck on + turns the gate on); enable_ch(0) waits for the running
  counter before programming EPWM0. Scope pin 29. Console is SERIAL0 @ 57600.

  Instrumented with trace_printf (RemoteProc trace buffer, independent of UART)
  so the flow can be followed even if the console is silent.
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

void setup(void)
{
    RC_TRACE("rc-test: setup() entry\n");
    hal.console->printf("\r\nAP-K3 RCOutput test: ch0 -> EPWM0_A -> pin 29\r\n");
    hal.console->printf("enable a Linux pwm channel now (starts the tbclk gate)\r\n");

    RC_TRACE("rc-test: before init()\n");
    hal.rcout->init();
    RC_TRACE("rc-test: after init()\n");

    hal.rcout->set_freq(1U << 0, 50);   // 50 Hz frame on channel 0

    RC_TRACE("rc-test: before enable_ch(0)\n");
    hal.rcout->enable_ch(0);            // waits for the time base, starts EPWM0
    RC_TRACE("rc-test: after enable_ch(0)\n");

    hal.console->printf("RCOutput ch0 enabled\r\n");
}

static const uint16_t pulses_us[3] = { 1000, 1500, 2000 };
static uint8_t idx = 0;

void loop(void)
{
    RC_TRACE("rc-test: loop iter idx=%u requested=%u us\n",
             (unsigned)idx, (unsigned)pulses_us[idx]);

    hal.rcout->write(0, pulses_us[idx]);

    hal.console->printf("RCOutput ch0 = %u us (readback %u)\r\n",
                        (unsigned)pulses_us[idx],
                        (unsigned)hal.rcout->read(0));
    idx = (idx + 1) % 3;

    RC_TRACE("rc-test: delay(3000) enter\n");
    hal.scheduler->delay(3000);
    RC_TRACE("rc-test: delay(3000) return\n");
}

AP_HAL_MAIN();
