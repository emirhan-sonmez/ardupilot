#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include "Scheduler.h"
#include <ch.h>
#include <hal.h>

using namespace ChibiOS_K3;

/*
  M3 scheduler: enough to run the UART_test example.

  init() performs the ChibiOS bring-up the demo main() normally does (halInit +
  chSysInit) and records the main thread. delay/delay_microseconds sleep the
  calling thread. The timer/io process lists are stored but not serviced yet.
*/

void Scheduler::init()
{
    // Bring up the ChibiOS HAL and RT kernel (as the demo main() does). After
    // chSysInit() this thread becomes the ChibiOS main thread and the systick
    // starts, so AP_HAL time and delays work from here on.
    halInit();
    chSysInit();
    _main_thread = (void *)chThdGetSelfX();
    _initialized = false;
}

void Scheduler::delay(uint16_t ms)
{
    if (ms == 0) {
        return;
    }
    chThdSleepMilliseconds(ms);
}

void Scheduler::delay_microseconds(uint16_t us)
{
    if (us == 0) {
        return;
    }
    chThdSleepMicroseconds(us);
}

void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    // TODO: run from a periodic timer thread. Not needed by UART_test.
    (void)proc;
}

void Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
    // TODO: run from a low-priority IO thread. Not needed by UART_test.
    (void)proc;
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    (void)failsafe;
    (void)period_us;
}

void Scheduler::set_system_initialized()
{
    _initialized = true;
}

bool Scheduler::is_system_initialized()
{
    return _initialized;
}

void Scheduler::reboot(bool hold_in_bootloader)
{
    // TODO: real R5F reset path. Not needed by UART_test.
    (void)hold_in_bootloader;
    while (true) {
    }
}

bool Scheduler::in_main_thread() const
{
    return (void *)chThdGetSelfX() == _main_thread;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
