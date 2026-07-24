#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include "Scheduler.h"

using namespace ChibiOS_K3;

/*
  S2 stub scheduler. See Scheduler.h. Not functional; S3 makes it real with
  ChibiOS/RT.
*/

void Scheduler::init()
{
    _initialized = false;
}

void Scheduler::delay(uint16_t ms)
{
    // TODO(S3): chThdSleepMilliseconds(ms)
    (void)ms;
}

void Scheduler::delay_microseconds(uint16_t us)
{
    // TODO(S3): chThdSleepMicroseconds(us)
    (void)us;
}

void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    // TODO(S3): add to a timer-process list serviced by a periodic thread/VT
    (void)proc;
}

void Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
    // TODO(S3): add to an io-process list serviced by a low-priority thread
    (void)proc;
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    // TODO(S3): run the failsafe callback from the timer thread/VT
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
    // TODO(S3): trigger a real R5F reset
    (void)hold_in_bootloader;
    for (;;) { }
}

bool Scheduler::in_main_thread() const
{
    // TODO(S3): compare against the stored main thread handle
    return true;
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
