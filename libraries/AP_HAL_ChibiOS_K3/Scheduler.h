#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_K3_Namespace.h"

/*
  Scheduler for the AM67/K3 ChibiOS backend.

  M3 scope: enough for the UART_test example. init() brings up ChibiOS
  (halInit + chSysInit); delay/delay_microseconds use chThdSleep*. The timer/io
  process lists are stored but NOT yet serviced (UART_test does not need periodic
  callbacks) — a later step adds the timer thread/VT that runs them.
*/
class ChibiOS_K3::Scheduler : public AP_HAL::Scheduler {
public:
    void init() override;
    void delay(uint16_t ms) override;
    void delay_microseconds(uint16_t us) override;
    void register_timer_process(AP_HAL::MemberProc) override;
    void register_io_process(AP_HAL::MemberProc) override;
    void register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) override;
    void set_system_initialized() override;
    bool is_system_initialized() override;
    void reboot(bool hold_in_bootloader = false) override;
    bool in_main_thread() const override;
    bool thread_create(AP_HAL::MemberProc proc, const char *name,
                        uint32_t stack_size, priority_base base,
                        int8_t priority) override;

private:
    bool _initialized;
    // ChibiOS main thread handle (opaque thread_t*), set in init().
    void *_main_thread;

    static void _thread_trampoline(void *ctx);
};
