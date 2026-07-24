#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_K3_Namespace.h"

/*
  Scheduler for the AM67/K3 ChibiOS backend.

  NOTE (S2): the .cpp bodies are minimal stubs so the tree compiles and links.
  They are NOT functional (delay does not delay, timer/io procs are stored but
  never run, threads are not created). S3 replaces them with real ChibiOS/RT
  logic (chThdSleep* for delay, chThdCreateStatic for threads, a virtual-timer
  or dedicated thread for the timer/io process lists), which needs ch.h from the
  chibios_k3 make-integration.
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

private:
    bool _initialized;
};
