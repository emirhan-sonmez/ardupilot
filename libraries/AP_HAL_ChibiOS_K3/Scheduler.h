#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_K3_Namespace.h"

/*
  Scheduler for the AM67/K3 ChibiOS backend.

  init() brings up ChibiOS (halInit + chSysInit); delay/delay_microseconds use
  chThdSleep*.

  register_timer_process() and register_io_process() are serviced by two threads
  started in init(). They used to be silent no-ops ("stored but not serviced"),
  which was the root cause of Q-32: AP_Param drains its 30-deep deferred save
  queue from an IO process (AP_Param.cpp:1567), so with the registration ignored
  the queue filled and never emptied. AP_Param::save() then spun forever in
  delay_microseconds() waiting for a free slot -- its only early-out requires the
  vehicle to be armed -- and the main loop never returned. It presented as a hard
  hang: trace frozen, PWM latched at the last commanded value, QGC comms lost,
  while the RTOS and every other thread stayed perfectly healthy.

  A registration that is accepted and silently never run is the dangerous shape
  here. Anything that registers and assumes it runs (StorageManager, AP_Terrain,
  AP_CANManager, AP_Filesystem, AP_SmartRTL, the telemetry backends, ToneAlarm,
  AP_Camera) fails the same quiet way.
*/
class ChibiOS_K3::Scheduler : public AP_HAL::Scheduler
{
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
    // Matches AP_HAL_ChibiOS. Nothing on this board comes close to either
    // limit today; overflow is reported to the trace rather than silently
    // dropped, because a dropped registration is exactly the Q-32 failure.
    static const uint8_t MAX_TIMER_PROCS = 8;
    static const uint8_t MAX_IO_PROCS = 8;

    bool _initialized;
    // ChibiOS main thread handle (opaque thread_t*), set in init().
    void *_main_thread;

    AP_HAL::MemberProc _timer_proc[MAX_TIMER_PROCS];
    uint8_t _num_timer_procs;
    bool _in_timer_proc;

    AP_HAL::MemberProc _io_proc[MAX_IO_PROCS];
    uint8_t _num_io_procs;
    bool _in_io_proc;

    AP_HAL::Proc _failsafe;

    static void _thread_trampoline(void *ctx);
    static void _timer_thread(void *arg);
    static void _io_thread(void *arg);
    void _run_timers();
    void _run_io();
};
