#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include "Scheduler.h"
#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <AP_Math/AP_Math.h>
#include "hwdef/boot/trace.h"

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

    // Q-25: chSysInit() leaves this thread at the ChibiOS default NORMALPRIO.
    // thread_create() below ignores `base` and computes NORMALPRIO + priority
    // for every thread it creates (see its TODO comment) -- so any caller
    // passing a positive offset (e.g. AP_Logger's start_io_thread(),
    // PRIORITY_IO with offset 1) ends up ABOVE the main thread instead of
    // below it. Stock AP_HAL_ChibiOS avoids this by explicitly boosting the
    // main thread to APM_MAIN_PRIORITY (180, well above its IO tier); this
    // port has no such tiering yet, so mirror the same fix at minimum scope:
    // give main enough headroom that no currently-used thread_create() offset
    // can equal or exceed it. Root-caused via hardware trace: the log_io
    // thread (created mid-setup() at NORMALPRIO+1) was silently outranking
    // and starving the main loop, matching the exact "loops=1 forever"
    // symptom -- setup() completed, one loop() iteration ran, then nothing.
    chThdSetPriority((tprio_t)constrain_int32((int32_t)NORMALPRIO + 10,
                                               (int32_t)LOWPRIO, (int32_t)HIGHPRIO));
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

/*
  Trampoline: ChibiOS thread entry points are plain C functions (void
  (*)(void*)), but AP_HAL::MemberProc is a bound-member functor. Take a
  heap copy of the functor (it must outlive thread_create() returning),
  invoke it, then free the copy -- this thread's stack frame is gone the
  instant io_thread()/etc. returns, so nothing after that point may touch
  the functor. Mirrors AP_HAL_ChibiOS's thread_create_trampoline.
*/
void Scheduler::_thread_trampoline(void *ctx)
{
    AP_HAL::MemberProc *proc = (AP_HAL::MemberProc *)ctx;
    (*proc)();
    free(proc);
}

bool Scheduler::thread_create(AP_HAL::MemberProc proc, const char *name,
                               uint32_t stack_size, priority_base base,
                               int8_t priority)
{
    (void)base;  // TODO: per-class priority tuning once more than one
                 // thread class (PRIORITY_IO) actually uses thread_create()
                 // on this board.

    AP_HAL::MemberProc *tproc = (AP_HAL::MemberProc *)malloc(sizeof(proc));
    if (tproc == nullptr) {
        trace_printf("rtos: thread_create(%s) FAILED functor alloc\n", name);
        return false;
    }
    *tproc = proc;

    const tprio_t thd_priority = (tprio_t)constrain_int32(
        (int32_t)NORMALPRIO + priority, (int32_t)LOWPRIO, (int32_t)HIGHPRIO);

    trace_printf("rtos: thread_create(%s) stack=%u prio=%u requested\n",
                 name, (uint32_t)stack_size, (uint32_t)thd_priority);

    thread_t *thd = chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(stack_size),
                                         name, thd_priority,
                                         _thread_trampoline, tproc);
    if (thd == nullptr) {
        free(tproc);
        trace_printf("rtos: thread_create(%s) FAILED chThdCreateFromHeap "
                     "(stack=%u, heap exhausted?)\n", name, (uint32_t)stack_size);
        return false;
    }

    trace_printf("rtos: thread_create(%s) OK\n", name);
    return true;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
