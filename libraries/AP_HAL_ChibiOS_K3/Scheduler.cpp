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

    _num_timer_procs = 0;
    _num_io_procs = 0;
    _in_timer_proc = false;
    _in_io_proc = false;
    _failsafe = nullptr;

    /* Timer tier sits just ABOVE the main thread (which init() just pinned to
       NORMALPRIO+10), mirroring stock ArduPilot where APM_TIMER_PRIORITY
       outranks APM_MAIN_PRIORITY: a 1 kHz tier that cannot preempt the main
       loop is not a 1 kHz tier. Keep the work in timer callbacks short.

       IO tier sits BELOW main. That ordering is load-bearing, not cosmetic --
       Q-25 was AP_Logger's log_io thread landing at NORMALPRIO+1 while main was
       still at NORMALPRIO, silently outranking it and starving the main loop to
       a standstill. Anything in the IO tier must stay under main. */
    thread_t *timer_thd = chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(2048),
                          "timer",
                          (tprio_t)constrain_int32((int32_t)NORMALPRIO + 11,
                                  (int32_t)LOWPRIO,
                                  (int32_t)HIGHPRIO),
                          _timer_thread, this);
    if (timer_thd == nullptr) {
        trace_printf("rtos: FAILED to create timer thread -- "
                     "register_timer_process() callbacks will NOT run\n");
    }

    thread_t *io_thd = chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(4096),
                                           "io",
                                           (tprio_t)constrain_int32((int32_t)NORMALPRIO + 1,
                                                   (int32_t)LOWPRIO,
                                                   (int32_t)HIGHPRIO),
                                           _io_thread, this);
    if (io_thd == nullptr) {
        /* Loud, because this is precisely how Q-32 presented: the vehicle runs
           normally for minutes and then the main loop stops for good once
           AP_Param's 30-deep save queue fills with nothing draining it. */
        trace_printf("rtos: FAILED to create io thread -- "
                     "register_io_process() callbacks will NOT run, "
                     "AP_Param saves WILL wedge the main loop (Q-32)\n");
    }

    trace_printf("rtos: timer/io threads started (timer=%s io=%s)\n",
                 timer_thd ? "ok" : "FAILED", io_thd ? "ok" : "FAILED");
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

/*
  Both registration calls are idempotent per proc, matching AP_HAL_ChibiOS:
  several callers register from code that can run more than once, and a
  duplicate would run the callback twice per tick.

  A full table is traced rather than ignored. These registrations were no-ops
  for the whole life of the port and the resulting failure (Q-32) took four
  hardware sessions to find precisely because nothing ever said "your callback
  is not running".
*/
void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }
    if (_num_timer_procs >= MAX_TIMER_PROCS) {
        trace_printf("rtos: register_timer_process TABLE FULL (%u), callback DROPPED\n",
                     (uint32_t)MAX_TIMER_PROCS);
        return;
    }
    // The timer thread reads _num_timer_procs without a lock, so publish the
    // slot before the count -- otherwise it can observe a count that includes
    // an entry that has not been written yet.
    _timer_proc[_num_timer_procs] = proc;
    _num_timer_procs++;
}

void Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            return;
        }
    }
    if (_num_io_procs >= MAX_IO_PROCS) {
        trace_printf("rtos: register_io_process TABLE FULL (%u), callback DROPPED\n",
                     (uint32_t)MAX_IO_PROCS);
        return;
    }
    _io_proc[_num_io_procs] = proc;
    _num_io_procs++;
}

void Scheduler::_run_timers()
{
    if (_in_timer_proc) {
        return;
    }
    _in_timer_proc = true;
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i]) {
            _timer_proc[i]();
        }
    }
    if (_failsafe != nullptr) {
        _failsafe();
    }
    _in_timer_proc = false;
}

void Scheduler::_run_io()
{
    if (_in_io_proc) {
        return;
    }
    _in_io_proc = true;
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i]) {
            _io_proc[i]();
        }
    }
    _in_io_proc = false;
}

/*
  1 kHz timer tier, the rate AP_HAL callers assume for register_timer_process().
*/
void Scheduler::_timer_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (true) {
        chThdSleepMicroseconds(1000);
        if (!sched->_initialized) {
            continue;
        }
        sched->_run_timers();
    }
}

/*
  IO tier. 20 ms is ample: the work here is deferred and bursty rather than
  periodic, and each callback drains its own backlog in one call (AP_Param's
  save_io_handler pops the whole queue), so the tier only has to run often
  enough that a 30-deep queue cannot fill between visits.
*/
void Scheduler::_io_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (true) {
        chThdSleepMilliseconds(20);
        if (!sched->_initialized) {
            continue;
        }
        sched->_run_io();
    }
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    /* period_us is ignored: the failsafe runs from the 1 kHz timer tier, which
       is at least as often as any caller asks for. Stock AP_HAL_ChibiOS does
       the same. */
    (void)period_us;
    _failsafe = failsafe;
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
