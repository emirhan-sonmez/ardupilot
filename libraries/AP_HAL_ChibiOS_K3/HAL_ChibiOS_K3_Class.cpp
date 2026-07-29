#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include "HAL_ChibiOS_K3_Class.h"

/*
  S2 skeleton (thin vertical slice): every AP_HAL interface is served by an
  Empty:: stub. This is enough for an AP_HAL example to LINK against the
  GemstoneO1R5F board once the ChibiOS make-integration exists.

  S3 replaces the console UARTDriver, Scheduler and Util below with real
  ChibiOS_K3:: implementations backed by the AM67 port (and points
  HAL_Semaphore at ChibiOS_K3::Semaphore in AP_HAL/board/chibios_k3.h).
*/
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

// Our own (non-Empty) implementations
#include "Scheduler.h"
#include "Semaphores.h"
#include "Util.h"
#include "UARTDriver.h"
#include "RCOutput.h"
#include "RCInput.h"
#include "bench_passthrough.h"
#include <hal.h>   // for the ChibiOS SerialDriver SD1
#include "hwdef/boot/trace.h"  // RemoteProc trace buffer (readable without UART)
#include "hwdef/boot/stack_paint.h"  // Q-25: SYS/main-thread stack high-water mark

// --- driver instances ---
// The AM67 port implements a single physical UART (SD1 = UART1, 40-pin header
// pins 8 TX / 10 RX). Its TX side (pin 8) is AP serial0 (SERIAL0), carrying
// MAVLink 2 out -- no GCS is attached this milestone, so outbound-only is
// fine (see MAVLink and QGroundControl notes). Its RX side (pin 10) belongs
// exclusively to ChibiOS_K3::RCInput (iBus in, see RCInput.h/.cpp);
// ChibiOS_K3::UARTDriver's _read()/_available() are disabled so the two
// don't race for the same incoming bytes. serial1-9 have no wired hardware
// yet -> Empty:: (null) stubs.
//
// hal.console is a SEPARATE Empty:: instance, not aliased to serial0: once
// MAVLink starts, nothing may write plain text to the physical UART (it would
// corrupt the MAVLink byte stream). Boot/diagnostic breadcrumbs go to the
// RemoteProc trace buffer (trace_printf) instead, readable at
// /sys/kernel/debug/remoteproc/remoteprocN/trace0.
static ChibiOS_K3::UARTDriver serial0Driver((void *)&SD1);
static Empty::UARTDriver consoleDriver;
static Empty::UARTDriver serial1Driver;
static Empty::UARTDriver serial2Driver;
static Empty::UARTDriver serial3Driver;
static Empty::UARTDriver serial4Driver;
static Empty::UARTDriver serial5Driver;
static Empty::UARTDriver serial6Driver;
static Empty::UARTDriver serial7Driver;
static Empty::UARTDriver serial8Driver;
static Empty::UARTDriver serial9Driver;
static Empty::I2CDeviceManager i2cDeviceManager;
static Empty::SPIDeviceManager spiDeviceManager;
static Empty::WSPIDeviceManager wspiDeviceManager;
static Empty::AnalogIn analogIn;
static Empty::Storage storageDriver;
static Empty::GPIO gpioDriver;
static ChibiOS_K3::RCInput rcinDriver((void *)&SD1);   // real: iBus on SD1 RX, pin 10
static ChibiOS_K3::RCOutput rcoutDriver;   // real: channel 0 -> EPWM0_A -> pin 29
static ChibiOS_K3::Scheduler schedulerInstance;  // real (stub bodies until S3)
static ChibiOS_K3::Util utilInstance;            // real (stub bodies until S3)
static Empty::OpticalFlow opticalFlowDriver;
static Empty::Flash flashDriver;

HAL_ChibiOS_K3::HAL_ChibiOS_K3() :
    AP_HAL::HAL(
        &serial0Driver,
        &serial1Driver,
        &serial2Driver,
        &serial3Driver,
        &serial4Driver,
        &serial5Driver,
        &serial6Driver,
        &serial7Driver,
        &serial8Driver,
        &serial9Driver,
        &i2cDeviceManager,
        &spiDeviceManager,
        &wspiDeviceManager,
        &analogIn,
        &storageDriver,
        &consoleDriver,    // console: deliberately NOT the real UART (see above)
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlowDriver,
        &flashDriver,
        nullptr)           // no CAN yet (K3 MCAN is Phase 3b)
{}

void HAL_ChibiOS_K3::run(int argc, char* const argv[], Callbacks* callbacks) const
{
    (void)argc;
    (void)argv;

    /* --- bring-up diagnostics (M3) ---------------------------------------
       The RemoteProc trace buffer is readable on the Linux host at
       /sys/kernel/debug/remoteproc/remoteprocN/trace0 and does NOT depend on
       the UART pins. A *fresh* build stamp appearing there proves this ELF
       actually loaded (rather than a stale image still running). The
       breadcrumbs then show how far boot gets even if the console is silent.
       Deliberately first, ahead of stack_paint_init(): if anything below
       here hangs or faults, this line still made it out. */
    trace_init();
    trace_printf("AP-K3: run() entry, build %s %s\n", __DATE__, __TIME__);

    /* Q-25: paint the unused portion of the SYS/main-thread stack so the
       heartbeat can report how deep the call chain (AP_AHRS::update() etc.)
       actually reaches. Every frame pushed after this point narrows the
       painted range, so it still wants to run as early as possible. */
    stack_paint_init();
    trace_printf("AP-K3: stack_paint_init done\n");

    /* Initialise drivers in a sane order. Scheduler first. */
    scheduler->init();               // halInit() + chSysInit()
    trace_printf("AP-K3: scheduler->init done\n");

    /* serial0 (SD1 / AM67 UART1, pins 8 TX / 10 RX) is the MAVLink UART for
       this milestone. Leave it unopened here -- AP_SerialManager (inside
       callbacks->setup()) owns begin() at the SERIAL0_BAUD parameter, so it
       is opened exactly once, at the right baud, by the normal vehicle boot
       path. */
    trace_printf("AP-K3: MAVLink UART = serial0 (SD1/UART1, pins 8 TX/10 RX)\n");

    /* AP_BoardConfig::board_setup() would normally call hal.rcin->init() (and
       gpio/rcout) but that path is gated to `#if CONFIG_HAL_BOARD ==
       HAL_BOARD_CHIBIOS` (the stock ChibiOS HAL's board ID, not ours) --
       see board_drivers.cpp. Call it explicitly here instead. */
    rcin->init();
    trace_printf("AP-K3: rcin->init done (iBus on SD1 RX, pin 10)\n");

    /* PWM safety (M2): SERVOx_FUNCTION defaults to disabled and Storage is
       Empty:: (nothing persists), so SRV_Channels will not touch any output
       on its own this milestone -- QuadPlane motor/servo assignment is out
       of scope here. Explicitly set every real RCOutput channel to a safe
       1000 us idle and enable it, once, before vehicle setup(). No arming,
       no cycling, no 7th channel (EHRPWM0_B/pin 8 stays untouched). */
    for (uint8_t ch = 0; ch < 6; ch++) {
        rcout->write(ch, 1000);
    }
    for (uint8_t ch = 0; ch < 6; ch++) {
        rcout->enable_ch(ch);
    }
    for (uint8_t ch = 0; ch < 6; ch++) {
        rcout->write(ch, 1000);
    }
    trace_printf("AP-K3: 6 RCOutput channels safe-initialized at 1000us\n");

    trace_printf("AP-K3: entering vehicle setup()\n");
    callbacks->setup();
    trace_printf("AP-K3: setup() returned\n");

    scheduler->set_system_initialized();

    /* Steady-state health report, low rate (5 s) so it does not flood the
       16 KiB trace buffer. Answers the two open questions at once:
         loops=   main loop rate. ~50-400/5s means wait_for_sample() is
                  pacing correctly; a huge number means _have_sample is
                  never being cleared (ins.update() not running) and the
                  loop is free-running.
         thr=     cumulative writes to the AM67 UART1 THR register. If this
                  keeps climbing while the PC still captures nothing, the
                  bytes are leaving the SoC and the fault is in the pad mux
                  / wiring / adapter, not in software. If it is frozen, the
                  software TX path stopped feeding the UART. */
    for (;;) {
        // Drain iBus bytes before running vehicle code this tick, so
        // read_radio() (an AP_Scheduler fast task) sees fresh data.
        // register_timer_process() is still a no-op on this port (see
        // Scheduler.cpp), so this is hand-pumped here rather than from a
        // dedicated timer thread. A separate thread draining this earlier
        // (started before setup()) was tried and reverted -- it reliably
        // stalled setup() with a receiver connected; this per-tick call
        // does not (root cause of the thread-based stall not understood --
        // see [[Session Notes]]).
        rcinDriver.update();

        // A channel whose peripheral clock wasn't running yet at the
        // boot-time safe-init (i.e. the Linux PWM-enable script, handoff
        // section 6, hadn't been run yet) can recover once it is -- cheap
        // per attempt (~5ms per still-dead peripheral) but rate-limited
        // here so it doesn't cost anything once every channel is up.
        {
            static uint32_t last_retry_ms;
            const uint32_t now_ms = AP_HAL::millis();
            if (now_ms - last_retry_ms >= 2000) {
                last_retry_ms = now_ms;
                rcoutDriver.retry_pending();
            }
        }

        callbacks->loop();

        // Bench RC->PWM passthrough for the four quad-X outputs (task 4,
        // stretch). Independent of the vehicle's own loop -- see
        // bench_passthrough.cpp for scope and safety notes. Deliberately
        // AFTER callbacks->loop(): Plane's own SRV_Channels output
        // (Plane::set_servos(), an AP_Scheduler fast task) also writes
        // channels 0-3 every tick even with SERVOn_FUNCTION unconfigured
        // (observed on hardware: ch0/ch2 came up at 1500/1100us, not this
        // passthrough's 1000us idle, while it ran first) -- QuadPlane mixer
        // configuration is explicitly out of scope for this milestone (see
        // the ArduPilot iBus Port Handoff, section 4c), so rather than
        // configure SRV_Channels to leave these outputs alone, this runs
        // last and unconditionally overwrites them with the arm-gated
        // value every tick. PROPELLERS OFF.
        ChibiOS_K3::bench_passthrough_update();

        // Keep the UART TX draining independently of driver calls (the THRE
        // interrupt is not firing on this UART -- see am67_uart1_tx_pump()).
        const uint32_t tx_queued = am67_uart1_tx_pump();

        static uint32_t loops;
        static uint32_t last_report_ms;
        loops++;
        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_report_ms >= 5000) {
            last_report_ms = now_ms;
            trace_printf("AP-K3: alive t=%ums loops=%u txq=%u stackhw=%u/%u uart[notify=%u isr=%u thre=%u fifo=%u deq=%u thr=%u]\n",
                         now_ms, loops, tx_queued,
                         stack_paint_highwater(), stack_paint_total(),
                         am67_uart1_notify_count, am67_uart1_isr_count,
                         am67_uart1_thre_count, am67_uart1_load_fifo_count,
                         am67_uart1_bytes_dequeued, am67_uart1_thr_writes);
        }
    }
}

static HAL_ChibiOS_K3 hal_chibios_k3;

const AP_HAL::HAL& AP_HAL::get_HAL() {
    return hal_chibios_k3;
}

AP_HAL::HAL& AP_HAL::get_HAL_mutable() {
    return hal_chibios_k3;
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
