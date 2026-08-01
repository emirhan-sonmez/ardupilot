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
#include "IPCUARTDriver.h"
#include "RCOutput.h"
#include "RCInput.h"
#include "SPIDevice.h"
#include "bench_passthrough.h"
#include "bench_imu.h"
#include <AP_RCProtocol/AP_RCProtocol.h>   // AP::RC(), for the rc health line
#include <hal.h>   // for the ChibiOS SerialDriver SD1
#include "hwdef/boot/trace.h"  // RemoteProc trace buffer (readable without UART)
#include "hwdef/boot/ipc_ring.h"  // MAVLink transport to Linux (DR-016)
#include "hwdef/boot/stack_paint.h"  // Q-25: SYS/main-thread stack high-water mark

// --- driver instances ---
// serial0 (SERIAL0) carries MAVLink 2, and as of DR-016 it is NOT a physical
// UART: it is a shared-memory ring pair to Linux (hwdef/boot/ipc_ring.c),
// bridged there to UDP 14550 for QGroundControl. The aircraft has to fly, so
// a wired ground link was rejected; wireless forces Linux into the path
// because Wi-Fi is SDIO + wl18xx and the R5F cannot reach it.
//
// This also settles the pin-10 conflict by removing it. The AM67 port has a
// single physical UART (SD1 = UART1, header pins 8 TX / 10 RX) and MAVLink
// used to share it with iBus, which meant MAVLink was TX-only -- QGC could
// never talk back. SD1 now belongs entirely to ChibiOS_K3::RCInput (iBus on
// pin 10) and is no longer an AP_HAL serial port at all. NOTE: that makes
// RCInput::init() responsible for sdStart()ing it, since AP_SerialManager no
// longer opens it for us.
//
// ChibiOS_K3::UARTDriver is consequently unused right now. It is kept, not
// deleted: it is the working, hardware-verified serial backend and it is what
// a SiK telemetry radio on a second UART would use (see [[MAVLink and
// QGroundControl]] -- Wi-Fi is a bench/config link, not a flight link).
//
// serial1-9 have no wired hardware yet -> Empty:: (null) stubs.
//
// hal.console is a SEPARATE Empty:: instance, not aliased to serial0: nothing
// may write plain text into the MAVLink byte stream. Boot/diagnostic
// breadcrumbs go to the RemoteProc trace buffer (trace_printf) instead,
// readable at /sys/kernel/debug/remoteproc/remoteprocN/trace0.
static ChibiOS_K3::IPCUARTDriver serial0Driver;
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
static ChibiOS_K3::SPIDeviceManager spiDeviceManager;
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

    /* serial0 is the MAVLink port and is backed by the shared-memory rings to
       Linux (DR-016), not by a UART. Established here, before anything can
       write to it, rather than being left to AP_SerialManager's begin():
       ipc_ring_init() bumps the epoch and clears the indices, and doing that
       later -- after GCS_MAVLINK has begun streaming, or worse, four times
       over as the boot path reopens the port -- would tear the stream under a
       Linux bridge that had already attached. begin() is idempotent for
       exactly this reason. */
    ipc_ring_init();

    /* AP_BoardConfig::board_setup() would normally call hal.rcin->init() (and
       gpio/rcout) but that path is gated to `#if CONFIG_HAL_BOARD ==
       HAL_BOARD_CHIBIOS` (the stock ChibiOS HAL's board ID, not ours) --
       see board_drivers.cpp. Call it explicitly here instead. */
    rcin->init();
    trace_printf("AP-K3: rcin->init done (iBus on SD1 RX, pin 10)\n");

    /* PWM safety (M2): SERVOx_FUNCTION defaults to disabled and Storage is
       Empty:: (nothing persists), so SRV_Channels will not touch any output
       on its own this milestone -- real motor/servo assignment (FRAME_CLASS/
       FRAME_TYPE) is out of scope here. Explicitly set every real RCOutput
       channel to a safe 1000 us idle and enable it, once, before vehicle
       setup(). No arming, no cycling, no 7th channel (EHRPWM0_B/pin 8 stays
       untouched). */
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

    /* Q-34 root-cause fix: hand the PWM channels to bench_passthrough
       exclusively, from here on. Set AFTER the safe-init above (which uses the
       ordinary write() path and must still be allowed through) and BEFORE
       callbacks->setup(), so ArduCopter's AP_Motors init-time writes are
       blocked too, not just its per-tick ones.

       DR-012's "bench_passthrough runs last every tick" was not enough and
       could never be: CMPA/CMPB load from shadow at CTR=ZERO, once per 20ms
       period, at a phase unrelated to the main loop, so the pin takes
       whichever writer touched the shadow last before that load -- not
       whichever ran last in the iteration. See RCOutput.h,
       set_exclusive_mask(). */
    rcoutDriver.set_exclusive_mask(ChibiOS_K3::PT_EXCLUSIVE_MASK);

    /*
      Bench ICM-20948 bring-up on MCU_MCSPI0 CS3. Before setup(), so a wrong
      chip select or a bus Linux still owns shows up as its own trace line
      rather than being lost among the vehicle's own init output. Reports and
      returns on failure -- never blocks the boot.

      Mutually exclusive with the real AP_InertialSensor backend
      (HAL_GEMSTONE_INS_ICM20948). bench_imu.cpp drives SPID1 directly with
      spiSelect()/spiPolledExchange() and takes no bus lock, so running it
      alongside the backend's periodic callback would interleave two
      transactions on one chip select. That is the "two masters" failure this
      port already spent a session on at the Linux/R5F boundary; there is no
      reason to recreate it inside the firmware.
    */
#if HAL_GEMSTONE_INS_ICM20948
    trace_printf("AP-K3: bench_imu skipped, AP_InertialSensor backend owns CS3\n");
#else
    ChibiOS_K3::bench_imu_init();
#endif

    /* Prove the AP_HAL SPI path independently of bench_imu.cpp's direct SPID1
       access, before AP_InertialSensor is given anything that depends on it.
       Runs after bench_imu_init() so the two cannot be confused for each other
       in the trace, and so a failure here against a bench_imu success points
       squarely at this layer rather than at the bus. */
    spiDeviceManager.selftest();

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
         thr=     cumulative writes to the AM67 UART1 THR register. Kept, but
                  it no longer says anything about MAVLink: that moved to the
                  shared-memory rings (DR-016) and SD1 is RX-only for iBus
                  now, so this should sit still. See the mav= line below for
                  link health. */
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

        // AQCTLA/B is otherwise written once, at enable_ch() time, and
        // never revisited -- same shape as the TBPRD/frequency bug fixed
        // 2026-07-30. Cheap (a few register writes, no CMPA/CMPB touch),
        // safe every tick.
        rcoutDriver.reassert_outputs();

        callbacks->loop();

        // Bench RC->PWM passthrough for the four quad-X outputs (task 4,
        // stretch). Independent of the vehicle's own loop -- see
        // bench_passthrough.cpp for scope and safety notes. Deliberately
        // AFTER callbacks->loop(): under ArduPlane, Plane's own SRV_Channels
        // output (Plane::set_servos(), an AP_Scheduler fast task) wrote
        // channels 0-3 every tick even with SERVOn_FUNCTION unconfigured
        // (observed on hardware: ch0/ch2 came up at 1500/1100us, not this
        // passthrough's 1000us idle, while it ran first). Switched to
        // ArduCopter 2026-07-30 -- expect the same or a stronger conflict
        // from Copter's own AP_Motors output, since FRAME_CLASS/FRAME_TYPE
        // default to an active quad-X mixer rather than an opt-in
        // SRV_Channels function; not yet re-verified on hardware under
        // Copter. Real motor mixer configuration remains out of scope for
        // this milestone, so rather than configure the vehicle to leave
        // these outputs alone, this runs last and unconditionally
        // overwrites them with the arm-gated value every tick. PROPELLERS
        // OFF.
        ChibiOS_K3::bench_passthrough_update();

        // Sensor read-out, rate-limited internally (50 Hz sample, 0.2 Hz
        // trace line). No-op until bench_imu_init() found the part, and
        // compiled out entirely when the real backend owns the bus.
#if !HAL_GEMSTONE_INS_ICM20948
        ChibiOS_K3::bench_imu_update();
#endif

        // SD1 TX carries nothing since MAVLink moved to the rings (DR-016),
        // so this is now a cheap no-op on an empty queue rather than a
        // load-bearing pump. Kept deliberately: the THRE interrupt still does
        // not fire on this UART (Q-26), so anything that ever writes to SD1
        // again -- a SiK radio on this port, a debug console -- would strand
        // its bytes without it, and the failure would be silent.
        const uint32_t tx_queued = am67_uart1_tx_pump();

        static uint32_t loops;
        static uint32_t last_report_ms;
        static uint32_t last_tick_ms;
        static uint32_t dt_max_ms;
        static uint32_t last_rc_bytes;
        loops++;
        const uint32_t now_ms = AP_HAL::millis();

        /* Worst-case iteration time in the reporting window. The RX queue holds
           64 bytes = ~15.4ms of iBus (see RCInput::update()), so any iteration
           above that drops receiver bytes and desynchronises the decoder while
           num_channels() stays latched -- frozen sticks that no failsafe can
           see. dtmax is the direct test for that, and the first thing to read
           if control is lost minutes into a run. */
        if (last_tick_ms != 0) {
            const uint32_t dt = now_ms - last_tick_ms;
            if (dt > dt_max_ms) {
                dt_max_ms = dt;
            }
        }
        last_tick_ms = now_ms;

        if (now_ms - last_report_ms >= 5000) {
            last_report_ms = now_ms;
            const uint32_t rc_bytes = rcinDriver.bytes_seen();
            /* trcomp is the compaction COUNT, reported directly rather than
               inferred from trcdrop arithmetic. Q-32 kills the firmware at what
               looks like exactly two compactions on two different boards, but
               that was reconstructed after the fact from byte counts; this
               separates "died at the Nth compaction" from "died N seconds in",
               which the compaction differential turns on. */
            trace_printf("AP-K3: rc dtmax=%ums rcb=%u/5s rcch=%u thr=%u trcdrop=%u trcomp=%u\n",
                         dt_max_ms, rc_bytes - last_rc_bytes,
                         (uint32_t)AP::RC().num_channels(),
                         (uint32_t)AP::RC().read(2),
                         trace_bytes_dropped(),
                         trace_compaction_count());
            last_rc_bytes = rc_bytes;
            dt_max_ms = 0;
            /* MAVLink link health, all from the shared-memory rings (DR-016).
               These three numbers separate failures that otherwise look
               identical from QGC's side ("no vehicle"):
                 txq=      bytes queued towards Linux. Climbing and staying
                           high means the daemon is not draining -- either it
                           died or it never started.
                 refused=  bytes GCS_MAVLink was denied for lack of room.
                           Non-zero at all means the above went on long
                           enough to overrun 8 KiB.
                 host=     the daemon's own liveness counter, which the R5F
                           never writes. Frozen while txq climbs is a dead
                           bridge; advancing while QGC sees nothing puts the
                           fault in the network, not on this board. */
            ipc_ring_tick();
            trace_printf("AP-K3: mav txq=%u refused=%u host=%u\n",
                         ipc_ring_tx_pending(), ipc_ring_tx_refused(),
                         ipc_ring_host_alive());
            /* pwmblk= (Q-34): RCOutput writes rejected by the exclusive mask,
               i.e. AP_Motors/SRV_Channels attempts to drive the motor pins.
               Climbing at roughly loop rate x 6 is the direct proof that a
               second writer really was competing for these outputs; frozen at
               0 while the scope still dances would mean the competing writer
               is something else and this fix is aimed wrong. */
            trace_printf("AP-K3: alive t=%ums loops=%u txq=%u pwmblk=%u stackhw=%u/%u uart[notify=%u isr=%u thre=%u fifo=%u deq=%u thr=%u]\n",
                         now_ms, loops, tx_queued,
                         rcoutDriver.foreign_writes_blocked(),
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
