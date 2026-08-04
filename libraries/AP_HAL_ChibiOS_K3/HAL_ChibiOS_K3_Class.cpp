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
#include "Storage.h"
#include "bench_passthrough.h"
#include "bench_imu.h"
#include <AP_RCProtocol/AP_RCProtocol.h>   // AP::RC(), for the rc health line
#include <AP_Arming/AP_Arming.h>           // AP::arming(), for the arm-state trace line
#include <AP_Motors/AP_Motors_Class.h>     // AP_Motors::get_singleton(), for the ctl trace line
#include <RC_Channel/RC_Channel.h>         // rc(), calibrated channel values
#include <AP_AHRS/AP_AHRS.h>               // AP::ahrs(), attitude for the ctl trace line
#include <SRV_Channel/SRV_Channel.h>       // SRV_Channels::get_emergency_stop()
#include <AP_Notify/AP_Notify.h>           // AP_Notify::flags.flight_mode
#include <hal.h>   // for the ChibiOS SerialDriver SD1
#include "hwdef/boot/trace.h"  // RemoteProc trace buffer (readable without UART)
#include "hwdef/boot/ipc_ring.h"  // MAVLink transport to Linux (DR-016)
#include "hwdef/boot/stack_paint.h"  // Q-25: SYS/main-thread stack high-water mark
#include <am67_mailbox.h>
#include <am67_wdt.h>   // M9: MCU RTI windowed watchdog  // remoteproc shutdown handshake with Linux

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
/* M4: real persistent storage over the shared-memory window (Storage.cpp).
   Empty::Storage read back zeros and discarded every write, so parameters
   appeared to save and silently did not. */
static ChibiOS_K3::Storage storageDriver;
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

/*
  Inbound remoteproc mailbox messages from Linux. ISR context, must not block.

  SAFETY, AND THE ORDERING IS NOT NEGOTIABLE: park the outputs first, ack
  second. A stopped R5F leaves the PWM peripherals emitting their last
  commanded pulse width indefinitely (Safety and Failsafes 3.1), and there is
  no watchdog -- M9 is open and the device tree exposes no watchdog device at
  all -- so nothing downstream will catch it.

  Today the kernel's reset assert fails (Q-06) and the core keeps running, so
  parking is what actually happens rather than a race against reset. Keep it
  that way regardless: a shutdown request is a request for a safe output state,
  whether or not the stop that follows succeeds, and the ordering must already
  be correct on the day the reset starts working.
*/
static bool mailbox_message(uint32_t msg)
{
    switch (msg) {
    case RP_MBOX_SHUTDOWN:
        rcoutDriver.park_all_disarmed();

        /* Trace BEFORE the ack, not after. The ack starts a hard ~2ms deadline
           (see below) and trace_printf can compact the 16 KiB buffer, which is
           an interrupts-off bulk copy of uncached DDR -- easily enough to blow
           it. Nothing after the ack may be slow. */
        trace_printf("AP-K3: mbox SHUTDOWN -> outputs parked, ack sent\n");

        /* Deliberately unchecked: if the TX FIFO is full the stop simply times
           out as it did before this existed. There is no useful recovery from
           an ISR, and retrying in a loop here is the one thing that could make
           matters worse. */
        (void)mailbox_send(RP_MBOX_SHUTDOWN_ACK);

        /* We ack and keep running, on purpose. k3_r5_rproc_stop() then polls
           is_core_in_wfi() against a ~2ms deadline and returns -ETIMEDOUT, so
           `stop` still fails -- but it fails in ~7ms instead of blocking 25s
           waiting for an ack that never comes, and the firmware survives it.

           Halting in WFI here does make `stop` return 0, but `start` cannot
           bring the core back: TI SCI refuses this core's module reset in both
           directions with -ENODEV (Q-06). So halting trades a failed stop for a
           dead core that needs a power cycle, which is strictly worse. Do not
           reintroduce the halt until the reset refusal is solved. */
        break;

    case RP_MBOX_ECHO_REQUEST:
        /* Free liveness probe from Linux that does not depend on the trace
           buffer or the IPC ring -- both of which froze together during Q-32,
           leaving no way to distinguish a stalled main loop from a dead core. */
        (void)mailbox_send(RP_MBOX_ECHO_REPLY);
        break;

    default:
        /* Trace and ignore. Never act on an unrecognised id: the kernel also
           sends suspend-related messages this port does not implement, and
           guessing at them risks parking the aircraft's outputs mid-flight. */
        trace_printf("AP-K3: mbox unhandled msg=0x%x\n", msg);
        break;
    }

    return false;
}

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

    /* M4. Nothing in ArduPilot's generic path calls hal.storage->init():
       AP_HAL_ChibiOS gets away with that by opening storage lazily inside
       read_block()/write_block(). This backend needs a real init, because it
       has to wait for the Linux daemon to publish the image before AP_Param
       reads it -- a read that lands early returns zeros, which ArduPilot
       cannot distinguish from a blank EEPROM, so it would format the store
       and write defaults over the saved parameters.

       Must therefore run before callbacks->setup(), and before anything else
       touches a parameter. */
    storageDriver.init();

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

    /* Answer the Linux remoteproc shutdown request. Without this,
       `echo stop > /sys/class/remoteproc/remoteprocN/state` blocks ~25s and
       fails -EBUSY, so loading new firmware required a full power cycle --
       compounded by Q-39, where `reboot` does not restart the SoC either.
       Installed after set_exclusive_mask() because the handler parks the
       outputs through write_exclusive() and must be the accepted writer. */
    mailbox_init(mailbox_message);

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

    /* The INS probe below gets exactly one attempt, and on a cold power cycle
       the bus is still Linux's when setup() runs. Wait for the unbind first --
       see wait_for_imu_bus(). Delays boot by however long userspace takes to
       reach gemstone-r5f-setup.service; PWM outputs hold their disarmed idle
       throughout, and there are no propellers on this airframe yet. */
    ChibiOS_K3::wait_for_imu_bus(HAL_GEMSTONE_IMU_BUS_WAIT_MS);
#else
    ChibiOS_K3::bench_imu_init();
#endif

    /* Prove the AP_HAL SPI path independently of bench_imu.cpp's direct SPID1
       access, before AP_InertialSensor is given anything that depends on it.
       Runs after bench_imu_init() so the two cannot be confused for each other
       in the trace, and so a failure here against a bench_imu success points
       squarely at this layer rather than at the bus. */
    spiDeviceManager.selftest();

    /* Q-05: which barometer is really on CS1. Runs here for the same reason as
       selftest() -- after the bus wait, before setup(), while nothing else
       holds the controller. */
    spiDeviceManager.baro_ident();

    /*
      M9 step 1: measure RTICLK, do NOT arm.

      The DWWD timeout is (PRLD + 1) * 2^13 / RTICLK, and RTICLK for MCU_RTI is
      set by device-tree clock parents this firmware neither configures nor can
      read back. Arming against a guessed rate either never fires or resets the
      board in a loop -- and a reset loop on a board whose only recovery is a
      physical power cycle (Q-39) is an expensive way to learn the clock.
      Report it, then arm in a later build with a number rather than a guess.
    */
    {
        const uint32_t hz = am67_wdt_measure_clock();
        if (hz == 0U) {
            trace_printf("AP-K3: wdt: RTI down-counter did not advance -- "
                         "module not clocked, or not ours\n");
        } else {
            trace_printf("AP-K3: wdt: MCU_RTI clock ~%u Hz, status=%x. "
                         "1s timeout would need PRLD=%u\n",
                         hz, am67_wdt_status(),
                         (uint32_t)((hz / 8192U) - 1U));
        }
    }

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
        //
        // Disabled by default as of 2026-08-03 (PT_ENABLE): ArduCopter's own
        // AP_Motors mixer drives the outputs now, so stabilisation computed
        // from the IMU and EKF3 actually reaches the pins instead of being
        // dropped by the exclusive mask. Guarded rather than deleted -- see
        // bench_passthrough.h.
#if PT_ENABLE
        ChibiOS_K3::bench_passthrough_update();
#endif

        // Sensor read-out, rate-limited internally (50 Hz sample, 0.2 Hz
        // trace line). No-op until bench_imu_init() found the part, and
        // compiled out entirely when the real backend owns the bus.
#if !HAL_GEMSTONE_INS_ICM20948
        ChibiOS_K3::bench_imu_update();
#endif

#if IMU_BUS_DIAG_LENSWEEP
        /* TEMP-DIAG(Q-35): one shot, 30 s in. Deliberately not at boot: the
           R5F is started by remoteproc before Linux userspace unbinds
           omap2_mcspi, so anything touching the bus that early fails against
           a controller Linux still owns. By 30 s the unbind has long since
           run and the INS backend is settled, and the sweep takes the bus
           semaphore per transaction so the two cannot interleave.
           Costs a large one-off dtmax spike. PROPELLERS OFF.
           REMOVE-AFTER: Q-35 closed. */
        static bool lendiag_done;
        if (!lendiag_done && (AP_HAL::millis() > 30000U)) {
            lendiag_done = true;
            /* AP_HAL path only. An earlier revision also called
               bench_imu_bus_diag() here to bring the part up first, which was
               correct only while the INS probe was failing and nothing owned
               CS3. Now that wait_for_imu_bus() lets the backend attach, that
               call reset the part underneath a live backend and drove SPID1
               with no bus lock -- two masters on one chip select, measured as
               27/256 reads returning 0x00. bus_length_diag() takes the bus
               semaphore per transaction and needs no bring-up of its own. */
            spiDeviceManager.bus_length_diag();
        }
#endif

        // SD1 TX no longer has a pad at all: MAVLink moved to the rings
        // (DR-016), and as of 2026-08-03 the epwm0-gpio5-gpio14 overlay takes
        // pin 8 for EHRPWM0_B and reconfigures main_uart1 to an RX-only pin
        // group. UART1 is receive-only hardware now -- pin 10, iBus, owned by
        // RCInput.
        //
        // Kept anyway, as a drain rather than a pump: the THRE interrupt does
        // not fire on this UART (Q-26), so a write to SD1 from anywhere would
        // otherwise fill the TX queue and block its writer forever. With no TX
        // pad those bytes cannot reach a wire either way, so this exists to
        // ensure that mistake fails harmlessly instead of hanging the main
        // loop. A non-zero tx= in the alive line means someone is writing to a
        // port that physically cannot transmit.
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
            /* Raw iBus channels 1-6, straight from AP_RCProtocol, before any
               RCMAP/RC_CHANNELS interpretation. Added 2026-08-03 for two
               questions the single thr= could not answer: which stick actually
               drives which channel (thr= had been pinned at 1000 while the
               others moved), and whether the arm switch reaches the board at
               all. A switch that never changes its number here is a
               transmitter or mixing problem, not a firmware one. */
            trace_printf("AP-K3: rcch %u:%u %u:%u %u:%u %u:%u %u:%u %u:%u\n",
                         1U, (uint32_t)AP::RC().read(0),
                         2U, (uint32_t)AP::RC().read(1),
                         3U, (uint32_t)AP::RC().read(2),
                         4U, (uint32_t)AP::RC().read(3),
                         5U, (uint32_t)AP::RC().read(4),
                         6U, (uint32_t)AP::RC().read(5));
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

            /* Arming state, so "the motors will not spin" is answerable
               without a ground station. Since PT_ENABLE=0 handed the outputs
               to AP_Motors, nothing reaches a pin until the VEHICLE is armed
               -- and the vehicle refuses until prearm passes. The individual
               refusal reasons are traced by AP_Arming::check_failed(), which
               only runs when something asks it to arm; this line reports the
               standing state either way.

               armed=  the soft-armed flag AP_Motors gates its output on.
               prearm= result of the last prearm run: 0 means something is
                       refusing, and the AP-K3: PreArm: lines say what. */
            /* utilInstance, not hal.util: this translation unit defines the
               HAL, so the global `hal` reference does not exist here. */
            trace_printf("AP-K3: arm armed=%u prearm=%u\n",
                         (uint32_t)utilInstance.get_soft_armed(),
                         (uint32_t)AP::arming().get_last_prearm_checks_result());

            /* Commanded pulse width per motor, read back from RCOutput.
               Separates three failures that all look like "it does not
               stabilise" from outside:

                 all four equal and moving together with the throttle stick
                   -> the mixer is running but contributing no attitude
                      correction (spool state, or a zero attitude error)
                 all four equal and NOT moving
                   -> nothing is driving the outputs at all
                 four different values that change when the frame is tilted
                   -> stabilisation IS working and the problem is downstream,
                      in ESC calibration or motor wiring

               Cheap: four cached reads, no hardware access. */
            trace_printf("AP-K3: mot ch0=%u ch1=%u ch2=%u ch3=%u\n",
                         (uint32_t)rcoutDriver.read(0),
                         (uint32_t)rcoutDriver.read(1),
                         (uint32_t)rcoutDriver.read(2),
                         (uint32_t)rcoutDriver.read(3));

            /* Why the motors sit on their floor. Everything here is a library
               singleton, so this costs the HAL no dependency on vehicle code.

                 ctlin=  RC_Channels' CALIBRATED throttle, 0-1000. This is what
                         the vehicle actually uses, as opposed to the raw
                         microseconds in the rcch line above. Raw sweeping
                         1000-2000 while this stays 0 means the RCn_MIN/MAX/
                         REVERSED mapping is wrong, not the receiver.
                 thrin=  AP_Motors' filtered throttle demand, x1000. Zero while
                         ctlin is non-zero puts the fault between the vehicle's
                         throttle handling and the mixer -- flight mode, or the
                         spool/landing gate.
                 spool=  0 SHUT_DOWN, 1 GROUND_IDLE, 2 SPOOLING_UP,
                         3 THROTTLE_UNLIMITED, 4 SPOOLING_DOWN. Attitude mixing
                         only exists in state 3; anything else outputs every
                         motor at the same value no matter how the frame is
                         tilted, which is indistinguishable from "stabilisation
                         is broken" without this number.
                 roll/pitch= AHRS attitude in degrees. If these do not move when
                         the frame is tilted, the estimate is the problem and
                         nothing downstream can work. */
            {
                const AP_Motors *mot = AP_Motors::get_singleton();
                const RC_Channel *thr_ch = rc().channel(2);
                /* estop/mode, 2026-08-03. spool stuck at 1 (GROUND_IDLE) while
                   armed at full throttle means ap.throttle_zero is being held
                   true, and in STABILIZE the only things that do that
                   independently of the throttle stick are the motor emergency
                   stop and the motor interlock. Both are RC-option driven, so
                   a switch sitting in the wrong position silently pins the
                   motors at idle with no message anywhere.

                   mode= is AP_Notify's flight mode number (Copter: 0 STABILIZE,
                   2 ALT_HOLD, 5 LOITER). Any mode other than 0 changes what the
                   throttle stick means, which looks identical to this from the
                   bench. */
                /* desired vs actual spool, plus the interlock, 2026-08-03.
                   spool stuck at GROUND_IDLE has exactly two possible causes
                   and these two numbers separate them:

                     desired=1 -> the VEHICLE is asking for ground idle, i.e.
                                  copter.ap.throttle_zero is true. That flag is
                                  only held true (with a non-zero throttle and
                                  no e-stop) when a channel is assigned
                                  MOTOR_INTERLOCK and its switch is off.
                     desired=3 but spool=1 -> the vehicle wants full range and
                                  AP_Motors is refusing. update_spool_state()
                                  will not leave GROUND_IDLE while _interlock
                                  is false, and Copter clears that on
                                  in_arming_delay, the interlock switch, or the
                                  e-stop (motors.cpp:94).

                   intlk= is AP_Motors' own interlock flag, the one the state
                   machine actually reads. */
                const AP_Motors *m2 = AP_Motors::get_singleton();
                trace_printf("AP-K3: ctl2 estop=%u mode=%u desired=%u intlk=%u\n",
                             (uint32_t)SRV_Channels::get_emergency_stop(),
                             (uint32_t)AP_Notify::flags.flight_mode,
                             m2 != nullptr ? (uint32_t)m2->get_desired_spool_state() : 99U,
                             m2 != nullptr ? (uint32_t)m2->get_interlock() : 99U);
                trace_printf("AP-K3: ctl ctlin=%d thrin=%d spool=%u roll=%d pitch=%d\n",
                             thr_ch != nullptr ? (int32_t)thr_ch->get_control_in() : -1,
                             mot != nullptr ? (int32_t)(mot->get_throttle() * 1000.0f) : -1,
                             mot != nullptr ? (uint32_t)mot->get_spool_state() : 99U,
                             (int32_t)AP::ahrs().get_roll_deg(),
                             (int32_t)AP::ahrs().get_pitch_deg());
            }
            /* pwmblk= (Q-34): RCOutput writes rejected by the exclusive mask,
               i.e. AP_Motors/SRV_Channels attempts to drive the motor pins.

               With PT_ENABLE=0 (the default since 2026-08-03) the mask is 0
               and the expected value is a FROZEN 0 -- AP_Motors owns the pins
               and nothing is being dropped. A climbing pwmblk now means the
               passthrough was compiled back in and is stealing the outputs
               from the vehicle's mixer.

               With PT_ENABLE=1 it inverts: climbing at roughly loop rate x 4
               is the proof that a second writer really was competing, and a
               frozen 0 while the scope dances would mean the competing writer
               is something else and the fix is aimed wrong. */
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

const AP_HAL::HAL& AP_HAL::get_HAL()
{
    return hal_chibios_k3;
}

AP_HAL::HAL& AP_HAL::get_HAL_mutable()
{
    return hal_chibios_k3;
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
