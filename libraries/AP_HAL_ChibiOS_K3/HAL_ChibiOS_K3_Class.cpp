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
#include <hal.h>   // for the ChibiOS SerialDriver SD1
#include <ch.h>    // chnWriteTimeout / TIME_IMMEDIATE for the direct SD1 probe
#include "hwdef/boot/trace.h"  // RemoteProc trace buffer (readable without UART)

// --- driver instances ---
// The AM67 port implements a single physical UART (SD1 = UART1, 40-pin header
// pins 8/10). Only AP serial0 (console) maps to it. serial1-9 have no wired
// hardware yet -> Empty:: (null) stubs, so nothing else contends for SD1.
static ChibiOS_K3::UARTDriver serial0Driver((void *)&SD1);
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
static Empty::RCInput rcinDriver;
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
        &serial0Driver,    // console
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
       breadcrumbs then show how far boot gets even if the console is silent. */
    trace_init();
    trace_printf("AP-K3: run() entry, build %s %s\n", __DATE__, __TIME__);

    /* Initialise drivers in a sane order. Scheduler first, then the console. */
    scheduler->init();               // halInit() + chSysInit()
    trace_printf("AP-K3: scheduler->init done\n");

    /* Open the console at 57600 to match the baud the example's setup()
       re-opens with (and the user's terminal). */
    serial(0)->begin(57600);
    trace_printf("AP-K3: serial0 begun\n");

    /* UART diagnosis: write directly to the ChibiOS SD1, bypassing the AP
       console/BetterStream path, and report SD1 state + how many bytes the
       driver accepted. This isolates "is SD1 TX actually working" from "is
       hal.console wired/flushing". TIME_IMMEDIATE so it never blocks. */
    {
        static const char probe[] = "\r\nAP-K3 direct SD1 probe\r\n";
        size_t n = chnWriteTimeout(&SD1, (const uint8_t *)probe,
                                   sizeof(probe) - 1U, TIME_IMMEDIATE);
        trace_printf("AP-K3: SD1 state=%u direct-write accepted=%u bytes\n",
                     (uint32_t)SD1.state, (uint32_t)n);
    }

    callbacks->setup();
    trace_printf("AP-K3: setup() returned\n");
    scheduler->set_system_initialized();

    uint32_t loop_count = 0;
    for (;;) {
        trace_printf("AP-K3: -> loop() #%u\n", loop_count++);
        callbacks->loop();
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
