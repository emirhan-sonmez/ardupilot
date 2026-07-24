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

// --- driver instances (all Empty:: for now) ---
static Empty::UARTDriver serial0Driver;   // console  <- becomes ChibiOS_K3 in S3
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
static Empty::RCOutput rcoutDriver;
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
    /* Initialise drivers in a sane order. Scheduler first, then the console. */
    scheduler->init();
    serial(0)->begin(115200);

    callbacks->setup();
    scheduler->set_system_initialized();

    for (;;) {
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
