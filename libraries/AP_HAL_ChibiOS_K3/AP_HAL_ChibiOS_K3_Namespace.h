#pragma once

/*
  Namespace for the TI AM67 / K3 Cortex-R5F AP_HAL backend (ChibiOS/RT).

  This backend is built up as a thin vertical slice: initially every AP_HAL
  interface is served by an Empty:: stub (see HAL_ChibiOS_K3_Class.cpp), and the
  real, ChibiOS-backed implementations are introduced here one at a time.

  Roadmap (classes to be declared/implemented in this namespace):
    S3 (first boot):  Scheduler, UARTDriver, Semaphore, BinarySemaphore, Util
    H-series:         UARTDriver (multi-instance), SPIDevice/SPIDeviceManager,
                      I2CDevice/I2CDeviceManager, Storage, GPIO, AnalogIn
    later:            RCOutput, RCInput, CANIface (K3 MCAN)
*/

namespace ChibiOS_K3
{
// Real implementations (declared as they land).
class Semaphore;
class BinarySemaphore;
class Scheduler;
class Util;
class UARTDriver;
class IPCUARTDriver;
class RCOutput;
class RCInput;
struct SPIDeviceDesc;
class SPIBus;
class SPIDevice;
class SPIDeviceManager;
class Storage;
}
