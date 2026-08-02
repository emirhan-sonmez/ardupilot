#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include "Storage.h"
#include "hwdef/boot/ipc_storage.h"
#include "hwdef/boot/trace.h"

using namespace ChibiOS_K3;

/*
  How long init() waits for the Linux daemon to publish the image.

  This wait is not a convenience. AP_Param reads the store immediately after
  init(), and an all-zero read is indistinguishable from a genuinely blank
  EEPROM: ArduPilot concludes the storage is unformatted, formats it, and
  writes defaults -- over the top of a perfectly good parameter file that
  simply had not arrived yet. Losing a tuned airframe's parameters to a boot
  race is a far worse failure than booting a few seconds later.

  Same class of one-shot race as the IMU probe (wait_for_imu_bus), and for the
  same underlying reason: this core is started by remoteproc during the
  kernel's own boot, long before Linux userspace is running.
*/
#ifndef HAL_GEMSTONE_STORAGE_WAIT_MS
#define HAL_GEMSTONE_STORAGE_WAIT_MS 60000
#endif

void Storage::init()
{
    ipc_storage_init();

    if (!ipc_storage_wait_ready(HAL_GEMSTONE_STORAGE_WAIT_MS)) {
        /*
          ipc_storage_wait_ready() has already traced the failure loudly.
          Carry on rather than blocking the boot: an aircraft that boots with
          default parameters is recoverable, one that never boots is not.
        */
        return;
    }
}

void Storage::read_block(void *dst, uint16_t src, size_t n)
{
    ipc_storage_read((uint32_t)src, (uint8_t *)dst, (uint32_t)n);
}

void Storage::write_block(uint16_t dst, const void *src, size_t n)
{
    ipc_storage_write((uint32_t)dst, (const uint8_t *)src, (uint32_t)n);
}

bool Storage::healthy(void)
{
    return ipc_storage_ready();
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
