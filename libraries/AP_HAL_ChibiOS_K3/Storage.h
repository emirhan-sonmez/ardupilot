#pragma once

#include "AP_HAL_ChibiOS_K3.h"

/*
  M4. Persistent parameter storage backed by a Linux file, reached over the
  shared-memory window that already carries MAVLink (DR-016).

  Replaces Empty::Storage, which read back zeros and discarded every write --
  configuration appeared to save and silently did not, so nothing survived a
  reload and no calibration could be trusted.

  See hwdef/boot/ipc_storage.h for the wire contract, and for why the backing
  store is not the onboard 24c32 EEPROM (it shares WKUP_I2C0 with the PMIC)
  and not the SD card (the R5F cannot reach that controller either).
*/
class ChibiOS_K3::Storage : public AP_HAL::Storage
{
public:
    void init() override;
    void read_block(void *dst, uint16_t src, size_t n) override;
    void write_block(uint16_t dst, const void *src, size_t n) override;

    /*
      Reports whether the daemon attached, NOT whether writes have landed.
      An unhealthy store is one that cannot persist at all; a store with a
      flush still in flight is healthy and merely behind.
    */
    bool healthy(void) override;

    /*
      Deliberately NOT overriding get_storage_ptr(). Handing ArduPilot a raw
      pointer into the shared window would let it write the image without
      going through write_block(), so dirty_seq would never advance and the
      daemon would never persist the change -- a silent regression to exactly
      the Empty::Storage behaviour this class exists to remove.
    */
};
