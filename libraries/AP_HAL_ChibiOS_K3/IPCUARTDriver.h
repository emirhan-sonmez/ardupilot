#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_K3_Namespace.h"

/*
  AP_HAL serial port backed by the shared-memory rings to Linux
  (hwdef/boot/ipc_ring.c), not by any physical UART.

  This is the MAVLink transport (DR-016). It exists because the aircraft has
  to fly: a wired link to the ground station is not acceptable, wireless means
  Linux has to be in the path (Wi-Fi is SDIO + wl18xx, the R5F cannot reach
  it), and a shared-memory ring gets there without the RSC_VDEV entry and
  mailbox handshake that real RPMsg would need.

  It also removes two problems the physical UART had, rather than working
  around them:

    - RX exists here. ChibiOS_K3::UARTDriver had to refuse reads because pin 10
      belongs to iBus RCInput, so QGC could never talk back.
    - txspace() is 8 KiB, not 64 bytes. GCS_MAVLink defers any message larger
      than the reported space, so with the old queue anything bigger than
      64 bytes -- AUTOPILOT_VERSION among them -- was deferred forever.

  Unlike the UART driver there is no interrupt, no THRE workaround and no
  pump: a write lands in DDR immediately and Linux polls for it. Nothing in
  this class can block.
*/
class ChibiOS_K3::IPCUARTDriver : public AP_HAL::UARTDriver
{
public:
    IPCUARTDriver();

    bool is_initialized() override;
    bool tx_pending() override;
    uint32_t txspace() override;

protected:
    void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    ssize_t _read(uint8_t *buffer, uint16_t count) override;
    void _end() override;
    void _flush() override;
    uint32_t _available() override;
    bool _discard_input() override;

private:
    bool _initialized;

    // begin() diagnostics, bounded. AP_SerialManager and GCS_MAVLINK::init()
    // between them open a port up to four times during boot; the ring must
    // survive all of them without being reset, so tracing the count is how
    // that gets confirmed on hardware rather than assumed.
    uint8_t _begin_count;
};
