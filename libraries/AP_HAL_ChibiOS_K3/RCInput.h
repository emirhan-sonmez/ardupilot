#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_K3_Namespace.h"

/*
  iBus RC input over the shared UART1 (SD1) RX side, 40-pin header pin 10.
  Reads raw bytes directly off the ChibiOS SerialDriver's RX queue and feeds
  them to AP_RCProtocol via AP::RC().process_byte() -- framing, checksum and
  channel extraction are entirely AP_RCProtocol_IBUS's job, not reimplemented
  here (see the ArduPilot iBus Port Handoff note, section 4a).

  This class owns SD1 outright as of DR-016. MAVLink used to share the same
  physical UART (TX on pin 8) and has since moved to the shared-memory rings
  to Linux (IPCUARTDriver), so SD1 is no longer an AP_HAL serial port at all.
  Consequence worth knowing: nothing else in the boot path calls sdStart() on
  it any more, so init() below does -- previously AP_SerialManager's
  serial0->begin() did it as a side effect.
*/
class ChibiOS_K3::RCInput : public AP_HAL::RCInput
{
public:
    // iBus line rate. Fixed by the protocol, not a parameter. Used both to
    // open SD1 and to tell AP_RCProtocol what the line rate is; the two must
    // not be allowed to drift apart.
    static constexpr uint32_t IBUS_BAUD = 115200;

    explicit RCInput(void *serial_driver);

    void init() override;
    bool new_input() override;
    uint8_t num_channels() override;
    uint16_t read(uint8_t ch) override;
    uint8_t read(uint16_t *periods, uint8_t len) override;
    const char *protocol() const override;

    // Drains whatever bytes are currently queued in the UART's RX FIFO into
    // AP_RCProtocol. Must be called periodically -- register_timer_process()
    // is still a no-op on this port (see Scheduler.cpp), so
    // HAL_ChibiOS_K3::run() calls this directly from the main loop, the same
    // way it already hand-pumps UART TX via am67_uart1_tx_pump().
    //
    // Tried running this from a dedicated thread started before
    // callbacks->setup() instead (to also drain during setup(), which can
    // run 15-57s). Reverted: it reliably stalled setup() on hardware with a
    // receiver connected (never with the receiver silent), root cause not
    // understood. Do not reintroduce without figuring out why first.
    void update();

    // Cumulative bytes pulled off the RX queue since boot. Diff it over a
    // reporting window to get the live iBus byte rate: ~4160 B/s is a healthy
    // link, a collapse points at wiring or the receiver, and a healthy rate
    // alongside frozen channel values means decode lost sync (dropped bytes)
    // rather than the link going away.
    uint32_t bytes_seen() const
    {
        return _bytes_seen;
    }

private:
    void *_sd;   // ChibiOS SerialDriver* (SD1)
    uint32_t _bytes_seen = 0;
};
