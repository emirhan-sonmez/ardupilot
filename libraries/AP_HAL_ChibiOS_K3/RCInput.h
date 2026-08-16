#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_K3_Namespace.h"

/*
  iBus RC input over the shared UART1 (SIOD1) RX side, 40-pin header pin 10.
  Reads raw bytes directly off the ChibiOS RX queue and feeds them to
  AP_RCProtocol via AP::RC().process_byte() -- framing, checksum and channel
  extraction are entirely AP_RCProtocol_IBUS's job, not reimplemented here
  (see the ArduPilot iBus Port Handoff note, section 4a).

  This class owns UART1 outright as of DR-016. MAVLink used to share the same
  physical UART (TX on pin 8) and has since moved to the shared-memory rings
  to Linux (IPCUARTDriver), so it is no longer an AP_HAL serial port at all.
  Consequence worth knowing: nothing else in the boot path opens it any more,
  so init() below does -- previously AP_SerialManager's serial0->begin() did
  it as a side effect.

  XHAL note: the classic SerialDriver this used to sit on does not exist in
  XHAL. Its replacement is a plain SIODriver, which is FIFO-level only, plus
  hal_buffered_sio_c -- the wrapper that supplies the software RX queue and
  the asynchronous_channel_i that chnReadTimeout() needs. The wrapper is what
  this class owns; the bare SIODriver underneath it is what the constructor is
  handed.
*/
class ChibiOS_K3::RCInput : public AP_HAL::RCInput
{
public:
    // iBus line rate. Fixed by the protocol, not a parameter. Used both to
    // open the UART and to tell AP_RCProtocol what the line rate is; the two
    // must not be allowed to drift apart.
    static constexpr uint32_t IBUS_BAUD = 115200;

    /*
      Software RX queue depth, carried over from the classic build's
      SERIAL_BUFFERS_SIZE (raised from 64 as part of the Q-36 mitigation). It
      holds ~123 ms of iBus -- 32-byte frames at 130 Hz, ~4160 B/s. The UART's
      own hardware FIFO is 64 bytes, about 15 ms, which is why the buffered SIO
      wrapper is used rather than the bare SIODriver: any main-loop iteration
      longer than the queue depth drops bytes, and dropped bytes corrupt iBus
      framing in a way that presents as frozen sticks rather than as signal
      loss. See update() for the rest of that story.
    */
    static constexpr size_t RX_QUEUE_SIZE = 512;

    explicit RCInput(void *sio_driver);

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
    // way UARTDriver hand-pumps its TX queue into the FIFO.
    //
    // Tried running this from a dedicated thread started before
    // callbacks->setup() instead (to also drain during setup(), which can
    // run 15-57s). Reverted: it reliably stalled setup() on hardware with a
    // receiver connected (never with the receiver silent), root cause not
    // understood. Do not reintroduce without figuring out why first.
    void update();

    // Drains anything sitting in this UART's TX queue into the hardware FIFO
    // and returns how many bytes were still queued afterwards.
    //
    // A safety valve, not a transmit path: this UART has no TX pad at all
    // (DR-016 gave pin 8 to EHRPWM0_B) and the THRE interrupt does not fire on
    // it (Q-26), so a stray writer would otherwise fill the queue and block
    // itself forever. A non-zero return means something is writing to a port
    // that physically cannot transmit. Lives here because RCInput owns the
    // buffered-SIO wrapper; it replaces the classic HAL's am67_uart1_tx_pump().
    uint32_t tx_drain();

    // Bytes currently waiting in the software RX queue.
    //
    // This is the number that matters for Q-36: the queue backing up means the
    // main loop is not draining it fast enough, and the next step after full
    // is dropped bytes and a desynchronised iBus decoder. It replaces the
    // classic driver's am67_uart1_* ISR counters in the alive line -- those
    // instrumented the THRE problem (Q-26) from inside the AM67 serial LLD,
    // and the XHAL UARTv1 driver exports no such counters. Adding them there
    // would mean carrying AP-specific instrumentation in a driver headed
    // upstream.
    uint32_t rx_queued();

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
    void *_siop;   // ChibiOS SIODriver* (SIOD1), the bare peripheral
    uint32_t _bytes_seen = 0;
};
