#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_K3_Namespace.h"

/*
  iBus RC input over the shared UART1 (SD1) RX side, 40-pin header pin 10.
  Reads raw bytes directly off the ChibiOS SerialDriver's RX queue and feeds
  them to AP_RCProtocol via AP::RC().process_byte() -- framing, checksum and
  channel extraction are entirely AP_RCProtocol_IBUS's job, not reimplemented
  here (see the ArduPilot iBus Port Handoff note, section 4a).

  serial0/ChibiOS_K3::UARTDriver keeps MAVLink TX on the same physical wire's
  TX side (pin 8); its RX side is disabled (see UARTDriver.cpp) so this class
  is the sole consumer of SD1's incoming bytes -- two readers pulling off the
  same ChibiOS input queue would each steal bytes meant for the other and
  break both protocols' framing.
*/
class ChibiOS_K3::RCInput : public AP_HAL::RCInput {
public:
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

private:
    void *_sd;   // ChibiOS SerialDriver* (SD1)
};
