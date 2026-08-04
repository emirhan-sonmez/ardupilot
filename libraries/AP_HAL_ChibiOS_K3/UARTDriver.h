#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_K3_Namespace.h"

/*
  Console UARTDriver for the AM67/K3 board: a thin wrapper over a ChibiOS
  SerialDriver (SD1 for the 40-pin header console). The SerialDriver pointer is
  held as an opaque void* so ch.h/hal.h stay out of this header.

  M3 scope: synchronous, blocking _write (reliable bring-up console). A buffered/
  async path can come later.
*/
class ChibiOS_K3::UARTDriver : public AP_HAL::UARTDriver
{
public:
    explicit UARTDriver(void *serial_driver);

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
    void *_sd;          // ChibiOS SerialDriver*
    bool _initialized;

    // TX diagnostics: bounded, first-few-calls-only (see .cpp). Not a
    // behavior change -- purely for tracing the MAVLink TX path over trace0.
    uint8_t _begin_count;
    uint8_t _write_trace_count;
};
