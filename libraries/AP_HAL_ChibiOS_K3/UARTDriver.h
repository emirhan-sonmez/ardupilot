#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_K3_Namespace.h"

/*
  Console UARTDriver for the AM67/K3 board: a thin wrapper over a ChibiOS
  buffered SIO driver (hal_buffered_sio_c). The driver pointer is held as an
  opaque void* so ch.h/hal.h stay out of this header.

  XHAL note: this used to wrap a classic SerialDriver, which XHAL does not
  have. The nearest equivalent is hal_buffered_sio_c -- a SIODriver plus the
  software input/output queues and the asynchronous_channel_i that
  chnWriteTimeout() needs. The caller owns the wrapper and its buffers and
  hands this class an already-constructed one, exactly as it used to hand over
  an already-declared SerialDriver.

  Not currently instantiated: DR-016 moved MAVLink to the shared-memory rings
  and gave UART1 entirely to RCInput, so nothing constructs this today. It is
  kept because it is the serial backend a SiK telemetry radio on a second UART
  would use. Anything changed here is therefore compile-verified only.

  M3 scope: synchronous, non-blocking _write (reliable bring-up console). A
  fully async path can come later.
*/
class ChibiOS_K3::UARTDriver : public AP_HAL::UARTDriver
{
public:
    explicit UARTDriver(void *buffered_sio);

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
    void *_bsio;        // ChibiOS hal_buffered_sio_c*
    bool _initialized;

    // TX diagnostics: bounded, first-few-calls-only (see .cpp). Not a
    // behavior change -- purely for tracing the MAVLink TX path over trace0.
    uint8_t _begin_count;
    uint8_t _write_trace_count;
};
