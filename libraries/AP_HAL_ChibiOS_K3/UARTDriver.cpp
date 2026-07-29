#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include "UARTDriver.h"
#include <ch.h>
#include <hal.h>
#include "hwdef/boot/trace.h"

using namespace ChibiOS_K3;

UARTDriver::UARTDriver(void *serial_driver) :
    _sd(serial_driver),
    _initialized(false),
    _begin_count(0),
    _write_trace_count(0)
{
}

void UARTDriver::_begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace)
{
    (void)rxSpace;
    (void)txSpace;
    SerialDriver *sd = (SerialDriver *)_sd;
    SerialConfig cfg = { baud };
    sdStart(sd, &cfg);
    _initialized = true;
    if (_begin_count < 255) {
        _begin_count++;
    }
    // Every begin() call re-runs the AM67 uart_init() sequence (FIFO reset,
    // new baud divisor, IER rewritten) -- traced every time (not just the
    // first few) since AP_SerialManager/GCS_MAVLINK call this UART's begin()
    // up to 4 times during boot (console init, per-port init, and GCS_MAVLINK
    // ::init()'s SiK-wake sequence begins it twice more) and repeat calls
    // are cheap/rare enough not to be "high-frequency loop" spam.
    trace_printf("AP-K3: UART begin #%u baud=%u\n", (uint32_t)_begin_count, baud);
}

size_t UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    SerialDriver *sd = (SerialDriver *)_sd;

    const bool do_trace = (_write_trace_count < 8);
    uint32_t space_before = 0;
    if (do_trace) {
        chSysLock();
        space_before = (uint32_t)oqGetEmptyI(&sd->oqueue);
        chSysUnlock();
    }

    // Non-blocking write, per the AP_HAL contract: callers (GCS_MAVLink in
    // particular) check txspace() first and cope with a short write. The
    // previous TIME_INFINITE blocking write deadlocked the entire vehicle
    // main loop the first time a burst outgrew the TX FIFO, because the
    // THRE interrupt does not currently fire on this UART so the software
    // queue never drained (am67_uart1_thre_count == 0). Pump first so any
    // bytes stranded from a previous call get moved out before we try to
    // queue more.
    (void)am67_uart1_tx_pump();
    const size_t accepted = chnWriteTimeout(sd, buffer, size, TIME_IMMEDIATE);

    if (do_trace) {
        _write_trace_count++;
        trace_printf("AP-K3: UART write #%u req=%u txspace_before=%u accepted=%u\n",
                     (uint32_t)_write_trace_count, (uint32_t)size,
                     space_before, (uint32_t)accepted);
    }

    return accepted;
}

ssize_t UARTDriver::_read(uint8_t *buffer, uint16_t count)
{
    // RX on this UART belongs exclusively to ChibiOS_K3::RCInput (iBus,
    // pin 10) -- see the driver-instance comment in HAL_ChibiOS_K3_Class.cpp.
    // Draining bytes here too would race RCInput for the same ChibiOS input
    // queue and corrupt both the iBus framing and whatever this caller
    // thought it was reading. No GCS is attached this milestone, so this
    // serial port only ever needs to be TX (MAVLink out); returning "no
    // bytes" is correct, not a stub.
    (void)_sd;
    (void)buffer;
    (void)count;
    return 0;
}

void UARTDriver::_end()
{
    sdStop((SerialDriver *)_sd);
    _initialized = false;
}

void UARTDriver::_flush()
{
    // THRE interrupt is not firing on this UART, so push queued bytes out
    // explicitly rather than assuming the ISR will do it.
    (void)am67_uart1_tx_pump();
}

uint32_t UARTDriver::_available()
{
    // See _read() -- RX belongs to RCInput on this UART.
    return 0;
}

bool UARTDriver::_discard_input()
{
    // See _read() -- RX belongs to RCInput on this UART; nothing to discard
    // from this driver's perspective (would otherwise steal iBus bytes).
    (void)_sd;
    return true;
}

bool UARTDriver::is_initialized()
{
    return _initialized;
}

bool UARTDriver::tx_pending()
{
    SerialDriver *sd = (SerialDriver *)_sd;
    chSysLock();
    bool pending = oqGetFullI(&sd->oqueue) > 0;
    chSysUnlock();
    return pending;
}

uint32_t UARTDriver::txspace()
{
    SerialDriver *sd = (SerialDriver *)_sd;
    // Pump before reporting: without a working THRE interrupt a full queue
    // would otherwise report 0 space forever, the caller would stop writing,
    // and nothing would ever trigger a drain again.
    (void)am67_uart1_tx_pump();
    chSysLock();
    size_t n = oqGetEmptyI(&sd->oqueue);
    chSysUnlock();
    return (uint32_t)n;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
