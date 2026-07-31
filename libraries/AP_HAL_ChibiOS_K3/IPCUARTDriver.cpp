#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include "IPCUARTDriver.h"
#include "hwdef/boot/ipc_ring.h"
#include "hwdef/boot/trace.h"

using namespace ChibiOS_K3;

IPCUARTDriver::IPCUARTDriver() :
    _initialized(false),
    _begin_count(0)
{
}

void IPCUARTDriver::_begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace)
{
    // Baud and buffer sizes are meaningless for a memory transport. They are
    // accepted and ignored rather than rejected: AP_SerialManager passes
    // SERIAL0_BAUD unconditionally, and refusing would take out the whole
    // GCS init path for a parameter that cannot apply here.
    (void)baud;
    (void)rxSpace;
    (void)txSpace;

    // Idempotent -- see ipc_ring_init(). The ring is normally already up by
    // the time this runs (run() initialises it before callbacks->setup()), and
    // resetting the indices here under a live Linux daemon would tear the
    // stream mid-frame.
    ipc_ring_init();
    _initialized = true;

    if (_begin_count < 255) {
        _begin_count++;
    }
    trace_printf("AP-K3: ipc begin #%u (baud %u ignored, memory transport)\n",
                 (uint32_t)_begin_count, baud);
}

size_t IPCUARTDriver::_write(const uint8_t *buffer, size_t size)
{
    // Non-blocking and never partial-blocking: a short return is the AP_HAL
    // contract and GCS_MAVLink checks txspace() first. With 8 KiB per
    // direction a short write means the Linux daemon has stopped draining,
    // which ipc_ring_tx_refused() records for the health line.
    return (size_t)ipc_ring_write(buffer, (uint32_t)size);
}

ssize_t IPCUARTDriver::_read(uint8_t *buffer, uint16_t count)
{
    // Real, unlike ChibiOS_K3::UARTDriver::_read(). This is the whole point of
    // the transport: QGC can talk back, so parameters, commands and
    // calibration work.
    return (ssize_t)ipc_ring_read(buffer, (uint32_t)count);
}

void IPCUARTDriver::_end()
{
    // The ring is not torn down. Linux may still be attached, and there is no
    // hardware to release -- the memory stays valid and the daemon keeps
    // seeing a consistent header. Only the AP-side flag drops.
    _initialized = false;
}

void IPCUARTDriver::_flush()
{
    // Nothing to do: a write is already in DDR by the time _write() returns.
    // There is no FIFO, no interrupt and no pump to force (contrast
    // UARTDriver::_flush(), which must force am67_uart1_tx_pump() because the
    // THRE interrupt never fires -- Q-26).
}

uint32_t IPCUARTDriver::_available()
{
    return ipc_ring_rx_available();
}

bool IPCUARTDriver::_discard_input()
{
    ipc_ring_discard_rx();
    return true;
}

bool IPCUARTDriver::is_initialized()
{
    return _initialized;
}

bool IPCUARTDriver::tx_pending()
{
    return ipc_ring_tx_pending() > 0;
}

uint32_t IPCUARTDriver::txspace()
{
    return ipc_ring_tx_space();
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
