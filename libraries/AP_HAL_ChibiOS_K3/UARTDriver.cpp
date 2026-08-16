#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include "UARTDriver.h"
#include <ch.h>
#include <hal.h>
#include "hwdef/boot/trace.h"

using namespace ChibiOS_K3;

/*
  Moves queued bytes into the UART's hardware TX FIFO.

  This replaces am67_uart1_tx_pump(), a helper the classic AM67 serial driver
  exported for exactly this purpose; XHAL's SIO driver has no equivalent and
  should not grow one, since the need is specific to this board. The reason it
  is needed at all has not changed: the THRE interrupt does not fire on this
  UART (am67_uart1_thre_count stayed at 0 across every hardware run), so
  nothing drains the software queue on its own. Without pumping, the first
  burst that outgrows the FIFO fills the queue and it never empties again.

  Written against the public queue and SIO APIs rather than by reaching into
  the peripheral: oqGetI() pops one byte under the system lock, sioPutX()
  pushes it, and sioIsTXFullX() bounds the loop at the FIFO's real capacity.

  REMOVE-AFTER: a hardware run shows the XHAL UARTv1 driver's TX interrupt
  firing on this board, at which point the buffered driver drains itself and
  this becomes dead weight.
*/
static size_t tx_pump(void *ip)
{
    hal_buffered_sio_c *bsio = (hal_buffered_sio_c *)ip;
    size_t moved = 0;

    while (!sioIsTXFullX(bsio->siop)) {
        msg_t b;

        chSysLock();
        b = oqGetI(&bsio->oqueue);
        chSysUnlock();

        if (b < MSG_OK) {
            break;                 // queue empty
        }
        sioPutX(bsio->siop, (uint8_t)b);
        moved++;
    }
    return moved;
}

UARTDriver::UARTDriver(void *buffered_sio) :
    _bsio(buffered_sio),
    _initialized(false),
    _begin_count(0),
    _write_trace_count(0)
{
}

void UARTDriver::_begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace)
{
    (void)rxSpace;
    (void)txSpace;

    /*
      Static so the driver's stored config pointer stays valid: XHAL keeps the
      pointer it is given and dereferences it again later, so a local would
      dangle the moment this returns.
    */
    static SIOConfig cfg;
    cfg.baud = baud;
    cfg.lcr  = TI_UART_LCR_8N1;
    cfg.fcr  = TI_UART_FCR_FIFOEN | TI_UART_FCR_RXTRIGGER_8;

    const msg_t msg = drvStart(_bsio, &cfg);
    _initialized = (msg == HAL_RET_SUCCESS);
    if (_begin_count < 255) {
        _begin_count++;
    }
    // Every begin() call re-runs the AM67 UART init sequence (FIFO reset,
    // new baud divisor, IER rewritten) -- traced every time (not just the
    // first few) since AP_SerialManager/GCS_MAVLINK call this UART's begin()
    // up to 4 times during boot (console init, per-port init, and GCS_MAVLINK
    // ::init()'s SiK-wake sequence begins it twice more) and repeat calls
    // are cheap/rare enough not to be "high-frequency loop" spam.
    trace_printf("AP-K3: UART begin #%u baud=%u msg=%d\n",
                 (uint32_t)_begin_count, baud, (int)msg);
}

size_t UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    hal_buffered_sio_c *bsio = (hal_buffered_sio_c *)_bsio;

    const bool do_trace = (_write_trace_count < 8);
    uint32_t space_before = 0;
    if (do_trace) {
        chSysLock();
        space_before = (uint32_t)oqGetEmptyI(&bsio->oqueue);
        chSysUnlock();
    }

    // Non-blocking write, per the AP_HAL contract: callers (GCS_MAVLink in
    // particular) check txspace() first and cope with a short write. The
    // previous TIME_INFINITE blocking write deadlocked the entire vehicle
    // main loop the first time a burst outgrew the TX FIFO, because the
    // THRE interrupt does not currently fire on this UART so the software
    // queue never drained. Pump first so any bytes stranded from a previous
    // call get moved out before we try to queue more.
    (void)tx_pump(bsio);
    const size_t accepted = chnWriteTimeout(&bsio->chn, buffer, size,
                                            TIME_IMMEDIATE);

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
    (void)_bsio;
    (void)buffer;
    (void)count;
    return 0;
}

void UARTDriver::_end()
{
    drvStop(_bsio);
    _initialized = false;
}

void UARTDriver::_flush()
{
    // THRE interrupt is not firing on this UART, so push queued bytes out
    // explicitly rather than assuming the ISR will do it.
    (void)tx_pump(_bsio);
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
    (void)_bsio;
    return true;
}

bool UARTDriver::is_initialized()
{
    return _initialized;
}

bool UARTDriver::tx_pending()
{
    hal_buffered_sio_c *bsio = (hal_buffered_sio_c *)_bsio;
    chSysLock();
    bool pending = oqGetFullI(&bsio->oqueue) > 0;
    chSysUnlock();
    return pending;
}

uint32_t UARTDriver::txspace()
{
    hal_buffered_sio_c *bsio = (hal_buffered_sio_c *)_bsio;
    // Pump before reporting: without a working THRE interrupt a full queue
    // would otherwise report 0 space forever, the caller would stop writing,
    // and nothing would ever trigger a drain again.
    (void)tx_pump(bsio);
    chSysLock();
    size_t n = oqGetEmptyI(&bsio->oqueue);
    chSysUnlock();
    return (uint32_t)n;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
