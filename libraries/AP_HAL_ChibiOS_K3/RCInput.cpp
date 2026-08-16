#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include "RCInput.h"
#include <ch.h>
#include <hal.h>
#include <AP_RCProtocol/AP_RCProtocol.h>
#include "hwdef/boot/trace.h"

using namespace ChibiOS_K3;

/*
  The buffered-SIO wrapper and its queues.

  File scope rather than members so the buffers land in .bss with a known size
  rather than inflating whatever object owns the RCInput, and so the config
  outlives every call: XHAL stores the config pointer inside the driver and
  dereferences it again later, so a stack-local would dangle.

  The TX queue is one byte because this side of the UART has no pad at all --
  the console TX pin was reassigned to EHRPWM0_B under DR-016. A zero-length
  queue is not allowed by the wrapper, so this is the smallest honest size; it
  is never written to.
*/
static hal_buffered_sio_c ibus_bsio;
static uint8_t ibus_rx_queue[ChibiOS_K3::RCInput::RX_QUEUE_SIZE];
static uint8_t ibus_tx_queue[1];

static const SIOConfig ibus_sio_config = {
    .baud = ChibiOS_K3::RCInput::IBUS_BAUD,
    .lcr  = TI_UART_LCR_8N1,
    .fcr  = TI_UART_FCR_FIFOEN | TI_UART_FCR_RXTRIGGER_8,
};

RCInput::RCInput(void *sio_driver) :
    _siop(sio_driver)
{
}

void RCInput::init()
{
    /*
      RCInput owns UART1 outright as of DR-016, and that includes starting it.

      This used to be done for us: serial0 was this UART, so AP_SerialManager's
      serial0->begin(SERIAL0_BAUD) opened it during callbacks->setup().
      MAVLink has since moved to the shared-memory rings (IPCUARTDriver), the
      UART is no longer an AP_HAL serial port, and nothing else in the boot
      path opens it. Without this call the receiver line is dead and the only
      symptom is `ibus: NO BYTES AT ALL` -- which looks exactly like a wiring
      fault, so it would cost a bench session to find.

      115200 8N1 is iBus, not a configurable choice, so it is pinned here
      rather than taken from a parameter. It is also what process_byte() below
      is told the line rate is; the two must agree.

      RXTRIGGER_8 rather than a deeper FIFO trigger: iBus frames are 32 bytes
      and arrive every ~7.7 ms, so a shallow trigger keeps the interrupt
      cadence tied to the data rather than to a timeout.
    */
    (void)bsioObjectInit(&ibus_bsio, (hal_sio_driver_c *)_siop,
                         ibus_rx_queue, sizeof(ibus_rx_queue),
                         ibus_tx_queue, sizeof(ibus_tx_queue));

    const msg_t msg = drvStart(&ibus_bsio, &ibus_sio_config);

    // AP_RCProtocol is initialised either way: the rest of the vehicle expects
    // it to exist, and a failure here means no bytes rather than no decoder.
    AP::RC().init();

    if (msg != HAL_RET_SUCCESS) {
        // Not fatal to the boot -- the vehicle still runs, it just has no RC
        // input. Saying so beats presenting a dead line as a wiring fault.
        trace_printf("AP-K3: RCInput UART1 drvStart FAILED msg=%d -- no RC input\n",
                     (int)msg);
        return;
    }

    trace_printf("AP-K3: RCInput init, iBus on UART1 RX (pin 10) @%u, UART started here\n",
                 (uint32_t)IBUS_BAUD);
}

uint32_t RCInput::tx_drain()
{
    size_t queued;

    while (!sioIsTXFullX(ibus_bsio.siop)) {
        msg_t b;

        chSysLock();
        b = oqGetI(&ibus_bsio.oqueue);
        chSysUnlock();

        if (b < MSG_OK) {
            break;                 // queue empty
        }
        sioPutX(ibus_bsio.siop, (uint8_t)b);
    }

    chSysLock();
    queued = oqGetFullI(&ibus_bsio.oqueue);
    chSysUnlock();

    return (uint32_t)queued;
}

uint32_t RCInput::rx_queued()
{
    size_t queued;

    chSysLock();
    queued = iqGetFullI(&ibus_bsio.iqueue);
    chSysUnlock();

    return (uint32_t)queued;
}

void RCInput::update()
{
    uint8_t b[64];
    size_t n = 0;

    /*
      Drain until the queue is actually empty, not just one bufferful.

      RX_QUEUE_SIZE is 512 (RCInput.h, raised from 64 as part of the Q-36
      mitigation), so the RX queue holds ~123 ms of iBus (32-byte frames at
      130 Hz, ~4160 B/s). Any main-loop iteration longer than that still
      overflows the queue and ChibiOS still drops the excess on the
      floor. Dropped bytes corrupt iBus framing, and AP_RCProtocol's channel
      count stays latched at its last good value while read() returns stale
      data -- sticks appear frozen with chans=14, and bench_passthrough's
      frame-timeout failsafe cannot see it because num_channels() never falls
      below its threshold.

      Suspected cause of control loss minutes into a run: the IMU resync path
      (bench_imu.cpp, DR-013) does a full re-bring-up of 32 polled SPI
      transactions at 250 kHz inside one iteration, measured at 210-221 ms.
      That still exceeds 123 ms. The queue size buys margin; it is not the
      fix, and the fix is making resync not block the main loop.

      Bounded rather than unbounded: RX_DRAIN_MAX_BYTES caps the work per tick
      so a receiver spraying faster than we can decode cannot starve the rest
      of the loop. Sized to drain a full queue in one tick -- less than that
      and a burst arriving after a stall could never be caught up on.
    */
    constexpr size_t RX_DRAIN_MAX_BYTES = RX_QUEUE_SIZE;
    while (n < RX_DRAIN_MAX_BYTES) {
        const size_t got = chnReadTimeout(&ibus_bsio.chn, b, sizeof(b),
                                          TIME_IMMEDIATE);
        if (got == 0) {
            break;
        }
        for (size_t i = 0; i < got; i++) {
            AP::RC().process_byte(b[i], IBUS_BAUD);
        }
        n += got;
        if (got < sizeof(b)) {
            break;                 // queue drained
        }
    }

    /*
      Dead-line warning only. This used to also print a live rate/byte
      report every 5s -- pulled once decode was confirmed working (chans=14,
      real checksummed frames) and the arm-channel index was found, since
      the buffer cost was getting in the way of diagnosing other things
      (16 KiB trace buffer, no wrap). If it's ever needed again: rate=0
      means dead line (wrong pin/port, no ground, unpowered receiver);
      rate>0 with zero valid channels means wrong baud/format or wired to a
      PWM servo output instead of the Servo port's UART pin.
    */
    static bool warned;

    _bytes_seen += n;

    const uint32_t now_ms = AP_HAL::millis();
    if (!warned && now_ms > 10000 && _bytes_seen == 0) {
        warned = true;
        trace_printf("ibus: NO BYTES AT ALL. rate=0 -- check: receiver "
                     "powered? bound to TX? pin 10 actually connected to "
                     "the receiver's Servo (not Sens) port? ground shared "
                     "with the board?\n");
    }
}

bool RCInput::new_input()
{
    return AP::RC().new_input();
}

uint8_t RCInput::num_channels()
{
    return AP::RC().num_channels();
}

uint16_t RCInput::read(uint8_t ch)
{
    return AP::RC().read(ch);
}

uint8_t RCInput::read(uint16_t *periods, uint8_t len)
{
    const uint8_t n = MIN(len, num_channels());
    for (uint8_t i = 0; i < n; i++) {
        periods[i] = AP::RC().read(i);
    }
    return n;
}

const char *RCInput::protocol() const
{
    return AP::RC().detected_protocol_name();
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
