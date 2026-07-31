#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include "RCInput.h"
#include <ch.h>
#include <hal.h>
#include <AP_RCProtocol/AP_RCProtocol.h>
#include "hwdef/boot/trace.h"

using namespace ChibiOS_K3;

RCInput::RCInput(void *serial_driver) :
    _sd(serial_driver)
{
}

void RCInput::init()
{
    /*
      RCInput owns SD1 outright as of DR-016, and that includes starting it.

      This used to be done for us: serial0 was SD1, so AP_SerialManager's
      serial0->begin(SERIAL0_BAUD) ran sdStart() during callbacks->setup().
      MAVLink has since moved to the shared-memory rings (IPCUARTDriver), SD1
      is no longer an AP_HAL serial port, and nothing else in the boot path
      opens it. Without this call the receiver line is dead and the only
      symptom is `ibus: NO BYTES AT ALL` -- which looks exactly like a wiring
      fault, so it would cost a bench session to find.

      115200 8N1 is iBus, not a configurable choice, so it is pinned here
      rather than taken from a parameter. It is also what process_byte() below
      is told the line rate is; the two must agree.
    */
    SerialDriver *sd = (SerialDriver *)_sd;
    SerialConfig cfg = { IBUS_BAUD };
    sdStart(sd, &cfg);

    AP::RC().init();
    trace_printf("AP-K3: RCInput init, iBus on UART1 RX (pin 10) @%u, SD1 started here\n",
                 (uint32_t)IBUS_BAUD);
}

void RCInput::update()
{
    SerialDriver *sd = (SerialDriver *)_sd;
    uint8_t b[64];
    size_t n = 0;

    /*
      Drain until the queue is actually empty, not just one bufferful.

      SERIAL_BUFFERS_SIZE is 512 (hwdef/cfg/halconf.h, raised from 64 as part
      of the Q-36 mitigation), so the RX queue holds ~123 ms of iBus (32-byte
      frames at 130 Hz, ~4160 B/s). Any main-loop iteration longer than that
      still overflows the queue and ChibiOS still drops the excess on the
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
    constexpr size_t RX_DRAIN_MAX_BYTES = SERIAL_BUFFERS_SIZE;
    while (n < RX_DRAIN_MAX_BYTES) {
        const size_t got = chnReadTimeout(sd, b, sizeof(b), TIME_IMMEDIATE);
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
