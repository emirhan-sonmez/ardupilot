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
    AP::RC().init();
    trace_printf("AP-K3: RCInput init, iBus on UART1 RX (pin 10) @115200\n");
}

void RCInput::update()
{
    SerialDriver *sd = (SerialDriver *)_sd;
    uint8_t b[64];
    const size_t n = chnReadTimeout(sd, b, sizeof(b), TIME_IMMEDIATE);
    for (size_t i = 0; i < n; i++) {
        AP::RC().process_byte(b[i], 115200);
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
    static uint32_t bytes_seen;
    static bool warned;

    bytes_seen += n;

    const uint32_t now_ms = AP_HAL::millis();
    if (!warned && now_ms > 10000 && bytes_seen == 0) {
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
