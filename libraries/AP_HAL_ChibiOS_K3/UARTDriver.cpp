#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include "UARTDriver.h"
#include <ch.h>
#include <hal.h>

using namespace ChibiOS_K3;

UARTDriver::UARTDriver(void *serial_driver) :
    _sd(serial_driver),
    _initialized(false)
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
}

size_t UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    SerialDriver *sd = (SerialDriver *)_sd;
    // Blocking write: queue every byte (the TX path drains as the UART sends),
    // so a console line is emitted in full. M3 bring-up choice.
    return chnWriteTimeout(sd, buffer, size, TIME_INFINITE);
}

ssize_t UARTDriver::_read(uint8_t *buffer, uint16_t count)
{
    SerialDriver *sd = (SerialDriver *)_sd;
    return (ssize_t)chnReadTimeout(sd, buffer, count, TIME_IMMEDIATE);
}

void UARTDriver::_end()
{
    sdStop((SerialDriver *)_sd);
    _initialized = false;
}

void UARTDriver::_flush()
{
    // Output drains via the TX interrupt; nothing to force here.
}

uint32_t UARTDriver::_available()
{
    SerialDriver *sd = (SerialDriver *)_sd;
    chSysLock();
    size_t n = iqGetFullI(&sd->iqueue);
    chSysUnlock();
    return (uint32_t)n;
}

bool UARTDriver::_discard_input()
{
    SerialDriver *sd = (SerialDriver *)_sd;
    uint8_t b;
    while (chnReadTimeout(sd, &b, 1, TIME_IMMEDIATE) == 1) {
    }
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
    chSysLock();
    size_t n = oqGetEmptyI(&sd->oqueue);
    chSysUnlock();
    return (uint32_t)n;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
