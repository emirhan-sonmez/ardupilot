#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include "SPIDevice.h"
#include <ch.h>
#include <hal.h>                // SPID1 = MCU_MCSPI0, SPIConfig
#include <string.h>
#include "hwdef/boot/trace.h"

using namespace ChibiOS_K3;

extern const AP_HAL::HAL& hal;

/*
  Device table. The name is what a driver passes to hal.spi->get_device().

  Speeds are both 250 kHz today because of DR-013: 1 MHz corrupts multi-byte
  transactions on this board's wiring (single-byte reads pass, which is what
  made it look clean for two sessions), and the electrical root cause was
  never measured on a scope. That is survivable for bring-up and NOT survivable
  for flight -- a 14-byte burst costs ~450us at 250 kHz, so a 1 kHz gyro
  callback would spend ~45% of the core inside a polled transfer. Raising
  speed_high_hz is the single highest-value change available to this port once
  DR-013 is reopened; keep speed_low_hz conservative regardless, it is only
  used for register configuration at start-up.
*/
static const SPIDeviceDesc device_table[] = {
    // name        cs  mode  low       high
    { "icm20948",   3,   3,  250000,   250000 },   // onboard IMU
    { "bmp390",     1,   3,  250000,   250000 },   // onboard barometer
};

static const uint8_t NUM_DEVICES = ARRAY_SIZE(device_table);

// One physical controller, so one bus object shared by every device.
static SPIBus spi_bus;

/*
  Bus-thread tick. CH_CFG_ST_FREQUENCY is 1000 with CH_CFG_ST_TIMEDELTA 0
  (periodic systick), so 1ms is the finest sleep this kernel can express and
  therefore the ceiling on callback rate as well as the jitter floor. A 1 kHz
  gyro callback lands at 1 kHz +/-1ms of phase noise, which the INS backends
  tolerate (they timestamp from their own FIFO), but it is the reason this
  cannot go faster without moving to tickless mode.
*/
static const uint32_t BUS_TICK_US = 1000;

/*
  Priority offset for the bus thread, passed to Scheduler::thread_create()
  which computes NORMALPRIO + priority and ignores the base (see Scheduler.cpp).

  Deliberately BELOW the main thread, which init() boosts to NORMALPRIO+10.
  Stock AP_HAL_ChibiOS runs device bus threads above the main loop, and that is
  the right answer once transfers are fast -- but at 250 kHz a 1 kHz callback
  is ~45% CPU of busy-polled SPI, and above main that starves the flight loop
  outright. This ordering makes a slow bus degrade sampling rate instead of
  degrading control. Raise it together with the bus speed, not before, and
  re-read the log_io starvation note in Scheduler.cpp first.
*/
static const int8_t BUS_THREAD_PRIORITY = 5;
static const uint32_t BUS_THREAD_STACK = 2048;

/*===========================================================================*/
/* SPIBus                                                                    */
/*===========================================================================*/

bool SPIBus::apply_config(const SPIDeviceDesc &desc, uint32_t speed_hz)
{
    if (_spi_started &&
        (_cur_cs == desc.cs_channel) &&
        (_cur_mode == desc.mode) &&
        (_cur_speed == speed_hz)) {
        return true;
    }

    SPIConfig cfg = {
        .end_cb     = nullptr,
        .speed      = speed_hz,
        .mode       = desc.mode,
        .cs_channel = desc.cs_channel,
    };
    spiStart(&SPID1, &cfg);
    if (!SPID1.ready) {
        trace_printf("spi: controller not ready (clock gated? Linux still bound "
                     "to 4b00000.spi?)\n");
        return false;
    }

    /*
      Board-level enable for the onboard IMU (MCU_GPIO0_12, active low).
      Deliberately not part of spi_lld_start(): it touches MCU_GPIO0, a
      peripheral the SPI driver does not own. Idempotent, so calling it on
      every genuine reconfiguration costs one GPIO write.
    */
    am67_spi0_imu_enable();

    _cur_cs = desc.cs_channel;
    _cur_mode = desc.mode;
    _cur_speed = speed_hz;
    _spi_started = true;
    return true;
}

/*
  All transfers are polled (spiPolledExchange), never the driver's
  interrupt-driven spiExchange().

  spiExchange() sleeps the calling thread until the transfer-complete interrupt
  arrives and has no timeout: if that interrupt never comes -- gated module
  clock, a channel that never asserts RX_FULL -- the caller is gone for good and
  the board looks frozen with no diagnostic at all. That is exactly how the
  first attempt at bench_imu.cpp failed. spi_lld_polled_exchange() busy-waits on
  CHSTAT with a bounded loop and reports SPID1.xfer_timeout, so a dead bus costs
  milliseconds and says so.

  Byte-at-a-time also means no combined tx/rx staging buffer is needed for the
  half-duplex case, unlike AP_HAL_ChibiOS which builds one on the stack sized to
  send_len+recv_len. FIFO reads from an INS backend can be hundreds of bytes and
  this port's threads have small stacks.
*/
static bool bus_xfer_bytes(const uint8_t *send, uint32_t send_len,
                           uint8_t *recv, uint32_t recv_len)
{
    bool ok = true;

    spiSelect(&SPID1);
    for (uint32_t i = 0; i < send_len; i++) {
        (void)spiPolledExchange(&SPID1, send[i]);
        if (SPID1.xfer_timeout) {
            ok = false;
            break;
        }
    }
    if (ok) {
        for (uint32_t i = 0; i < recv_len; i++) {
            recv[i] = (uint8_t)spiPolledExchange(&SPID1, 0);
            if (SPID1.xfer_timeout) {
                ok = false;
                break;
            }
        }
    }
    spiUnselect(&SPID1);
    return ok;
}

bool SPIBus::transfer(const uint8_t *send, uint32_t send_len,
                      uint8_t *recv, uint32_t recv_len)
{
    // AP_HAL convention: same buffer and equal lengths means the caller wants a
    // simultaneous exchange, not address-then-data.
    if ((send != nullptr) && (recv != nullptr) &&
        (send == recv) && (send_len == recv_len) && (send_len != 0)) {
        return transfer_fullduplex(send, recv, send_len);
    }
    return bus_xfer_bytes(send, (send == nullptr) ? 0 : send_len,
                          recv, (recv == nullptr) ? 0 : recv_len);
}

bool SPIBus::transfer_fullduplex(const uint8_t *send, uint8_t *recv,
                                 uint32_t len)
{
    bool ok = true;

    spiSelect(&SPID1);
    for (uint32_t i = 0; i < len; i++) {
        const uint8_t out = (send != nullptr) ? send[i] : 0;
        const uint8_t in = (uint8_t)spiPolledExchange(&SPID1, out);
        if (recv != nullptr) {
            recv[i] = in;
        }
        if (SPID1.xfer_timeout) {
            ok = false;
            break;
        }
    }
    spiUnselect(&SPID1);
    return ok;
}

bool SPIBus::start_thread()
{
    if (_thread_started) {
        return true;
    }
    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&SPIBus::thread_loop, void),
            "spi_bus", BUS_THREAD_STACK,
            AP_HAL::Scheduler::PRIORITY_SPI, BUS_THREAD_PRIORITY)) {
        trace_printf("spi: bus thread create FAILED, periodic callbacks dead\n");
        return false;
    }
    _thread_started = true;
    trace_printf("spi: bus thread started (tick=%uus prio=+%u)\n",
                 BUS_TICK_US, (uint32_t)BUS_THREAD_PRIORITY);
    return true;
}

void SPIBus::thread_loop()
{
    while (true) {
        const uint32_t now_us = AP_HAL::micros();

        for (uint8_t i = 0; i < _num_cb; i++) {
            Callback &c = _cb[i];
            if (!c.active) {
                continue;
            }
            // Signed compare so this stays correct across micros() wrapping.
            if ((int32_t)(now_us - c.next_usec) < 0) {
                continue;
            }
            /*
              Schedule from now rather than next_usec += period: if a callback
              overruns its period (very possible at 250 kHz, see the device
              table), accumulating the deficit would make it run back-to-back
              forever and monopolise the bus lock.
            */
            c.next_usec = now_us + c.period_usec;
            {
                WITH_SEMAPHORE(semaphore);
                c.cb();
            }
        }

        hal.scheduler->delay_microseconds(BUS_TICK_US);
    }
}

AP_HAL::Device::PeriodicHandle SPIBus::register_periodic_callback(
    uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    if (_num_cb >= MAX_CALLBACKS) {
        trace_printf("spi: periodic callback table full (%u), request ignored\n",
                     (uint32_t)MAX_CALLBACKS);
        return nullptr;
    }
    if (!start_thread()) {
        return nullptr;
    }

    Callback &c = _cb[_num_cb];
    c.cb = cb;
    c.period_usec = period_usec;
    c.next_usec = AP_HAL::micros() + period_usec;
    c.active = true;
    _num_cb++;

    if (period_usec < BUS_TICK_US) {
        trace_printf("spi: callback asked for %uus, bus tick is %uus -- rate "
                     "capped by CH_CFG_ST_FREQUENCY\n",
                     period_usec, BUS_TICK_US);
    }
    return (AP_HAL::Device::PeriodicHandle)&c;
}

bool SPIBus::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h,
                                      uint32_t period_usec)
{
    for (uint8_t i = 0; i < _num_cb; i++) {
        if ((AP_HAL::Device::PeriodicHandle)&_cb[i] == h) {
            _cb[i].period_usec = period_usec;
            return true;
        }
    }
    return false;
}

/*===========================================================================*/
/* SPIDevice                                                                 */
/*===========================================================================*/

SPIDevice::SPIDevice(SPIBus &bus, const SPIDeviceDesc &desc) :
    _bus(bus),
    _desc(desc),
    _speed_hz(desc.speed_low_hz)
{
    set_device_bus(0);
    set_device_address(desc.cs_channel);
}

bool SPIDevice::set_speed(AP_HAL::Device::Speed speed)
{
    _speed_hz = (speed == AP_HAL::Device::SPEED_HIGH) ? _desc.speed_high_hz
                                                      : _desc.speed_low_hz;
    return true;
}

bool SPIDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    if (!_bus.apply_config(_desc, _speed_hz)) {
        return false;
    }
    return _bus.transfer(send, send_len, recv, recv_len);
}

bool SPIDevice::transfer_fullduplex(const uint8_t *send, uint8_t *recv,
                                    uint32_t len)
{
    if (!_bus.apply_config(_desc, _speed_hz)) {
        return false;
    }
    return _bus.transfer_fullduplex(send, recv, len);
}

AP_HAL::Semaphore *SPIDevice::get_semaphore()
{
    return &_bus.semaphore;
}

AP_HAL::Device::PeriodicHandle SPIDevice::register_periodic_callback(
    uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return _bus.register_periodic_callback(period_usec, cb);
}

bool SPIDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h,
                                         uint32_t period_usec)
{
    return _bus.adjust_periodic_callback(h, period_usec);
}

/*===========================================================================*/
/* SPIDeviceManager                                                          */
/*===========================================================================*/

AP_HAL::SPIDevice *SPIDeviceManager::get_device_ptr(const char *name)
{
    if (name == nullptr) {
        return nullptr;
    }
    for (uint8_t i = 0; i < NUM_DEVICES; i++) {
        if (strcmp(name, device_table[i].name) == 0) {
            /*
              Heap-allocated and never freed: AP sensor drivers hold their
              device for the life of the vehicle, and this port has no need to
              support probe-then-discard. NEW_NOTHROW so a full heap reports as
              a failed probe rather than aborting the boot.
            */
            return NEW_NOTHROW SPIDevice(spi_bus, device_table[i]);
        }
    }
    trace_printf("spi: no device named '%s' in the table\n", name);
    return nullptr;
}

uint8_t SPIDeviceManager::get_count()
{
    return NUM_DEVICES;
}

const char *SPIDeviceManager::get_device_name(uint8_t idx)
{
    return (idx < NUM_DEVICES) ? device_table[idx].name : nullptr;
}

void SPIDeviceManager::selftest()
{
    AP_HAL::SPIDevice *dev = get_device_ptr("icm20948");
    if (dev == nullptr) {
        trace_printf("spi: selftest SKIPPED, no icm20948 device\n");
        return;
    }

    // ICM-20948 WHO_AM_I is bank-independent and reads 0xEA. The read flag is
    // bit 7 of the address byte, same as bench_imu.cpp's BIT_READ.
    dev->set_read_flag(0x80);

    uint8_t who = 0;
    bool ok;
    {
        WITH_SEMAPHORE(dev->get_semaphore());
        ok = dev->read_registers(0x00, &who, 1);
    }

    if (ok && (who == 0xEA)) {
        trace_printf("spi: selftest OK, icm20948 WHO_AM_I=%x via AP_HAL path\n",
                     (uint32_t)who);
    } else {
        trace_printf("spi: selftest FAILED, xfer=%u WHO_AM_I=%x (expected ea) "
                     "-- is Linux still bound to 4b00000.spi?\n",
                     (uint32_t)ok, (uint32_t)who);
    }
    delete dev;
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
