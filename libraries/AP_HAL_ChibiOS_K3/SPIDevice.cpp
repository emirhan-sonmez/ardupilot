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
    { "lps22df",    1,   3,  250000,   250000 },   // onboard barometer (Q-05:
    // ST LPS22DF, not the BMP390
    // the device tree names)
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

/*
  Identify the part actually fitted on CS1 (Q-05).

  The barometer has been documented as two different chips since bring-up: the
  ArduPilot Linux hwdef, the ArduPilot board page and this board's own device
  tree (`pressure@1: bosch,bmp390-spidev`) all say Bosch BMP390, while the T3
  board-spec page says ST LPS22DFTR. Every one of those is a DECLARATION. None
  is a probe, the board has been physically swapped since, and the two parts
  need different drivers.

  Reads both candidates' identity registers and reports what came back, rather
  than testing one and calling absence a failure -- "not a BMP390" and "the bus
  is broken" look identical from a single read.
*/
void SPIDeviceManager::baro_ident()
{
    AP_HAL::SPIDevice *dev = get_device_ptr("lps22df");
    if (dev == nullptr) {
        trace_printf("baro: ident SKIPPED, no lps22df device in the table\n");
        return;
    }
    dev->set_read_flag(0x80);

    /*
      BMP3xx SPI returns a DUMMY byte before the first real data byte, so a
      1-byte read of CHIP_ID yields the dummy and looks like a dead bus. Read
      two and take the second. The ST part has no such quirk, so its WHO_AM_I
      is read separately at its own length.
    */
    uint8_t bosch[2] = {};
    uint8_t st = 0;
    bool ok_b, ok_s;
    {
        WITH_SEMAPHORE(dev->get_semaphore());
        ok_b = dev->read_registers(0x00, bosch, 2);   /* BMP3xx CHIP_ID   */
        ok_s = dev->read_registers(0x0F, &st, 1);     /* LPS22DF WHO_AM_I */
    }

    trace_printf("baro: ident xfer=%u/%u bosch[0x00]=%x,%x st[0x0f]=%x\n",
                 (uint32_t)ok_b, (uint32_t)ok_s,
                 (uint32_t)bosch[0], (uint32_t)bosch[1], (uint32_t)st);

    if (bosch[1] == 0x60) {
        trace_printf("baro: BMP390 confirmed on CS1 (chip_id=0x60) -- Q-05 closed, use AP_Baro_BMP388\n");
    } else if (bosch[1] == 0x50) {
        trace_printf("baro: BMP388 on CS1 (chip_id=0x50), NOT the documented BMP390 -- same driver, note it\n");
    } else if (st == 0xB4) {
        trace_printf("baro: LPS22DF confirmed on CS1 (who_am_i=0xb4) -- Q-05 closed the OTHER way, use AP_Baro_LPS2XH\n");
    } else {
        trace_printf("baro: UNIDENTIFIED on CS1. Neither 0x60/0x50 (Bosch) nor 0xb4 (ST). "
                     "All-zero or all-ff means nothing is driving MISO: wrong chip select, "
                     "part absent, or the bus still Linux's.\n");
    }

    /*
      TEMP-DIAG(baro): configure the LPS22DF here and read it back raw.

      AP_Baro_LPS2XH reports 1307 hPa, a static value, with no temperature --
      which could be a bad conversion, a control register that did not take, a
      misread burst, or the scaling. Those are indistinguishable from the
      frontend. Driving the part directly and printing the register contents
      alongside the raw output bytes separates them: if CTRL_REG1 reads back
      what was written and STATUS shows fresh data, the sensor is fine and the
      fault is in the driver's conversion.

      AP_Baro re-runs its own _init() during setup(), so this configuration is
      overwritten and cannot mislead the real backend.
      REMOVE-AFTER: barometer reports plausible pressure.
    */
    if (st == 0xB4) {
        WITH_SEMAPHORE(dev->get_semaphore());

        dev->write_register(0x10, 0x00);            /* CTRL_REG1: idle       */
        dev->write_register(0x11, 1 << 3);          /* CTRL_REG2: BDU        */
        dev->write_register(0x10, (0x05 << 3) | 0x02); /* 50 Hz, 16x average */
        hal.scheduler->delay(100);

        uint8_t c1 = 0, c2 = 0, stat = 0, ifc = 0;
        uint8_t praw[3] = {}, traw[2] = {};
        dev->read_registers(0x10, &c1, 1);
        dev->read_registers(0x11, &c2, 1);
        dev->read_registers(0x0E, &ifc, 1);
        dev->read_registers(0x27, &stat, 1);
        dev->read_registers(0x28, praw, 3);
        dev->read_registers(0x2B, traw, 2);

        const uint32_t praw_u = ((uint32_t)praw[2] << 16) |
                                ((uint32_t)praw[1] << 8) | praw[0];
        const int16_t traw_s = (int16_t)(((uint16_t)traw[1] << 8) | traw[0]);

        trace_printf("baro: lps22df ctrl1=%x (want 2a) ctrl2=%x (want 08) if_ctrl=%x status=%x\n",
                     (uint32_t)c1, (uint32_t)c2, (uint32_t)ifc, (uint32_t)stat);
        trace_printf("baro: lps22df praw=%x,%x,%x -> %u = %u Pa   traw=%x,%x -> %d = %d cdegC\n",
                     (uint32_t)praw[0], (uint32_t)praw[1], (uint32_t)praw[2],
                     praw_u, (uint32_t)(praw_u / 40.96f),
                     (uint32_t)traw[0], (uint32_t)traw[1],
                     (int32_t)traw_s, (int32_t)traw_s);

        /* Second sample: a value that does not move between reads means the
           part is not converting, whatever the registers claim. */
        hal.scheduler->delay(100);
        uint8_t praw2[3] = {};
        dev->read_registers(0x27, &stat, 1);
        dev->read_registers(0x28, praw2, 3);
        const uint32_t praw2_u = ((uint32_t)praw2[2] << 16) |
                                 ((uint32_t)praw2[1] << 8) | praw2[0];
        trace_printf("baro: lps22df sample2 status=%x praw=%u delta=%d %s\n",
                     (uint32_t)stat, praw2_u, (int32_t)(praw2_u - praw_u),
                     (praw2_u == praw_u) ? "STATIC - not converting" : "changing");
    }

    delete dev;
}

/*
  TEMP-DIAG(Q-35): read-length characterisation over the AP_HAL path.

  Runs here rather than in bench_imu.cpp because that file drives SPID1
  directly and can only run before setup(), which on this board is before
  Linux has unbound omap2_mcspi -- bring-up simply fails that early. Going
  through AP_HAL::SPIDevice means the bus semaphore is held per transaction,
  so this is safe to call at any point in the run, alongside the INS backend.

  Reports instability, not correctness: min(ones, reads-ones) per bit position
  is nonzero only when the reads disagree with each other, so no knowledge of
  the true register contents is needed. Baseline recorded 2026-07-31 was len
  1-2 stable and len 4+ unstable at ~128/256 -- exact alternation, which is
  the signature of a stale word shifting the transaction rather than of
  electrical marginality.
  REMOVE-AFTER: Q-35 closed.
*/
void SPIDeviceManager::bus_length_diag()
{
    static const uint8_t lengths[] = { 1, 2, 4, 6, 8, 14 };
    constexpr uint8_t num_lengths = 6;
    constexpr uint16_t reads = 256;
    constexpr uint8_t max_len = 14;

    AP_HAL::SPIDevice *dev = get_device_ptr("icm20948");
    if (dev == nullptr) {
        trace_printf("spi: lendiag SKIPPED, no icm20948 device\n");
        return;
    }
    dev->set_read_flag(0x80);

    /*
      Correctness gate. This sweep reports *instability* -- min(ones, reads-ones)
      per bit -- which is 0 for constant data of any value. A part that answers
      0x00 to everything therefore scores a perfect STABLE on every length while
      measuring nothing at all, which is exactly what the first run of this
      diagnostic did. Refuse to report numbers unless the part is talking.
    */
    uint8_t who = 0;
    {
        WITH_SEMAPHORE(dev->get_semaphore());
        (void)dev->read_registers(0x00, &who, 1);
    }
    if (who != 0xEA) {
        trace_printf("spi: lendiag ABORT, WHO_AM_I=%x expected ea "
                     "-- part not responding, any STABLE result here is a lie\n",
                     (uint32_t)who);
        delete dev;
        return;
    }

    /* Sample of real bytes, so the trace shows what is being measured rather
       than only how stable it is. */
    {
        uint8_t s[8] = {};
        WITH_SEMAPHORE(dev->get_semaphore());
        if (dev->read_registers(0x00, s, 8)) {
            trace_printf("spi: lendiag sample o0..o7=%x,%x,%x,%x,%x,%x,%x,%x\n",
                         (uint32_t)s[0], (uint32_t)s[1], (uint32_t)s[2],
                         (uint32_t)s[3], (uint32_t)s[4], (uint32_t)s[5],
                         (uint32_t)s[6], (uint32_t)s[7]);
        }
    }

    trace_printf("spi: lendiag start n=%u via AP_HAL path\n", (uint32_t)reads);

    for (uint8_t li = 0; li < num_lengths; li++) {
        const uint8_t len = lengths[li];
        uint16_t ones[max_len][8] = {};
        uint32_t total = 0;
        uint16_t worst = 0;
        uint8_t worst_bit = 0, worst_off = 0;
        uint16_t failed = 0;
        uint16_t anchor_errs = 0;

        for (uint16_t i = 0; i < reads; i++) {
            uint8_t buf[max_len] = {};
            bool ok;
            {
                WITH_SEMAPHORE(dev->get_semaphore());
                ok = dev->read_registers(0x00, buf, len);
            }
            if (!ok) {
                failed++;
                continue;
            }
            /*
              Correctness anchor. Offset 0 is WHO_AM_I, a read-only constant
              0xEA, so every burst carries its own answer key. Instability
              alone cannot see a burst that is uniformly shifted by a stale
              word -- the shifted data is perfectly self-consistent across
              reads -- but the anchor lands on the wrong byte the moment a
              shift happens, which is exactly the fault the RX drain targets.
            */
            if (buf[0] != 0xEA) {
                anchor_errs++;
            }
            for (uint8_t o = 0; o < len; o++) {
                for (uint8_t b = 0; b < 8; b++) {
                    if ((buf[o] >> b) & 1U) {
                        ones[o][b]++;
                    }
                }
            }
        }

        const uint16_t good = (uint16_t)(reads - failed);
        for (uint8_t o = 0; o < len; o++) {
            for (uint8_t b = 0; b < 8; b++) {
                const uint16_t n1 = ones[o][b];
                const uint16_t n0 = (uint16_t)(good - n1);
                const uint16_t minority = (n1 > n0) ? n0 : n1;
                total += minority;
                if (minority > worst) {
                    worst = minority;
                    worst_bit = b;
                    worst_off = o;
                }
            }
        }

        trace_printf("spi: lendiag len=%u unstable=%u worst=b%u@o%u(%u/%u) xferfail=%u anchor=%u %s\n",
                     (uint32_t)len, (uint32_t)total,
                     (uint32_t)worst_bit, (uint32_t)worst_off,
                     (uint32_t)worst, (uint32_t)good, (uint32_t)failed,
                     (uint32_t)anchor_errs,
                     ((total == 0) && (anchor_errs == 0)) ? "STABLE" : "SUSPECT");
    }

    trace_printf("spi: lendiag done\n");
    delete dev;
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
