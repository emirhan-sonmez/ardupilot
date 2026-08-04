#include "AP_InertialSensor_ICM20948_K3.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include <AP_HAL_ChibiOS_K3/hwdef/boot/trace.h>

extern const AP_HAL::HAL &hal;

namespace
{

// Bank 0
constexpr uint8_t REG_WHO_AM_I   = 0x00;
constexpr uint8_t REG_USER_CTRL  = 0x03;
constexpr uint8_t REG_PWR_MGMT_1 = 0x06;
constexpr uint8_t REG_PWR_MGMT_2 = 0x07;
constexpr uint8_t REG_ACCEL_OUT  = 0x2D;   // accel, gyro and temperature, contiguous
// Bank 2
constexpr uint8_t REG_GYRO_SMPLRT_DIV    = 0x00;
constexpr uint8_t REG_GYRO_CONFIG_1      = 0x01;
constexpr uint8_t REG_ODR_ALIGN_EN       = 0x09;
constexpr uint8_t REG_ACCEL_SMPLRT_DIV_2 = 0x11;
constexpr uint8_t REG_ACCEL_CONFIG       = 0x14;
// All banks
constexpr uint8_t REG_BANK_SEL   = 0x7F;

/*
  Auxiliary I2C master registers. Bank 0 holds USER_CTRL and the status
  register; the SLV channels live in bank 3. Mixing the two up reads plausible
  garbage rather than failing, so every access below selects its bank first.
  Values cross-checked against the vendor example in
  ~/Documents/gemstone/examples/imu/icm20948.c, which drives this same part.
*/
// REG_USER_CTRL is already defined above with the other bank 0 registers.
constexpr uint8_t REG_I2C_MST_STATUS = 0x17;   // bank 0
constexpr uint8_t REG_I2C_MST_CTRL   = 0x01;   // bank 3
constexpr uint8_t REG_I2C_SLV4_ADDR  = 0x13;   // bank 3
constexpr uint8_t REG_I2C_SLV4_REG   = 0x14;   // bank 3
constexpr uint8_t REG_I2C_SLV4_CTRL  = 0x15;   // bank 3
constexpr uint8_t REG_I2C_SLV4_DO    = 0x16;   // bank 3
constexpr uint8_t REG_I2C_SLV4_DI    = 0x17;   // bank 3

constexpr uint8_t BIT_I2C_MST_EN     = 0x20;
constexpr uint8_t BIT_I2C_MST_RST    = 0x02;   // self clearing
constexpr uint8_t BIT_I2C_SLVX_EN    = 0x80;
constexpr uint8_t BIT_I2C_SLV4_DONE  = 0x40;
constexpr uint8_t BIT_I2C_SLV4_NACK  = 0x10;
constexpr uint8_t BIT_I2C_READ       = 0x80;   // OR into the slave address

// AK09916 magnetometer die, on the auxiliary bus.
constexpr uint8_t AK09916_I2C_ADDR   = 0x0C;
constexpr uint8_t AK09916_REG_WIA1   = 0x00;   // 0x48, company id
constexpr uint8_t AK09916_REG_WIA2   = 0x01;   // 0x09, device id
constexpr uint8_t AK09916_WIA1_VAL   = 0x48;
constexpr uint8_t AK09916_WIA2_VAL   = 0x09;
constexpr uint8_t AK09916_REG_ST1    = 0x10;   // first byte of the sample block
constexpr uint8_t AK09916_REG_CNTL2  = 0x31;
constexpr uint8_t AK09916_REG_CNTL3  = 0x32;
constexpr uint8_t AK09916_MODE_CONT  = 0x08;   // continuous, matches AP_Compass_AK09916
constexpr uint8_t AK09916_SRST       = 0x01;

/*
  The streamed block is ST1, HXL..HZH, TMPS, ST2 -- 9 bytes from 0x10. ST2 has
  to be inside the read: the AK09916 holds its output registers until ST2 is
  read, so a block that stops short leaves the magnetometer permanently latched
  on one sample.
*/
constexpr uint8_t AK09916_BLOCK_LEN  = 9;

// Bank 0. SLV0's copied bytes land here every master cycle.
constexpr uint8_t REG_EXT_SLV_SENS_DATA_00 = 0x3B;

// Bank 3, SLV0 channel.
constexpr uint8_t REG_I2C_SLV0_ADDR  = 0x03;
constexpr uint8_t REG_I2C_SLV0_REG   = 0x04;
constexpr uint8_t REG_I2C_SLV0_CTRL  = 0x05;

constexpr uint8_t WHO_AM_I_VAL   = 0xEA;
constexpr uint8_t BIT_RESET      = 0x80;   // PWR_MGMT_1
constexpr uint8_t BIT_I2C_IF_DIS = 0x10;   // USER_CTRL, pin the part to SPI

/*
  Ranges are wider than the bench read-out's +/-2g / +/-250dps. That module
  chose narrow ranges for resolution on a stationary bench; a flight-shaped
  backend needs headroom for real motion, and ArduCopter scales from the
  sensitivity constants below either way.
*/
constexpr uint8_t CFG_ACCEL_FS = 3;        // +/-16g
constexpr uint8_t CFG_GYRO_FS  = 3;        // +/-2000 dps
constexpr uint8_t CFG_DLPF     = 3;        // ~51 Hz bandwidth
constexpr uint8_t CFG_SMPLRT_DIV = 10;     // 1125/(1+10) = 102.3 Hz ODR

constexpr float ACCEL_SCALE = GRAVITY_MSS / 2048.0f;          // LSB/g at +/-16g
constexpr float GYRO_SCALE  = radians(1.0f) / 16.4f;          // LSB/(deg/s) at +/-2000dps

/*
  Sample period. The ICM is configured for a 102.3 Hz output data rate, so
  sampling faster only re-reads the same registers.

  The floor here is not the sensor, it is the bus: 14 single-register reads at
  250 kHz cost ~1 ms, and the SPI bus thread's tick is 1 ms
  (CH_CFG_ST_FREQUENCY=1000 with CH_CFG_ST_TIMEDELTA=0). 10 ms leaves a 10x
  margin on both. Raising this is gated on multi-byte reads working or the bus
  running faster -- not on changing this number.
*/
constexpr uint32_t SAMPLE_PERIOD_US = 10000;   // 100 Hz
constexpr uint16_t SAMPLE_RATE_HZ   = 100;

constexpr uint16_t BANK_SETTLE_US = 1000;
constexpr uint16_t SETTLE_US      = 200;

}  // namespace

AP_InertialSensor_ICM20948_K3::AP_InertialSensor_ICM20948_K3(
    AP_InertialSensor &imu,
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
    enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
      /* Order must match the declaration order in the header, not logical
         grouping -- the magnetometer members are declared before _dev. */
    , _mag_field(0.0f, 0.0f, 0.0f)
    , _mag_counter(0)
    , _mag_ok(false)
    , _mag_divider(0)
    , _dev(std::move(dev))
    , _rotation(rotation)
    , _gyro_instance(0)
    , _accel_instance(0)
    , _current_bank(-1)
    , _last_sample_ok(false)
{
}

AP_InertialSensor_Backend *AP_InertialSensor_ICM20948_K3::probe(
    AP_InertialSensor &imu,
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
    enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    AP_InertialSensor_ICM20948_K3 *sensor =
        NEW_NOTHROW AP_InertialSensor_ICM20948_K3(imu, std::move(dev), rotation);
    if (sensor == nullptr) {
        return nullptr;
    }
    if (!sensor->init_sensor()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

/*
  Every register access below is a 1-byte read or a 2-byte write, never a
  block. See the class comment: this is the whole reason the backend exists.
*/
bool AP_InertialSensor_ICM20948_K3::read_reg(uint8_t reg, uint8_t &value)
{
    return _dev->read_registers(reg, &value, 1);
}

/*
  Bring up the ICM's auxiliary I2C master.

  I2C_IF_DIS is deliberately NOT touched here: the primary interface is already
  pinned to SPI during init_sensor(), and re-writing USER_CTRL's other bits
  from a stale read is how a working SPI link gets dropped mid-configuration.
  Read-modify-write, set only I2C_MST_EN.
*/
bool AP_InertialSensor_ICM20948_K3::aux_master_init()
{
    uint8_t user_ctrl = 0;

    if (!select_bank(0) || !read_reg(REG_USER_CTRL, user_ctrl)) {
        return false;
    }

    // Reset the master first. The bit is self clearing; the part needs a
    // moment before the channel registers mean anything.
    if (!_dev->write_register(REG_USER_CTRL,
                              (uint8_t)(user_ctrl | BIT_I2C_MST_RST))) {
        return false;
    }
    hal.scheduler->delay(10);

    if (!_dev->write_register(REG_USER_CTRL,
                              (uint8_t)(user_ctrl | BIT_I2C_MST_EN))) {
        return false;
    }

    /*
      400 kHz on the auxiliary bus. 0x07 is the recommended divider for this
      part and is what the vendor example uses; the AK09916 is rated well
      above it, and the aux bus is entirely internal to the package so it has
      none of the signal-integrity exposure the external SPI wiring has.
    */
    if (!select_bank(3) || !_dev->write_register(REG_I2C_MST_CTRL, 0x07)) {
        return false;
    }

    hal.scheduler->delay(10);
    return true;
}

/*
  One byte to or from a device on the auxiliary bus, through SLV4.

  SLV4 is the configuration channel: unlike SLV0-3 it reports DONE and NACK, so
  a part that is absent or wedged is distinguishable from a bus that is simply
  quiet. That distinction is worth the extra registers -- without it, "no
  magnetometer" and "aux master misconfigured" look identical.
*/
bool AP_InertialSensor_ICM20948_K3::aux_xfer(uint8_t addr, uint8_t reg,
        uint8_t *value, bool is_read)
{
    uint8_t status = 0;

    if (!select_bank(0)) {
        return false;
    }
    /*
      I2C_MST_STATUS clears on read. Drain it before arming, or a DONE left
      over from the previous transaction is mistaken for this one completing
      and the caller reads stale data with no error.
    */
    (void)read_reg(REG_I2C_MST_STATUS, status);

    if (!select_bank(3)) {
        return false;
    }
    if (!_dev->write_register(REG_I2C_SLV4_ADDR,
                              (uint8_t)(is_read ? (addr | BIT_I2C_READ) : addr))) {
        return false;
    }
    if (!_dev->write_register(REG_I2C_SLV4_REG, reg)) {
        return false;
    }
    if (!is_read && !_dev->write_register(REG_I2C_SLV4_DO, *value)) {
        return false;
    }
    if (!_dev->write_register(REG_I2C_SLV4_CTRL, BIT_I2C_SLVX_EN)) {
        return false;
    }

    // Bounded: a transaction that never completes must not wedge the caller.
    bool done = false;
    for (uint8_t i = 0; i < 50; i++) {
        hal.scheduler->delay_microseconds(200);
        if (!select_bank(0) || !read_reg(REG_I2C_MST_STATUS, status)) {
            return false;
        }
        if ((status & BIT_I2C_SLV4_DONE) != 0) {
            done = true;
            break;
        }
    }

    if (!done) {
        // Disarm, or the channel keeps retrying against a dead slave forever.
        if (select_bank(3)) {
            (void)_dev->write_register(REG_I2C_SLV4_CTRL, 0);
        }
        return false;
    }
    if ((status & BIT_I2C_SLV4_NACK) != 0) {
        return false;
    }

    if (is_read) {
        if (!select_bank(3) || !read_reg(REG_I2C_SLV4_DI, *value)) {
            return false;
        }
    }
    return true;
}

/*
  Arm SLV0 to copy a block from an auxiliary device every master cycle.

  SLV4 is fine for configuration but hopeless as a data path: each transaction
  needs its own arm-and-poll round trip, so a 9-byte sample would cost nine of
  them. SLV0 runs autonomously and drops the bytes into EXT_SLV_SENS_DATA_00,
  which the host then reads as an ordinary register block.
*/
bool AP_InertialSensor_ICM20948_K3::aux_slv0_stream(uint8_t addr, uint8_t reg,
        uint8_t len)
{
    if (!select_bank(3)) {
        return false;
    }
    if (!_dev->write_register(REG_I2C_SLV0_ADDR, (uint8_t)(addr | BIT_I2C_READ))) {
        return false;
    }
    if (!_dev->write_register(REG_I2C_SLV0_REG, reg)) {
        return false;
    }
    // Length lives in the low nibble of CTRL, alongside the enable bit.
    return _dev->write_register(REG_I2C_SLV0_CTRL,
                                (uint8_t)(BIT_I2C_SLVX_EN | (len & 0x0F)));
}

/*
  Put the AK09916 into continuous mode and stream its samples.

  Mode and register values match AP_Compass_AK09916 so that a compass backend
  built on this sees exactly what the stock driver would.
*/
bool AP_InertialSensor_ICM20948_K3::aux_start_ak09916()
{
    if (!aux_write(AK09916_I2C_ADDR, AK09916_REG_CNTL3, AK09916_SRST)) {
        return false;
    }
    hal.scheduler->delay(10);

    if (!aux_write(AK09916_I2C_ADDR, AK09916_REG_CNTL2, AK09916_MODE_CONT)) {
        return false;
    }
    hal.scheduler->delay(10);

    return aux_slv0_stream(AK09916_I2C_ADDR, AK09916_REG_ST1, AK09916_BLOCK_LEN);
}

/*
  Copy the most recent streamed magnetometer block out of the ICM.
*/
bool AP_InertialSensor_ICM20948_K3::aux_read_mag(uint8_t *buf)
{
    if (!select_bank(0)) {
        return false;
    }
    return _dev->read_registers(REG_EXT_SLV_SENS_DATA_00, buf, AK09916_BLOCK_LEN);
}

AP_InertialSensor_ICM20948_K3 *AP_InertialSensor_ICM20948_K3::_singleton;

bool AP_InertialSensor_ICM20948_K3::get_mag_field(Vector3f &field,
        uint32_t &counter) const
{
    if (!_mag_ok) {
        return false;
    }
    field = _mag_field;
    counter = _mag_counter;
    return true;
}

/*
  Pull one magnetometer block, called from the IMU sample path.

  Rate-divided to half the IMU rate: the AK09916 runs at 100 Hz and a compass
  gains nothing from being read faster than it updates, while every read costs
  9 bytes on a 250 kHz bus shared with the gyro.

  Deliberately does NOT gate on ST1's data-ready bit. SLV0 performs its own
  read on the auxiliary side and consumes DRDY doing so, so by the time the
  host reads EXT_SLV_SENS_DATA the flag has already been cleared -- measured
  on hardware, ST1 reads 0x00 or 0x02 (overrun) while the field values are
  demonstrably updating. Requiring DRDY here, as AP_Compass_AK09916 does on a
  directly-attached part, would reject every sample.

  ST2 overflow IS honoured: that flag means the reading is out of range and
  genuinely must not be used.
*/
void AP_InertialSensor_ICM20948_K3::mag_sample()
{
    uint8_t b[AK09916_BLOCK_LEN];

    if (++_mag_divider < 2) {
        return;
    }
    _mag_divider = 0;

    if (!aux_read_mag(b)) {
        return;
    }
    if ((b[8] & 0x08) != 0) {          // ST2 HOFL, magnetic overflow
        return;
    }

    const int16_t mx = (int16_t)((uint16_t)b[2] << 8 | b[1]);
    const int16_t my = (int16_t)((uint16_t)b[4] << 8 | b[3]);
    const int16_t mz = (int16_t)((uint16_t)b[6] << 8 | b[5]);

    if (mx == 0 && my == 0 && mz == 0) {
        return;
    }

    _mag_field = Vector3f((float)mx, (float)my, (float)mz);
    _mag_counter++;
    _mag_ok = true;

}

/*
  Report whether the AK09916 answers on the auxiliary bus.

  Reads BOTH identity registers rather than one: WIA1 is a fixed company code
  and WIA2 the device code, so a part that answers one but not the other is a
  different device rather than a bus fault -- the same reasoning that settled
  Q-05 on the barometer.
*/
void AP_InertialSensor_ICM20948_K3::aux_probe_ak09916()
{
    uint8_t wia1 = 0, wia2 = 0;

    if (!aux_master_init()) {
        trace_printf("AP-K3: mag: aux I2C master init FAILED\n");
        return;
    }

    /*
      Read the master's own configuration back before blaming the
      magnetometer. A probe that returns zeros cannot distinguish "the
      AK09916 is not answering" from "the I2C master was never enabled", and
      on 2026-08-02 a boot failed here with xfer=0/0 while the IMU itself was
      configured and sampling normally.
    */
    {
        uint8_t uc = 0, mc = 0;
        (void)select_bank(0);
        (void)read_reg(REG_USER_CTRL, uc);
        (void)select_bank(3);
        (void)read_reg(REG_I2C_MST_CTRL, mc);
        trace_printf("AP-K3: mag: user_ctrl=%x (want bit5 set) mst_ctrl=%x (want 07)\n",
                     (uint32_t)uc, (uint32_t)mc);
    }

    /*
      Retry the identity read. The aux master shares the die with an IMU that
      has only just been reset and configured, and a single attempt gives no
      way to tell a slow start from a dead bus.
    */
    bool ok1 = false, ok2 = false;
    for (uint8_t attempt = 0; attempt < 5; attempt++) {
        ok1 = aux_read(AK09916_I2C_ADDR, AK09916_REG_WIA1, wia1);
        ok2 = aux_read(AK09916_I2C_ADDR, AK09916_REG_WIA2, wia2);
        if (ok1 && ok2 && wia1 == AK09916_WIA1_VAL && wia2 == AK09916_WIA2_VAL) {
            break;
        }
        trace_printf("AP-K3: mag: probe attempt %u failed (xfer=%u/%u wia=%x,%x), retrying\n",
                     (uint32_t)(attempt + 1), (uint32_t)ok1, (uint32_t)ok2,
                     (uint32_t)wia1, (uint32_t)wia2);
        hal.scheduler->delay(20);
        (void)aux_master_init();
    }

    trace_printf("AP-K3: mag: ak09916 probe xfer=%u/%u wia1=%x (want 48) wia2=%x (want 09)\n",
                 (uint32_t)ok1, (uint32_t)ok2, (uint32_t)wia1, (uint32_t)wia2);

    if (ok1 && ok2 && wia1 == AK09916_WIA1_VAL && wia2 == AK09916_WIA2_VAL) {
        trace_printf("AP-K3: mag: AK09916 PRESENT on aux bus at 0x0c\n");

        if (!aux_start_ak09916()) {
            trace_printf("AP-K3: mag: continuous-mode start FAILED\n");
            return;
        }
        hal.scheduler->delay(50);

        /*
          Two samples, spaced. Field values that are plausible AND move between
          reads prove the whole path: SLV0 is cycling, ST2 is being read so the
          part is not latched, and the block is not a stale snapshot. A single
          reading cannot distinguish live data from one frozen sample -- which
          is exactly how the barometer looked healthy while it was dead.
        */
        for (uint8_t i = 0; i < 2; i++) {
            uint8_t b[AK09916_BLOCK_LEN] = {};
            if (!aux_read_mag(b)) {
                trace_printf("AP-K3: mag: block read FAILED\n");
                return;
            }
            const int16_t mx = (int16_t)((uint16_t)b[2] << 8 | b[1]);
            const int16_t my = (int16_t)((uint16_t)b[4] << 8 | b[3]);
            const int16_t mz = (int16_t)((uint16_t)b[6] << 8 | b[5]);
            trace_printf("AP-K3: mag: st1=%x x=%d y=%d z=%d st2=%x (%d,%d,%d mGauss)\n",
                         (uint32_t)b[0], (int32_t)mx, (int32_t)my, (int32_t)mz,
                         (uint32_t)b[8],
                         (int32_t)(mx * 3 / 2), (int32_t)(my * 3 / 2),
                         (int32_t)(mz * 3 / 2));
            hal.scheduler->delay(100);
        }
    } else {
        trace_printf("AP-K3: mag: AK09916 not identified. Both zero means the aux "
                     "master is not running; NACK means nothing answers at 0x0c\n");
    }
}

bool AP_InertialSensor_ICM20948_K3::select_bank(uint8_t bank)
{
    if (_current_bank == (int8_t)bank) {
        return true;
    }
    if (!_dev->write_register(REG_BANK_SEL, (uint8_t)((bank << 4) & 0x30))) {
        return false;
    }
    _current_bank = (int8_t)bank;
    /*
      A bank switch is not an ordinary write: everything after it is addressed
      through the new bank, and the first transaction following one was
      observed on this hardware not to take (GYRO_CONFIG_1, the first write
      after switching to bank 2, read back as 0 every time while a later write
      in the same bank stuck).
    */
    hal.scheduler->delay_microseconds(BANK_SETTLE_US);
    return true;
}

/*
  Write, read back, and report whether it held.

  Worth being explicit about what this does and does not prove. On this bus,
  writes land reliably -- verified 2026-07-31 by writing a pattern once and
  reading it back 256 times, at both 250 kHz and 1 MHz. What used to fail was
  the verification read, which is why the bench module's identical check
  reported "configuration did not stick" for weeks against writes that had in
  fact stuck. Single-register reads are the reliable shape, so this check is
  now trustworthy; it would not have been if it read back in a block.
*/
bool AP_InertialSensor_ICM20948_K3::write_reg_verified(uint8_t reg, uint8_t value)
{
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        if (!_dev->write_register(reg, value)) {
            continue;
        }
        hal.scheduler->delay_microseconds(SETTLE_US);

        uint8_t readback = 0;
        if (read_reg(reg, readback) && readback == value) {
            return true;
        }
        trace_printf("AP-K3: ins20948: reg %x wrote %x read %x (attempt %u)\n",
                     (uint32_t)reg, (uint32_t)value, (uint32_t)readback,
                     (uint32_t)(attempt + 1));
    }
    return false;
}

bool AP_InertialSensor_ICM20948_K3::init_sensor()
{
    AP_HAL::Semaphore *sem = _dev->get_semaphore();
    WITH_SEMAPHORE(sem);

    _dev->set_read_flag(0x80);
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    _current_bank = -1;
    if (!select_bank(0)) {
        trace_printf("AP-K3: ins20948: bank select failed\n");
        return false;
    }

    uint8_t who = 0;
    if (!read_reg(REG_WHO_AM_I, who) || who != WHO_AM_I_VAL) {
        trace_printf("AP-K3: ins20948: WHO_AM_I=%x expected ea, not probing\n",
                     (uint32_t)who);
        return false;
    }

    // Reset, then wait. The part does not answer meaningfully while resetting
    // and the datasheet's 100 ms is not negotiable here.
    _dev->write_register(REG_PWR_MGMT_1, BIT_RESET);
    hal.scheduler->delay(100);
    _current_bank = -1;

    // Out of sleep, auto-select the best available clock.
    if (!write_reg_verified(REG_PWR_MGMT_1, 0x01)) {
        trace_printf("AP-K3: ins20948: could not clear sleep\n");
        return false;
    }
    // Accel and gyro both powered: PWR_MGMT_2 disable bits are [5:3] gyro,
    // [2:0] accel, so zero means everything on.
    if (!write_reg_verified(REG_PWR_MGMT_2, 0x00)) {
        return false;
    }
    // Pin the part to SPI. Without this it can still answer on I2C and the
    // two interfaces fight over the same register file.
    if (!write_reg_verified(REG_USER_CTRL, BIT_I2C_IF_DIS)) {
        return false;
    }

    if (!select_bank(2)) {
        return false;
    }
    if (!write_reg_verified(REG_GYRO_SMPLRT_DIV, CFG_SMPLRT_DIV) ||
        !write_reg_verified(REG_GYRO_CONFIG_1,
                            (uint8_t)((CFG_DLPF << 3) | (CFG_GYRO_FS << 1) | 1)) ||
        !write_reg_verified(REG_ACCEL_SMPLRT_DIV_2, CFG_SMPLRT_DIV) ||
        !write_reg_verified(REG_ACCEL_CONFIG,
                            (uint8_t)((CFG_DLPF << 3) | (CFG_ACCEL_FS << 1) | 1)) ||
        !write_reg_verified(REG_ODR_ALIGN_EN, 0x01)) {
        trace_printf("AP-K3: ins20948: configuration did not hold\n");
        return false;
    }

    if (!select_bank(0)) {
        return false;
    }

    trace_printf("AP-K3: ins20948: configured, +/-16g +/-2000dps ODR 102Hz, sampling at %u Hz\n",
                 (uint32_t)SAMPLE_RATE_HZ);
    return true;
}

void AP_InertialSensor_ICM20948_K3::start()
{
    if (!_imu.register_gyro(_gyro_instance, SAMPLE_RATE_HZ,
                            _dev->get_bus_id_devtype(DEVTYPE_INS_ICM20948)) ||
        !_imu.register_accel(_accel_instance, SAMPLE_RATE_HZ,
                             _dev->get_bus_id_devtype(DEVTYPE_INS_ICM20948))) {
        return;
    }

    set_gyro_orientation(_gyro_instance, _rotation);
    set_accel_orientation(_accel_instance, _rotation);

    _dev->register_periodic_callback(
        SAMPLE_PERIOD_US,
        FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ICM20948_K3::sample, void));

    /*
      Probe the magnetometer die once the IMU itself is configured and
      sampling. Reports only -- no compass backend is registered yet (T16) --
      but it proves the auxiliary I2C path end to end, which is the part of
      that work that could not be done any other way.
    */
    _singleton = this;
    aux_probe_ak09916();

    trace_printf("AP-K3: ins20948: started, gyro inst %u accel inst %u\n",
                 (uint32_t)_gyro_instance, (uint32_t)_accel_instance);
}

/*
  Runs on the SPI bus thread. Reads ACCEL_XOUT_H..GYRO_ZOUT_L as twelve
  separate single-register transactions.

  This is the expensive part -- ~12 register reads at ~64 us each is roughly
  0.8 ms of busy-polled SPI per sample, ~8% of the core at 100 Hz. That cost
  is the reason this backend cannot scale to flight rates, and it is paid
  deliberately: the 14-byte block read this replaces returned silently
  corrupted data (one bit in the high byte of every axis, so 256 LSB, so a
  bimodal attitude estimate).

  Temperature is not read. It costs two more transactions and nothing in the
  bench path consumes it.
*/
void AP_InertialSensor_ICM20948_K3::sample()
{
    uint8_t raw[12];
    bool ok = true;

    if (!select_bank(0)) {
        _last_sample_ok = false;
        return;
    }

    for (uint8_t i = 0; i < sizeof(raw); i++) {
        if (!read_reg((uint8_t)(REG_ACCEL_OUT + i), raw[i])) {
            ok = false;
            break;
        }
    }

    _last_sample_ok = ok;
    if (!ok) {
        return;
    }

    const int16_t ax = (int16_t)((uint16_t)raw[0] << 8 | raw[1]);
    const int16_t ay = (int16_t)((uint16_t)raw[2] << 8 | raw[3]);
    const int16_t az = (int16_t)((uint16_t)raw[4] << 8 | raw[5]);
    const int16_t gx = (int16_t)((uint16_t)raw[6] << 8 | raw[7]);
    const int16_t gy = (int16_t)((uint16_t)raw[8] << 8 | raw[9]);
    const int16_t gz = (int16_t)((uint16_t)raw[10] << 8 | raw[11]);

    /*
      Swap X and Y, and negate Z. Both halves are required and this must match
      AP_InertialSensor_Invensensev2 exactly, because the rotation constants in
      ArduPilot's tables -- including the one this board inherits from the
      ArduPilot Linux hwdef -- are written against that driver's output frame.

      Negating Z alone, as this did until 2026-08-02, is diag(1,1,-1): a
      determinant of -1, so a REFLECTION rather than a rotation. It yields a
      left-handed frame that no ROTATION_* value can correct, because rotations
      cannot undo a reflection. The stock mapping (Y, X, -Z) has determinant
      +1 and is a proper rotation.

      Symptom it caused: a level, upright board reported accel Z = +990 mg
      instead of -1000 mg, and the AHRS held roll at 178 degrees.
    */
    Vector3f accel((float)ay * ACCEL_SCALE,
                   (float)ax * ACCEL_SCALE,
                   -(float)az * ACCEL_SCALE);
    Vector3f gyro((float)gy * GYRO_SCALE,
                  (float)gx * GYRO_SCALE,
                  -(float)gz * GYRO_SCALE);

    /*
      Apply the board rotation and the calibration offsets/scales.

      set_accel_orientation() only RECORDS the rotation for the frontend; it
      does not apply it. Every stock backend calls _rotate_and_correct_*()
      explicitly before notifying, and omitting it here meant the board
      rotation was silently ignored for the life of this driver -- changing
      the constant passed to probe() had no effect whatsoever.

      Measured consequence: with the ICM-20948 mounted inverted on this PCB
      (raw accel Z reads -1g with the board flat and upright), the unrotated
      output put Z at +990 mg and parked the AHRS at 177 degrees of roll.
    */
    // Magnetometer rides the same bus and the same cadence; see mag_sample().
    mag_sample();

    _rotate_and_correct_accel(_accel_instance, accel);
    _rotate_and_correct_gyro(_gyro_instance, gyro);

    const uint64_t now_us = AP_HAL::micros64();
    _notify_new_accel_raw_sample(_accel_instance, accel, now_us);
    _notify_new_gyro_raw_sample(_gyro_instance, gyro, now_us);
}

bool AP_InertialSensor_ICM20948_K3::update()
{
    update_accel(_accel_instance);
    update_gyro(_gyro_instance);
    return true;
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
