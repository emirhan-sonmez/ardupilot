#include "AP_InertialSensor_ICM20948_K3.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include <AP_HAL_ChibiOS_K3/hwdef/boot/trace.h>

extern const AP_HAL::HAL &hal;

namespace {

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

    // Z is negated to match the Invensense convention ArduPilot's rotation
    // tables are written against.
    Vector3f accel((float)ax * ACCEL_SCALE,
                   (float)ay * ACCEL_SCALE,
                   -(float)az * ACCEL_SCALE);
    Vector3f gyro((float)gx * GYRO_SCALE,
                  (float)gy * GYRO_SCALE,
                  -(float)gz * GYRO_SCALE);

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
