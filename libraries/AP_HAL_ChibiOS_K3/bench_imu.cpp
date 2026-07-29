#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include "bench_imu.h"
#include <hal.h>                // SPID1 = MCU_MCSPI0
#include <string.h>
#include "hwdef/boot/trace.h"

/*
  Bench read-out of the onboard InvenSense ICM-20948 (accel + gyro + die
  temperature) over MCU_MCSPI0 chip select 3, reported to the RemoteProc
  trace buffer.

  Register sequence ported from the T3 Gemstone Linux example
  (examples/imu/icm20948.c), keeping its register order and its default
  ranges so a trace line here can be compared directly against that
  program's output on the same board. Only the bus layer differs: spidev
  ioctls become ChibiOS spiSelect/spiExchange/spiUnselect, and usleep()
  becomes chThdSleepMilliseconds().

  Deliberately NOT an AP_HAL SPIDevice and NOT an AP_InertialSensor
  backend. hal.spi is still Empty::SPIDeviceManager, so AP_InertialSensor
  registers no gyro or accel instance and none of this data reaches the
  EKF, the vehicle code or MAVLink. This exists to prove the bus, the chip
  select, the SPI mode and the IMU enable line on real hardware before that
  bridge is written; the proper backend replaces it.

  Bus ownership, mandatory before this can work: MCU_MCSPI0 is the same
  controller Linux exposes as 4b00000.spi / spidev0.*, and the onboard
  sensors hang off it (CS1 = barometer, CS3 = ICM-20948). Linux must be
  told to let go first, exactly like the PWM peripherals:

      echo 4b00000.spi | sudo tee /sys/bus/platform/drivers/omap2_mcspi/unbind

  Without that, both masters drive the bus and reads return garbage rather
  than an error. Re-bind (same path, `bind`) to hand it back.

  The ICM-20948's enable line (MCU_GPIO0_12, active low) is driven by
  spi0_imu_enable() in the ChibiOS SPI driver, not here -- see
  os/hal/ports/TI/AM67/hal_spi_lld.c.
*/

extern const AP_HAL::HAL& hal;

namespace {

// Bank 0
constexpr uint8_t REG_WHO_AM_I     = 0x00;
constexpr uint8_t REG_USER_CTRL    = 0x03;
constexpr uint8_t REG_PWR_MGMT_1   = 0x06;
constexpr uint8_t REG_ACCEL_OUT    = 0x2D;   // accel, gyro and temp, contiguous
// Bank 2
constexpr uint8_t REG_GYRO_SMPLRT_DIV     = 0x00;
constexpr uint8_t REG_GYRO_CONFIG_1       = 0x01;
constexpr uint8_t REG_ODR_ALIGN_EN        = 0x09;
constexpr uint8_t REG_ACCEL_SMPLRT_DIV_1  = 0x10;
constexpr uint8_t REG_ACCEL_SMPLRT_DIV_2  = 0x11;
constexpr uint8_t REG_ACCEL_CONFIG        = 0x14;
// All banks
constexpr uint8_t REG_BANK_SEL     = 0x7F;

constexpr uint8_t WHO_AM_I_VAL     = 0xEA;
constexpr uint8_t BIT_RESET        = 0x80;   // PWR_MGMT_1
constexpr uint8_t BIT_SLEEP        = 0x40;   // PWR_MGMT_1
constexpr uint8_t BIT_I2C_IF_DIS   = 0x10;   // USER_CTRL, pin the part to SPI
constexpr uint8_t BIT_READ         = 0x80;   // OR into a register address

// Matches icm20948_config_default(): +/-2g, +/-250dps, DLPF 3 (51 Hz
// bandwidth), 1125/(1+10) = 102.3 Hz output data rate. The narrow ranges are
// the example's deliberate choice for resolution; a real AP_InertialSensor
// backend will want +/-16g / +/-2000dps instead.
constexpr uint8_t CFG_ACCEL_FS     = 0;      // ACCE_FS_2G
constexpr uint8_t CFG_GYRO_FS      = 0;      // GYRO_FS_250DPS
constexpr uint8_t CFG_DLPF         = 3;
constexpr uint8_t CFG_SMPLRT_DIV   = 10;

constexpr float ACCEL_SENSITIVITY  = 16384.0f;  // LSB/g at +/-2g
constexpr float GYRO_SENSITIVITY   = 131.0f;    // LSB/(deg/s) at +/-250dps

// The ICM-20948 tolerates 7 MHz for register reads; the Linux hwdef for this
// board uses 4 MHz low-speed / 8 MHz high-speed. Stay at the low-speed value
// while the bus itself is still being proven.
constexpr uint32_t SPI_SPEED_HZ    = 4000000;
constexpr uint8_t  SPI_CS_CHANNEL  = 3;      // CS3 = ICM-20948 (CS1 = baro)

constexpr uint32_t SAMPLE_INTERVAL_MS = 20;   // 50 Hz read
constexpr uint32_t REPORT_INTERVAL_MS = 1000; // 1 Hz trace line

// The trace buffer is 16 KiB and does not wrap, so one line per second is
// about three minutes of visibility. Raise REPORT_INTERVAL_MS if a longer
// run matters more than resolution.

const SPIConfig spicfg = {
    .end_cb     = nullptr,
    .speed      = SPI_SPEED_HZ,
    .mode       = 3,                 // CPOL=1 CPHA=1, per the Linux hwdef
    .cs_channel = SPI_CS_CHANNEL,
};

constexpr uint32_t RETRY_INTERVAL_MS = 2000;

bool imu_present;
bool spi_started;
uint8_t attempts;
int8_t current_bank = -1;

void spi_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    // The controller clocks out len+1 bytes and reads every one of them from
    // the TX buffer, so both buffers must cover the whole transfer.
    uint8_t tx[16] = {};
    uint8_t rx[16] = {};

    if (len > sizeof(tx) - 1) {
        return;
    }
    tx[0] = reg | BIT_READ;

    spiSelect(&SPID1);
    spiExchange(&SPID1, len + 1u, tx, rx);
    spiUnselect(&SPID1);

    memcpy(buf, &rx[1], len);
}

void spi_write(uint8_t reg, uint8_t value)
{
    const uint8_t tx[2] = { reg, value };

    spiSelect(&SPID1);
    spiSend(&SPID1, sizeof(tx), tx);
    spiUnselect(&SPID1);
}

void set_bank(uint8_t bank)
{
    if (current_bank == (int8_t)bank) {
        return;
    }
    spi_write(REG_BANK_SEL, (uint8_t)((bank << 4) & 0x30));
    current_bank = (int8_t)bank;
}

uint8_t read_reg(uint8_t reg)
{
    uint8_t value = 0;
    spi_read(reg, &value, 1);
    return value;
}

/*
  One bring-up attempt. Returns true once the part answers.

  This is retried rather than run once because the ordering is forced on us:
  remoteproc starts this firmware during early kernel boot, long before SSH
  is reachable, so the bus can only be unbound from Linux *after* the R5F is
  already running. Exactly the situation RCOutput::retry_pending() exists
  for on the PWM side (see DR-006).
*/
bool imu_try_bringup()
{
    uint8_t who = 0;

    attempts++;

    spiStart(&SPID1, &spicfg);
    spi_started = true;
    if (!SPID1.ready) {
        // Same failure mode as the PWM peripherals: the module never left
        // reset, which on this SoC means its clock is gated because Linux
        // still owns it.
        if (attempts == 1) {
            trace_printf("AP-K3: imu: MCSPI0 not ready (clock gated? spi not unbound from Linux?), retrying\n");
        }
        return false;
    }

    // Cheap probe before committing to the reset sequence: WHO_AM_I answers
    // in any state, so a bus that is not ours yet costs a few microseconds
    // per retry instead of the 120ms the reset settling below takes.
    current_bank = -1;
    set_bank(0);
    who = read_reg(REG_WHO_AM_I);
    if (who != WHO_AM_I_VAL) {
        // 0x00 or 0xFF here is the signature of a bus nobody is driving:
        // Linux still bound, wrong chip select, or the IMU enable line not
        // asserted. A plausible-but-wrong value would mean a different part.
        if (attempts == 1) {
            trace_printf("AP-K3: imu: WHO_AM_I=%x, expected %x on CS%u, retrying\n",
                         who, WHO_AM_I_VAL, SPI_CS_CHANNEL);
        }
        return false;
    }

    // Reset, then wake. The reset takes tens of milliseconds; reading back
    // earlier returns whatever happened to be on the bus.
    spi_write(REG_PWR_MGMT_1, BIT_RESET);
    chThdSleepMilliseconds(100);

    current_bank = -1;
    set_bank(0);
    spi_write(REG_PWR_MGMT_1, (uint8_t)(read_reg(REG_PWR_MGMT_1) & ~BIT_SLEEP));
    chThdSleepMilliseconds(20);

    // The part auto-detects its host interface and the reset cleared that
    // choice. Pin it to SPI before anything else: until the I2C slave
    // interface is disabled, bus noise can re-select it.
    spi_write(REG_USER_CTRL, (uint8_t)(read_reg(REG_USER_CTRL) | BIT_I2C_IF_DIS));

    // Re-read after the reset: this is what proves the part survived it and
    // is still talking, not just that something answered once.
    who = read_reg(REG_WHO_AM_I);
    if (who != WHO_AM_I_VAL) {
        trace_printf("AP-K3: imu: WHO_AM_I=%x after reset, expected %x, retrying\n",
                     who, WHO_AM_I_VAL);
        return false;
    }
    trace_printf("AP-K3: imu: WHO_AM_I=%x OK on CS%u after %u attempt(s)\n",
                 who, SPI_CS_CHANNEL, attempts);

    // Filters first, ranges second: they share a register, and the range
    // writes preserve the filter bits rather than the other way round.
    set_bank(2);

    // DLPFCFG is bits 5:3 and bit 0 puts the filter in circuit; 0xC6 keeps
    // everything else in the register.
    spi_write(REG_GYRO_CONFIG_1,
              (uint8_t)((read_reg(REG_GYRO_CONFIG_1) & 0xC6) | (CFG_DLPF << 3) | 0x01));
    spi_write(REG_ACCEL_CONFIG,
              (uint8_t)((read_reg(REG_ACCEL_CONFIG) & 0xC6) | (CFG_DLPF << 3) | 0x01));

    // FS_SEL is bits 2:1 in both registers.
    spi_write(REG_GYRO_CONFIG_1,
              (uint8_t)((read_reg(REG_GYRO_CONFIG_1) & ~0x06) | (CFG_GYRO_FS << 1)));
    spi_write(REG_ACCEL_CONFIG,
              (uint8_t)((read_reg(REG_ACCEL_CONFIG) & ~0x06) | (CFG_ACCEL_FS << 1)));

    spi_write(REG_GYRO_SMPLRT_DIV, CFG_SMPLRT_DIV);
    // The accelerometer divider is 12 bits across two registers.
    spi_write(REG_ACCEL_SMPLRT_DIV_1, 0);
    spi_write(REG_ACCEL_SMPLRT_DIV_2, CFG_SMPLRT_DIV);
    // Start both sample clocks together so the samples stay in step.
    spi_write(REG_ODR_ALIGN_EN, 0x01);

    set_bank(0);
    return true;
}

}  // namespace

void ChibiOS_K3::bench_imu_init()
{
    imu_present = imu_try_bringup();
}

void ChibiOS_K3::bench_imu_update()
{
    static uint32_t last_sample_ms;
    static uint32_t last_report_ms;
    static uint32_t last_retry_ms;
    static int16_t ax, ay, az, gx, gy, gz, temp_raw;

    const uint32_t now_ms = AP_HAL::millis();

    if (!imu_present) {
        // Cheap: one failed attempt is a handful of register transactions,
        // and it stops entirely once the part answers.
        if (now_ms - last_retry_ms < RETRY_INTERVAL_MS) {
            return;
        }
        last_retry_ms = now_ms;
        if (spi_started) {
            spiStop(&SPID1);
            spi_started = false;
        }
        current_bank = -1;
        imu_present = imu_try_bringup();
        return;
    }

    if (now_ms - last_sample_ms < SAMPLE_INTERVAL_MS) {
        return;
    }
    last_sample_ms = now_ms;

    // ACCEL_OUT..TEMP_OUT is one contiguous block: 6 accel + 6 gyro + 2 temp.
    uint8_t raw[14];
    set_bank(0);
    spi_read(REG_ACCEL_OUT, raw, sizeof(raw));

    ax = (int16_t)((uint16_t)raw[0]  << 8 | raw[1]);
    ay = (int16_t)((uint16_t)raw[2]  << 8 | raw[3]);
    az = (int16_t)((uint16_t)raw[4]  << 8 | raw[5]);
    gx = (int16_t)((uint16_t)raw[6]  << 8 | raw[7]);
    gy = (int16_t)((uint16_t)raw[8]  << 8 | raw[9]);
    gz = (int16_t)((uint16_t)raw[10] << 8 | raw[11]);
    temp_raw = (int16_t)((uint16_t)raw[12] << 8 | raw[13]);

    if (now_ms - last_report_ms < REPORT_INTERVAL_MS) {
        return;
    }
    last_report_ms = now_ms;

    // trace_printf() supports %d/%u/%x/%c/%s only -- no floats. Report
    // milli-g, milli-deg/s and milli-degC so the values stay readable
    // without a float formatter. At rest, expect one axis near +/-1000 mg
    // (gravity), the other two near 0, and all three gyro axes within a few
    // hundred mdps of zero.
    const int32_t ax_mg = (int32_t)((float)ax * 1000.0f / ACCEL_SENSITIVITY);
    const int32_t ay_mg = (int32_t)((float)ay * 1000.0f / ACCEL_SENSITIVITY);
    const int32_t az_mg = (int32_t)((float)az * 1000.0f / ACCEL_SENSITIVITY);
    const int32_t gx_mdps = (int32_t)((float)gx * 1000.0f / GYRO_SENSITIVITY);
    const int32_t gy_mdps = (int32_t)((float)gy * 1000.0f / GYRO_SENSITIVITY);
    const int32_t gz_mdps = (int32_t)((float)gz * 1000.0f / GYRO_SENSITIVITY);
    const int32_t temp_mc = (int32_t)(((float)temp_raw / 333.87f + 21.0f) * 1000.0f);

    trace_printf("AP-K3: imu a=%d,%d,%d mg g=%d,%d,%d mdps t=%d mC\n",
                 (int)ax_mg, (int)ay_mg, (int)az_mg,
                 (int)gx_mdps, (int)gy_mdps, (int)gz_mdps,
                 (int)temp_mc);
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
