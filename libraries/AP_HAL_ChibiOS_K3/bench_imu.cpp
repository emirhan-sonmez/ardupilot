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
  ioctls become polled ChibiOS transfers (see the note on spiPolledExchange
  below), and usleep() becomes chThdSleepMilliseconds().

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

  The ICM-20948's enable line (MCU_GPIO0_12, active low) is asserted from
  here via am67_spi0_imu_enable(), defined in the ChibiOS SPI driver
  (os/hal/ports/TI/AM67/hal_spi_lld.c) but deliberately not called from
  spiStart(): it touches MCU_GPIO0, a peripheral neither layer owns.
*/

extern const AP_HAL::HAL& hal;

namespace {

// Bank 0
constexpr uint8_t REG_WHO_AM_I     = 0x00;
constexpr uint8_t REG_USER_CTRL    = 0x03;
constexpr uint8_t REG_LP_CONFIG    = 0x05;
constexpr uint8_t REG_PWR_MGMT_1   = 0x06;
constexpr uint8_t REG_PWR_MGMT_2   = 0x07;
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

constexpr uint8_t  SPI_CS_CHANNEL  = 3;      // CS3 = ICM-20948 (CS1 = baro)

/*
  Bus rate: 250 kHz, settled by measurement rather than by the datasheet.

  bus_check() (below) reads WHO_AM_I 32 times and counts correct answers.
  On hardware:

      4 MHz     0/32     every single read wrong
      1 MHz    32/32     clean for single-byte reads -- but multi-byte
                          transactions (a register address byte followed by
                          one or more data/dummy bytes in the same CS
                          assertion) corrupted individual bits past the
                          first byte, non-deterministically: the same
                          write/read pair returned different partial values
                          across attempts (Open Questions Q-33). Config
                          writes to the IMU are always multi-byte (address +
                          value), so this silently broke every write while
                          single-register reads kept looking clean.
    250 kHz    32/32     clean, including multi-byte config writes and the
                          14-byte accel/gyro/temp burst read -- confirmed on
                          hardware, stable readings over many samples.

  4 MHz is what the Linux hwdef lists as this part's low-speed rate, and it
  is what this module used for its first three hardware runs -- which is
  why those runs saw the same register read back as 0xff, 0x18 and 0x00,
  and why WHO_AM_I itself intermittently returned garbage. Linux reaches
  4 MHz with a kernel driver doing DMA and hardware-timed chip select; this
  is a polled register loop, which is not the same electrical duty cycle.
  1 MHz is what the NuttX AM67 port uses for these parts on this controller,
  but that reference never issues a multi-byte write over this bus, only
  single-word transfers -- so the 1 MHz choice never exercised the failure
  mode found here. Root cause is bus-timing/signal margin at 1 MHz on this
  board's CS3 wiring, not a driver logic bug (see Decision Log).

  Driving the IMU enable line made no measurable difference: 32/32 either
  way, and identical write behaviour. It is still asserted, because NuttX's
  board table says it is what activates the part and there is no cost to
  being right about it, but it is not load-bearing for the bus working.
*/
constexpr uint32_t SPI_SPEED_HZ = 250000;

constexpr uint32_t SAMPLE_INTERVAL_MS = 20;   // 50 Hz read
constexpr uint32_t REPORT_INTERVAL_MS = 1000; // 1 Hz trace line

/*
  Runtime sample sanity check. Even at 250 kHz (DR-013), corruption has
  been observed to resume mid-run, not just at bring-up -- every corrupted
  sample seen on hardware (2026-07-30, ArduCopter session) showed at least
  one gyro axis in the tens of thousands of mdps on a bench-mounted, still
  board; every genuine sample stayed under ~1000 mdps. 5000 is a wide
  margin on both sides of that observed split, not a guess.

  Without this, a bad SPI burst was silently reported as real data forever
  -- imu_present never went back to false, so nothing ever re-validated
  the bus once initial bring-up succeeded once.
*/
constexpr int32_t IMU_GYRO_SANITY_MDPS = 5000;
constexpr uint8_t IMU_BAD_SAMPLES_TO_RESYNC = 3;

// The trace buffer is 16 KiB and does not wrap, so one line per second is
// about three minutes of visibility. Raise REPORT_INTERVAL_MS if a longer
// run matters more than resolution.

SPIConfig spicfg = {
    .end_cb     = nullptr,
    .speed      = SPI_SPEED_HZ,
    .mode       = 3,                 // CPOL=1 CPHA=1, per the Linux hwdef.
                                     // NuttX maps CPOL/CPHA to the McSPI
                                     // POL/PHA bits the same way, so this
                                     // encoding is not in question.
    .cs_channel = SPI_CS_CHANNEL,
};

constexpr uint32_t RETRY_INTERVAL_MS = 2000;

// Inter-transaction settling. Bring-up runs a few dozen transactions once,
// so even the generous bank-switch value is invisible; the sample path in
// bench_imu_update() is a single burst read and pays SETTLE_US once.
constexpr uint16_t SETTLE_US      = 200;
constexpr uint16_t BANK_SETTLE_US = 1000;
constexpr uint8_t  WRITE_RETRIES  = 3;
constexpr uint8_t  BUS_CHECK_SAMPLES = 32;

bool imu_present;
bool spi_started;
uint8_t attempts;
int8_t current_bank = -1;

/*
  Every transfer here is polled (spiPolledExchange), never the driver's
  interrupt-driven spiExchange/spiSend.

  spiExchange() sleeps the calling thread until the transfer-complete
  interrupt arrives and has no timeout: if the interrupt never comes -- a
  gated module clock, a channel that never asserts RX_FULL -- the main
  thread is gone for good and the board looks frozen with no diagnostic at
  all. That is exactly what the first attempt at this did: the trace ended
  at "6 RCOutput channels safe-initialized" with nothing after it.

  spi_lld_polled_exchange() busy-waits on CHSTAT with a bounded loop and
  reports SPID1.xfer_timeout, so a dead bus costs milliseconds and says so.
  At 50 Hz over 15 bytes the extra CPU is irrelevant. The interrupt path
  stays in the driver for the real AP_HAL SPIDevice to use later, once the
  bus itself is proven.
*/
bool xfer_failed;

uint8_t spi_xfer_byte(uint8_t out)
{
    const uint8_t in = (uint8_t)spiPolledExchange(&SPID1, out);
    if (SPID1.xfer_timeout) {
        xfer_failed = true;
    }
    return in;
}

void spi_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    spiSelect(&SPID1);
    spi_xfer_byte((uint8_t)(reg | BIT_READ));
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = spi_xfer_byte(0);
    }
    spiUnselect(&SPID1);
}

void spi_write(uint8_t reg, uint8_t value)
{
    spiSelect(&SPID1);
    spi_xfer_byte(reg);
    spi_xfer_byte(value);
    spiUnselect(&SPID1);
    // Settling time between transactions. The part needs the chip select
    // high for a minimum period, and this costs nothing at bring-up rates.
    hal.scheduler->delay_microseconds(SETTLE_US);
}

void set_bank(uint8_t bank)
{
    if (current_bank == (int8_t)bank) {
        return;
    }
    spi_write(REG_BANK_SEL, (uint8_t)((bank << 4) & 0x30));
    current_bank = (int8_t)bank;
    // A bank switch is not an ordinary write: everything after it is
    // addressed through the new bank, and the first transaction following
    // it was observed on hardware not to take (GYRO_CONFIG_1, the first
    // write after switching to bank 2, read back as 0 every time while a
    // later write in the same bank stuck).
    hal.scheduler->delay_microseconds(BANK_SETTLE_US);
}

uint8_t read_reg(uint8_t reg)
{
    uint8_t value = 0;
    spi_read(reg, &value, 1);
    hal.scheduler->delay_microseconds(SETTLE_US);
    return value;
}

/*
  Reads WHO_AM_I repeatedly and counts how many come back correct.

  A single successful WHO_AM_I proves the wiring but says nothing about
  whether the bus is reliable, and "mostly works" is the failure mode that
  wastes the most time: it looks like a driver bug in whatever code happens
  to run next. One number here separates "bus is marginal" from "the
  register sequence is wrong" immediately.
*/
uint8_t bus_check(uint8_t samples)
{
    uint8_t good = 0;

    set_bank(0);
    for (uint8_t i = 0; i < samples; i++) {
        if (read_reg(REG_WHO_AM_I) == WHO_AM_I_VAL) {
            good++;
        }
    }
    return good;
}

/*
  Dumps the registers that determine whether a configuration write can take
  effect at all, plus the bank-select register itself.

  BANK_SEL is the important one. Every register address below 0x7F means a
  different thing per bank, so if that write is not holding, every
  conclusion drawn from a readback is wrong -- and it is the one register
  whose correctness was assumed rather than checked.
*/
void dump_state()
{
    set_bank(0);
    const uint8_t bank_rb   = read_reg(REG_BANK_SEL);
    const uint8_t user_ctrl = read_reg(REG_USER_CTRL);
    const uint8_t lp_config = read_reg(REG_LP_CONFIG);
    const uint8_t pwr1      = read_reg(REG_PWR_MGMT_1);
    const uint8_t pwr2      = read_reg(REG_PWR_MGMT_2);

    trace_printf("AP-K3: imu: bank0 sel=%x user_ctrl=%x lp_cfg=%x pwr1=%x pwr2=%x\n",
                 bank_rb, user_ctrl, lp_config, pwr1, pwr2);

    set_bank(2);
    const uint8_t bank2_sel = read_reg(REG_BANK_SEL);
    const uint8_t smplrt    = read_reg(REG_GYRO_SMPLRT_DIV);
    const uint8_t gyro_cfg  = read_reg(REG_GYRO_CONFIG_1);
    const uint8_t odr_align = read_reg(REG_ODR_ALIGN_EN);
    const uint8_t accel_d2  = read_reg(REG_ACCEL_SMPLRT_DIV_2);
    const uint8_t accel_cfg = read_reg(REG_ACCEL_CONFIG);

    trace_printf("AP-K3: imu: bank2 sel=%x smplrt=%x gyrocfg=%x odralign=%x acceld2=%x accelcfg=%x\n",
                 bank2_sel, smplrt, gyro_cfg, odr_align, accel_d2, accel_cfg);
}

// Write, read back, retry. Returns the value finally read, and traces every
// attempt that did not take -- a configuration write that silently fails
// surfaces much later as data of plausible shape and the wrong scale, which
// is far more expensive to debug than a loud failure here.
bool write_verified(uint8_t reg, uint8_t value, bool verbose)
{
    for (uint8_t i = 0; i < WRITE_RETRIES; i++) {
        spi_write(reg, value);
        const uint8_t rb = read_reg(reg);
        if (rb == value) {
            if (verbose && i > 0) {
                trace_printf("AP-K3: imu: reg %x took %u attempt(s)\n", reg, i + 1);
            }
            return true;
        }
        if (verbose) {
            trace_printf("AP-K3: imu: reg %x wrote %x read %x (attempt %u)\n",
                         reg, value, rb, i + 1);
        }
    }
    return false;
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

    // Step-by-step trace while the bus itself is still in question. "Which
    // register access was the last one to complete" is the only thing that
    // distinguishes a gated clock from a bus fault from a hung transfer.
    const bool verbose = (attempts <= 4);

    if (verbose) {
        trace_printf("AP-K3: imu: attempt %u at %u Hz\n", attempts, SPI_SPEED_HZ);
    }

    am67_spi0_imu_enable();
    spiStart(&SPID1, &spicfg);
    spi_started = true;
    if (!SPID1.ready) {
        // Same failure mode as the PWM peripherals: the module never left
        // reset, which on this SoC means its clock is gated because Linux
        // still owns it.
        if (verbose) {
            trace_printf("AP-K3: imu: MCSPI0 not ready (clock gated? spi not unbound from Linux?), retrying\n");
        }
        return false;
    }
    if (verbose) {
        trace_printf("AP-K3: imu: MCSPI0 ready, probing CS%u...\n", SPI_CS_CHANNEL);
    }

    // Cheap probe before committing to the reset sequence: WHO_AM_I answers
    // in any state, so a bus that is not ours yet costs milliseconds per
    // retry instead of the 120ms the reset settling below takes.
    xfer_failed = false;
    current_bank = -1;
    set_bank(0);
    who = read_reg(REG_WHO_AM_I);
    if (xfer_failed) {
        // CHSTAT never reported the transfer done: the module is mapped and
        // out of reset, but nothing is clocking. Distinct from a wrong
        // WHO_AM_I value, which means the bus works and the part does not
        // answer.
        if (verbose) {
            trace_printf("AP-K3: imu: SPI transfer timed out on CS%u (module clocked but not transferring), retrying\n",
                         SPI_CS_CHANNEL);
        }
        return false;
    }
    if (who != WHO_AM_I_VAL) {
        // 0x00 or 0xFF here is the signature of a bus nobody is driving:
        // Linux still bound, wrong chip select, or the IMU enable line not
        // asserted. A plausible-but-wrong value would mean a different part.
        if (verbose) {
            trace_printf("AP-K3: imu: WHO_AM_I=%x, expected %x on CS%u, retrying\n",
                         who, WHO_AM_I_VAL, SPI_CS_CHANNEL);
        }
        return false;
    }

    // The part is there. Now find out whether the bus is actually reliable
    // before trusting anything written over it.
    const uint8_t good = bus_check(BUS_CHECK_SAMPLES);
    if (verbose) {
        trace_printf("AP-K3: imu: bus check %u/%u at %u Hz\n",
                     good, BUS_CHECK_SAMPLES, SPI_SPEED_HZ);
    }
    if (good < BUS_CHECK_SAMPLES) {
        // Anything short of perfect is a marginal bus. Configuration writes
        // over it would sometimes stick and sometimes not, which is exactly
        // the failure this module already spent two hardware runs on.
        if (verbose) {
            trace_printf("AP-K3: imu: bus not clean at %u Hz, retrying\n",
                         SPI_SPEED_HZ);
        }
        return false;
    }

    // Reset, then wait for the part to come back. Nothing read during the
    // reset is trustworthy: the first version of this did a read-modify-
    // write on PWR_MGMT_1 immediately afterwards and got WHO_AM_I=0x0f out
    // the other side, because whatever garbage the read returned went
    // straight back into the register (RESET is a bit in that same
    // register, so a bad read can re-trigger the reset indefinitely).
    //
    // So: no read-modify-write anywhere in bring-up. Every write below is a
    // constant, and the part has to prove it is alive by answering WHO_AM_I
    // before any of them happen.
    spi_write(REG_PWR_MGMT_1, BIT_RESET);
    chThdSleepMilliseconds(100);

    current_bank = -1;
    who = 0;
    for (uint8_t i = 0; i < 20; i++) {
        set_bank(0);
        who = read_reg(REG_WHO_AM_I);
        if (who == WHO_AM_I_VAL) {
            break;
        }
        chThdSleepMilliseconds(10);
    }
    if (who != WHO_AM_I_VAL) {
        trace_printf("AP-K3: imu: WHO_AM_I=%x 200ms after reset, expected %x, retrying\n",
                     who, WHO_AM_I_VAL);
        return false;
    }

    // Wake with auto clock select (PLL if available, internal otherwise) --
    // the value ArduPilot's own Invensensev2 driver writes here. Bit 6
    // (SLEEP) clear is what actually starts the sensors.
    spi_write(REG_PWR_MGMT_1, 0x01);
    chThdSleepMilliseconds(20);

    // The part auto-detects its host interface and the reset cleared that
    // choice. Pin it to SPI: until the I2C slave interface is disabled, bus
    // noise can re-select it.
    spi_write(REG_USER_CTRL, BIT_I2C_IF_DIS);

    // Power the sensors on explicitly. PWR_MGMT_2 disable bits are [5:3]
    // for the accelerometer axes and [2:0] for the gyro axes, and 0x3F --
    // everything off -- is a documented reset value for this part. The
    // Linux example never writes this register and works, but it runs
    // against a device Linux has already brought up; nothing here has.
    //
    // This is the leading explanation for what the previous run showed:
    // GYRO_SMPLRT_DIV, ACCEL_SMPLRT_DIV_2 and ODR_ALIGN_EN read back 0 no
    // matter how often they were written, while GYRO_CONFIG_1 and
    // ACCEL_CONFIG in the same bank did take. Sample-rate registers
    // belonging to a powered-down sensor not latching, while pure
    // configuration registers do, fits exactly.
    spi_write(REG_PWR_MGMT_2, 0x00);
    chThdSleepMilliseconds(20);
    trace_printf("AP-K3: imu: WHO_AM_I=%x OK on CS%u, bus clean at %u Hz\n",
                 who, SPI_CS_CHANNEL, SPI_SPEED_HZ);

    // GYRO_CONFIG_1 / ACCEL_CONFIG both pack DLPFCFG at bits 5:3, FS_SEL at
    // bits 2:1 and FCHOICE (filter in circuit) at bit 0. The Linux example
    // read-modify-writes these in two passes; here they are single constant
    // writes for the same reason as above -- every reserved bit in both
    // registers resets to 0, so there is nothing worth preserving, and a
    // bad read cannot corrupt the result.
    if (verbose) {
        dump_state();
    }

    set_bank(2);

    const uint8_t gyro_cfg  = (uint8_t)((CFG_DLPF << 3) | (CFG_GYRO_FS << 1) | 0x01);
    const uint8_t accel_cfg = (uint8_t)((CFG_DLPF << 3) | (CFG_ACCEL_FS << 1) | 0x01);

    bool ok = true;
    ok &= write_verified(REG_GYRO_CONFIG_1, gyro_cfg, verbose);
    ok &= write_verified(REG_ACCEL_CONFIG, accel_cfg, verbose);
    ok &= write_verified(REG_GYRO_SMPLRT_DIV, CFG_SMPLRT_DIV, verbose);
    // The accelerometer divider is 12 bits across two registers.
    ok &= write_verified(REG_ACCEL_SMPLRT_DIV_1, 0, verbose);
    ok &= write_verified(REG_ACCEL_SMPLRT_DIV_2, CFG_SMPLRT_DIV, verbose);
    // Start both sample clocks together so the samples stay in step.
    ok &= write_verified(REG_ODR_ALIGN_EN, 0x01, verbose);

    set_bank(0);

    if (!ok) {
        trace_printf("AP-K3: imu: configuration did not stick after %u attempts each, retrying\n",
                     WRITE_RETRIES);
        if (verbose) {
            dump_state();
        }
        return false;
    }
    if (xfer_failed) {
        trace_printf("AP-K3: imu: a transfer timed out during configuration, retrying\n");
        return false;
    }
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
    static uint8_t bad_samples;
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
    xfer_failed = false;
    set_bank(0);
    spi_read(REG_ACCEL_OUT, raw, sizeof(raw));

    ax = (int16_t)((uint16_t)raw[0]  << 8 | raw[1]);
    ay = (int16_t)((uint16_t)raw[2]  << 8 | raw[3]);
    az = (int16_t)((uint16_t)raw[4]  << 8 | raw[5]);
    gx = (int16_t)((uint16_t)raw[6]  << 8 | raw[7]);
    gy = (int16_t)((uint16_t)raw[8]  << 8 | raw[9]);
    gz = (int16_t)((uint16_t)raw[10] << 8 | raw[11]);
    temp_raw = (int16_t)((uint16_t)raw[12] << 8 | raw[13]);

    const int32_t gx_mdps = (int32_t)((float)gx * 1000.0f / GYRO_SENSITIVITY);
    const int32_t gy_mdps = (int32_t)((float)gy * 1000.0f / GYRO_SENSITIVITY);
    const int32_t gz_mdps = (int32_t)((float)gz * 1000.0f / GYRO_SENSITIVITY);

    const bool implausible = xfer_failed ||
        (gx_mdps > IMU_GYRO_SANITY_MDPS) || (gx_mdps < -IMU_GYRO_SANITY_MDPS) ||
        (gy_mdps > IMU_GYRO_SANITY_MDPS) || (gy_mdps < -IMU_GYRO_SANITY_MDPS) ||
        (gz_mdps > IMU_GYRO_SANITY_MDPS) || (gz_mdps < -IMU_GYRO_SANITY_MDPS);

    if (implausible) {
        bad_samples++;
        trace_printf("AP-K3: imu: implausible sample (%u/%u) g=%d,%d,%d mdps, discarding\n",
                     (uint32_t)bad_samples, (uint32_t)IMU_BAD_SAMPLES_TO_RESYNC,
                     (int)gx_mdps, (int)gy_mdps, (int)gz_mdps);
        if (bad_samples >= IMU_BAD_SAMPLES_TO_RESYNC) {
            // Same escape hatch as a failed bring-up: force the next
            // update() to re-validate the bus (bus_check, WHO_AM_I, full
            // reconfigure) instead of continuing to trust a bus that has
            // just proven itself unreliable.
            trace_printf("AP-K3: imu: %u consecutive bad samples, forcing resync\n",
                         (uint32_t)IMU_BAD_SAMPLES_TO_RESYNC);
            imu_present = false;
            bad_samples = 0;
        }
        return;
    }
    bad_samples = 0;

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
    const int32_t temp_mc = (int32_t)(((float)temp_raw / 333.87f + 21.0f) * 1000.0f);

    trace_printf("AP-K3: imu a=%d,%d,%d mg g=%d,%d,%d mdps t=%d mC\n",
                 (int)ax_mg, (int)ay_mg, (int)az_mg,
                 (int)gx_mdps, (int)gy_mdps, (int)gz_mdps,
                 (int)temp_mc);
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
