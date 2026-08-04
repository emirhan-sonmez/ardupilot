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

/*
  TEMP-DIAG(Q-35): compile-time switch for the SPI bit-error characterisation
  below. Left on until Q-35 closes; set to 0 to get the plain bench read-out
  back without reverting code.
  REMOVE-AFTER: Q-35 closed.
*/
/*
  Off by default as of 2026-07-31. The sweep DAMAGES the part: after an epoch
  at 1 MHz or 4 MHz, subsequent 250 kHz reads come back with that epoch's
  stuck bit still set (1 MHz left bit 1 set and attempt 2 then read every
  register | 0x02, at 250 kHz, where the same reads had been clean before the
  sweep). So the diagnostic was manufacturing the configuration failures it
  was built to study, and any measurement taken after it is measuring the
  sweep. Turn it back on deliberately, on a boot that is measuring nothing
  else, and expect to reboot afterwards.
*/
#ifndef IMU_BUS_DIAG_ENABLED
#define IMU_BUS_DIAG_ENABLED 0
#endif

/* IMU_BUS_DIAG_LENSWEEP is defined in bench_imu.h -- the main loop needs it
   too. See there for why it is separate from IMU_BUS_DIAG_ENABLED. */
#define IMU_BUS_DIAG_ANY (IMU_BUS_DIAG_ENABLED || IMU_BUS_DIAG_LENSWEEP)

/*
  TEMP-DIAG(Q-35): split the 14-byte sample burst into 14 single-register
  reads. Set to 0 to get the burst back for comparison without reverting code.
  REMOVE-AFTER: bursts are reliable, or the sample path moves to a real
  AP_InertialSensor backend that can afford neither this nor the burst.
*/
#ifndef IMU_SPLIT_SAMPLE_READ
#define IMU_SPLIT_SAMPLE_READ 1
#endif

extern const AP_HAL::HAL& hal;

namespace
{

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
/*
  0.2 Hz, not the original 1 Hz. At 1 Hz this one line was ~60% of all trace
  output in steady state, which drove the buffer to compact every ~68s -- and
  each compaction is an interrupts-off bulk copy of uncached DDR (trace.c). The
  sample rate is unchanged at 50 Hz and the implausibility check still sees
  every sample; only how often a healthy reading is printed changed. Corrupted
  samples and resyncs still report immediately, so nothing diagnostic is lost.
*/
constexpr uint32_t REPORT_INTERVAL_MS = 5000;

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
/*
  There is deliberately NO inter-transaction delay in the split sample read.

  A 10us settle was tried and cost 100 ms/s: CH_CFG_ST_FREQUENCY is 1000, so
  delay_microseconds() cannot resolve below one 1 ms tick and rounds up.
  Fourteen of those per sample took the main loop from 149 Hz to 49.7 Hz and
  dtmax from 9-11 ms to 23 ms, with the loop paced entirely by the IMU read.

  No delay is needed: spiUnselect()/spiSelect() plus the address byte already
  hold the chip select high far longer than the part's minimum. If a gap ever
  is needed here, it has to be a busy-wait, not a scheduler delay.
*/
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

#if IMU_BUS_DIAG_ANY
/*
  TEMP-DIAG(Q-35): per-bit-position error characterisation of the SPI read and
  write paths.
  REMOVE-AFTER: Q-35 is closed -- the part holds a configuration and delivers
  uncorrupted samples across a 10-minute run.

  Why this exists. The 2026-07-31 trace showed steady-state samples that were
  strictly bimodal, the two clusters separated by ~256 LSB on every axis. 256
  LSB is bit 8 of the 16-bit sample, which is bit 0 of the high byte, which is
  the LAST bit clocked in that byte. Converted readings cannot distinguish:

    - "bit 0 of the high bytes" from "bit 0 of every byte" (a low-byte error
      is 1 LSB, invisible once scaled to mg/mdps);
    - a write that never landed from a write that landed and was then read
      back corrupted. Every failure in the 2026-07-30c log is recorded as
      "wrote X read Y", which is one write followed by one read and cannot
      separate the two.

  So this measures raw bytes, and it writes the pattern ONCE before reading it
  back many times. If the modal readback differs from what was written, the
  write path is at fault. If the readbacks disagree with each other, the read
  path is. Those are different bugs with different fixes.

  Memory-light by construction: rather than storing N samples, it keeps a
  population count of 1-bits per (byte offset, bit position). The majority
  value over N reads is the truth and min(count, N-count) is that bit's error
  count, in 6*8 uint16_t = 96 bytes. Main-stack high-water is 1876 of 8192.
*/
constexpr uint8_t  DIAG_PATTERN_LEN = 6;
constexpr uint16_t DIAG_READS       = 256;

/*
  Bank 2 XG_OFFS_USRH..ZG_OFFS_USRL: six contiguous R/W bytes whose only
  function is trimming gyro bias. Nothing else in the part offers six adjacent
  bytes that accept an arbitrary pattern. Harmless to scribble on with the
  aircraft on a bench, and restored to zero when the sweep finishes.
*/
constexpr uint8_t REG_XG_OFFS_USRH = 0x03;

/*
  Chosen for bit coverage, not aesthetics. 0xFF can only lose bits and 0x00 can
  only gain them, which is what separates the "bits lost, never gained"
  signature from ordinary noise; the rest mix adjacent 1s and 0s so a bit whose
  margin depends on the previous bit's level shows itself.
*/
const uint8_t diag_pattern[DIAG_PATTERN_LEN] = { 0xA5, 0x5A, 0xFF, 0x00, 0xCC, 0x33 };

/*
  Longest burst under test. 14 matches bench_imu_update()'s ACCEL_OUT..TEMP_OUT
  read exactly, which is the transaction the flight path actually issues and
  the one whose corruption shows up as the 256 LSB bimodality.
*/
constexpr uint8_t DIAG_MAX_LEN = 14;

// [offset][bit] -> how many of DIAG_READS reads saw a 1 in that position.
uint16_t diag_ones[DIAG_MAX_LEN][8];

void diag_reset_counts()
{
    for (uint8_t o = 0; o < DIAG_MAX_LEN; o++) {
        for (uint8_t b = 0; b < 8; b++) {
            diag_ones[o][b] = 0;
        }
    }
}

void diag_accumulate(const uint8_t *buf, uint8_t len)
{
    for (uint8_t o = 0; o < len; o++) {
        for (uint8_t b = 0; b < 8; b++) {
            if ((buf[o] >> b) & 1U) {
                diag_ones[o][b]++;
            }
        }
    }
}

/*
  Burst-length sweep -- the measurement that matters.

  Established 2026-07-31: 2-byte transactions (one register read) are perfect,
  0/256 across many epochs, while 7-byte bursts corrupt exactly one bit in
  exactly 128 of 256 reads. Exactly half is deterministic alternation, not
  marginal timing -- compare 4 MHz, which scatters (29, 49, 130, 512) the way
  real electrical marginality does. Two different faults; this measures the
  first one.

  What this reports is *instability*, not correctness: min(ones, reads-ones)
  per bit is nonzero only when the 256 reads disagree with each other. That
  removes any need to know the true register contents, so the sweep can run
  over registers whose reserved bits read back differently from what was
  written without polluting the result.

  The answer needed is the largest L that is stable. If L=1 works and nothing
  else does, the flight sample path has to become 14 single-register reads.
  If L holds to 4 or 8, the burst just needs splitting.
*/
void diag_length_sweep(uint8_t base_reg)
{
    static const uint8_t lengths[] = { 1, 2, 3, 4, 6, 8, 14 };
    constexpr uint8_t num_lengths = 7;
    uint8_t buf[DIAG_MAX_LEN];

    for (uint8_t li = 0; li < num_lengths; li++) {
        const uint8_t len = lengths[li];
        uint32_t total = 0;
        uint8_t worst_bit = 0;
        uint8_t worst_off = 0;
        uint32_t worst_val = 0;

        diag_reset_counts();
        for (uint16_t i = 0; i < DIAG_READS; i++) {
            spi_read(base_reg, buf, len);
            diag_accumulate(buf, len);
            hal.scheduler->delay_microseconds(SETTLE_US);
        }

        for (uint8_t o = 0; o < len; o++) {
            for (uint8_t b = 0; b < 8; b++) {
                const uint16_t ones = diag_ones[o][b];
                const uint16_t zeros = (uint16_t)(DIAG_READS - ones);
                const uint16_t minority = (ones > zeros) ? zeros : ones;
                total += minority;
                if (minority > worst_val) {
                    worst_val = minority;
                    worst_bit = b;
                    worst_off = o;
                }
            }
        }

        trace_printf("AP-K3: imu-diag  len=%u unstable=%u worst=b%u@o%u(%u/%u) %s\n",
                     (uint32_t)len, (uint32_t)total,
                     (uint32_t)worst_bit, (uint32_t)worst_off,
                     (uint32_t)worst_val, (uint32_t)DIAG_READS,
                     (total == 0) ? "STABLE" : "unstable");
    }
}

/*
  Single-byte read path, with no write anywhere in it. WHO_AM_I is read-only,
  so a wrong answer here cannot be blamed on a lost configuration write -- it
  is the cleanest read-path measurement the part offers.
*/
void diag_single_byte(uint16_t reads)
{
    uint16_t bit_errs[8] = { 0 };
    uint16_t errs = 0;

    set_bank(0);
    for (uint16_t i = 0; i < reads; i++) {
        const uint8_t got = read_reg(REG_WHO_AM_I);
        const uint8_t diff = (uint8_t)(got ^ WHO_AM_I_VAL);
        if (diff != 0) {
            errs++;
            for (uint8_t b = 0; b < 8; b++) {
                if ((diff >> b) & 1U) {
                    bit_errs[b]++;
                }
            }
        }
    }

    trace_printf("AP-K3: imu-diag  single who_am_i errs=%u/%u b7..b0=%u,%u,%u,%u,%u,%u,%u,%u\n",
                 (uint32_t)errs, (uint32_t)reads,
                 (uint32_t)bit_errs[7], (uint32_t)bit_errs[6],
                 (uint32_t)bit_errs[5], (uint32_t)bit_errs[4],
                 (uint32_t)bit_errs[3], (uint32_t)bit_errs[2],
                 (uint32_t)bit_errs[1], (uint32_t)bit_errs[0]);
}

/*
  Multi-byte burst, which is the shape the sample path actually uses and the
  shape the NuttX reference never exercised (single-word transfers only -- see
  the SPI_SPEED_HZ commentary above).
*/
void diag_burst(uint16_t reads)
{
    uint8_t buf[DIAG_PATTERN_LEN];
    uint8_t mode[DIAG_PATTERN_LEN];
    uint32_t bitpos_errs[8]  = { 0 };
    uint32_t offset_errs[DIAG_PATTERN_LEN] = { 0 };

    set_bank(2);
    for (uint8_t o = 0; o < DIAG_PATTERN_LEN; o++) {
        spi_write((uint8_t)(REG_XG_OFFS_USRH + o), diag_pattern[o]);
    }

    diag_reset_counts();
    for (uint16_t i = 0; i < reads; i++) {
        spi_read(REG_XG_OFFS_USRH, buf, DIAG_PATTERN_LEN);
        diag_accumulate(buf, DIAG_PATTERN_LEN);
        hal.scheduler->delay_microseconds(SETTLE_US);
    }

    // Majority vote per bit. A bit that is right most of the time contributes
    // its minority count as the error total; a bit that is wrong most of the
    // time flips the mode instead, and the mode-vs-written comparison below is
    // what catches that case.
    for (uint8_t o = 0; o < DIAG_PATTERN_LEN; o++) {
        uint8_t m = 0;
        for (uint8_t b = 0; b < 8; b++) {
            const uint16_t ones = diag_ones[o][b];
            const uint16_t zeros = (uint16_t)(reads - ones);
            const uint16_t minority = (ones > zeros) ? zeros : ones;
            if (ones > zeros) {
                m |= (uint8_t)(1U << b);
            }
            bitpos_errs[b] += minority;
            offset_errs[o] += minority;
        }
        mode[o] = m;
    }

    bool write_ok = true;
    for (uint8_t o = 0; o < DIAG_PATTERN_LEN; o++) {
        if (mode[o] != diag_pattern[o]) {
            write_ok = false;
        }
    }

    trace_printf("AP-K3: imu-diag  burst wrote=%x,%x,%x,%x,%x,%x mode=%x,%x,%x,%x,%x,%x writepath=%s\n",
                 (uint32_t)diag_pattern[0], (uint32_t)diag_pattern[1],
                 (uint32_t)diag_pattern[2], (uint32_t)diag_pattern[3],
                 (uint32_t)diag_pattern[4], (uint32_t)diag_pattern[5],
                 (uint32_t)mode[0], (uint32_t)mode[1], (uint32_t)mode[2],
                 (uint32_t)mode[3], (uint32_t)mode[4], (uint32_t)mode[5],
                 write_ok ? "OK" : "FAILED");

    trace_printf("AP-K3: imu-diag  burst bitpos b7..b0=%u,%u,%u,%u,%u,%u,%u,%u\n",
                 (uint32_t)bitpos_errs[7], (uint32_t)bitpos_errs[6],
                 (uint32_t)bitpos_errs[5], (uint32_t)bitpos_errs[4],
                 (uint32_t)bitpos_errs[3], (uint32_t)bitpos_errs[2],
                 (uint32_t)bitpos_errs[1], (uint32_t)bitpos_errs[0]);

    trace_printf("AP-K3: imu-diag  burst offset o0..o5=%u,%u,%u,%u,%u,%u\n",
                 (uint32_t)offset_errs[0], (uint32_t)offset_errs[1],
                 (uint32_t)offset_errs[2], (uint32_t)offset_errs[3],
                 (uint32_t)offset_errs[4], (uint32_t)offset_errs[5]);
}

/*
  Restores the six offset registers. Runs at the known-best speed, after the
  sweep, so a failure at 4 MHz cannot leave a bias trim behind.
*/
void diag_clear_offsets()
{
    set_bank(2);
    for (uint8_t o = 0; o < DIAG_PATTERN_LEN; o++) {
        spi_write((uint8_t)(REG_XG_OFFS_USRH + o), 0);
    }
}

/*
  Dumps the MCSPI registers that mcspi_init() is supposed to have just set.

  The stuck-bit mask walks one position left per controller re-init (0x01,
  0x02, ... 0x10, 0x40, 0x80), which is state surviving a reconfiguration
  rather than a wrong constant. If any of these differ between a clean epoch
  and a corrupted one, that difference is the bug. If they are all identical,
  the fault is below the register interface and the next step is the scope.

  Read directly rather than through the driver: spi_ch_getreg() is static to
  hal_spi_lld.c, and adding an accessor to the ChibiOS port for a diagnostic
  that is meant to be deleted is the wrong trade.
*/
uint32_t spi_peek(uint32_t offset)
{
    return *(volatile uint32_t *)(SPID1.base + offset);
}

void diag_dump_ctrl(const char *tag)
{
    const uint32_t ch = MCSPI_CH_OFFSET(SPI_CS_CHANNEL);

    trace_printf("AP-K3: imu-diag  regs[%s] modulctrl=%x sysconfig=%x irqstatus=%x\n",
                 tag,
                 (uint32_t)spi_peek(MCSPI_MODULCTRL_OFFSET),
                 (uint32_t)spi_peek(MCSPI_SYSCONFIG_OFFSET),
                 (uint32_t)spi_peek(MCSPI_IRQSTATUS_OFFSET));
    trace_printf("AP-K3: imu-diag  regs[%s] chconf=%x chctrl=%x chstat=%x hlsys=%x\n",
                 tag,
                 (uint32_t)spi_peek(MCSPI_CHCONF0_OFFSET + ch),
                 (uint32_t)spi_peek(MCSPI_CHCTRL0_OFFSET + ch),
                 (uint32_t)spi_peek(MCSPI_CHSTAT0_OFFSET + ch),
                 (uint32_t)spi_peek(MCSPI_HL_SYSCONFIG_OFFSET));

    // Channels share one bus. A CS left asserted by another channel puts a
    // second slave on MISO, so every channel's CHCONF matters, not just ours.
    trace_printf("AP-K3: imu-diag  regs[%s] chconf0..3=%x,%x,%x,%x\n",
                 tag,
                 (uint32_t)spi_peek(MCSPI_CHCONF0_OFFSET + MCSPI_CH_OFFSET(0)),
                 (uint32_t)spi_peek(MCSPI_CHCONF0_OFFSET + MCSPI_CH_OFFSET(1)),
                 (uint32_t)spi_peek(MCSPI_CHCONF0_OFFSET + MCSPI_CH_OFFSET(2)),
                 (uint32_t)spi_peek(MCSPI_CHCONF0_OFFSET + MCSPI_CH_OFFSET(3)));
}

/*
  Reads a fixed set of registers and reports the OR of (read ^ expected) for
  the ones whose value is known and constant. Isolates the stuck mask for an
  epoch in one line, instead of inferring it from six "wrote X read Y" lines.

  WHO_AM_I is the only register whose correct value is known unconditionally.
  The rest are compared against what a read returned moments earlier, so a
  disagreement means the read path is unstable within the epoch, whatever the
  true register contents are.
*/
void diag_stuck_mask()
{
    static const uint8_t probe_regs[] = { 0x00, 0x03, 0x05, 0x06, 0x07 };
    constexpr uint8_t num_probe = 5;
    uint8_t first[num_probe];
    uint8_t diff_or = 0;
    uint8_t who_diff = 0;

    set_bank(0);
    for (uint8_t i = 0; i < num_probe; i++) {
        first[i] = read_reg(probe_regs[i]);
    }
    who_diff = (uint8_t)(first[0] ^ WHO_AM_I_VAL);

    for (uint8_t pass = 0; pass < 8; pass++) {
        for (uint8_t i = 0; i < num_probe; i++) {
            diff_or |= (uint8_t)(read_reg(probe_regs[i]) ^ first[i]);
        }
    }

    trace_printf("AP-K3: imu-diag  probe who_am_i=%x whodiff=%x reread_diff=%x r03=%x r05=%x r06=%x r07=%x\n",
                 (uint32_t)first[0], (uint32_t)who_diff, (uint32_t)diff_or,
                 (uint32_t)first[1], (uint32_t)first[2],
                 (uint32_t)first[3], (uint32_t)first[4]);
}

#if IMU_BUS_DIAG_ENABLED
void diag_at_speed(uint32_t speed_hz)
{
    if (spi_started) {
        spiStop(&SPID1);
        spi_started = false;
    }
    spicfg.speed = speed_hz;
    spiStart(&SPID1, &spicfg);
    spi_started = true;

    // The cached bank number describes the part, not the bus, but a speed
    // change re-runs mcspi_init() and any in-flight state is gone with it.
    // Forcing a re-select costs one transaction and removes the question.
    current_bank = -1;
    xfer_failed  = false;

    trace_printf("AP-K3: imu-diag speed=%u n=%u\n",
                 (uint32_t)speed_hz, (uint32_t)DIAG_READS);
    diag_dump_ctrl("post-init");
    diag_stuck_mask();
    diag_single_byte(DIAG_READS);
    diag_burst(DIAG_READS);
    // Pattern is now in bank 2 0x03..0x08, so a sweep based at 0x00 covers
    // both plain config registers and six bit-rich ones.
    diag_length_sweep(0x00);
    diag_dump_ctrl("post-burst");
    if (xfer_failed) {
        trace_printf("AP-K3: imu-diag  NOTE a transfer timed out at this speed\n");
    }
}

/*
  Sweeps the three speeds DR-013 argued about, with per-bit resolution instead
  of the pass/fail count bus_check() gives. 4 MHz is expected to be bad (0/32
  historically) and is included precisely for that: a known-bad point
  calibrates what the counters look like when the bus really is failing.
*/
void bus_diag_sweep()
{
    /*
      4 MHz is deliberately NOT in this list any more. Its failures are a
      different fault -- writes lose bits and the error counts scatter -- and
      running it POISONS the part for every later epoch: after the 2026-07-31
      sweep, single-byte reads that had been 0/256 came back |0x08 and the
      AP_HAL selftest read WHO_AM_I=1a. Characterise it separately, on a boot
      that is not also measuring something else.
    */
    static const uint32_t speeds[] = { 250000, 1000000 };
    constexpr uint8_t num_speeds = 2;

    // Reference epoch: the controller as bring-up left it, before this sweep
    // touches anything. On the 2026-07-31 boot this was the one epoch whose
    // register reads were sane, so it is the "known good" side of the diff.
    diag_dump_ctrl("epoch0");
    diag_stuck_mask();

    // Trigger isolation, before the speed sweep perturbs anything. Four
    // rounds in the epoch bring-up left behind, one re-init at the same
    // speed, four more rounds. Same transactions throughout; the only
    // difference is the mcspi_init() in the middle.
    for (uint8_t i = 0; i < num_speeds; i++) {
        diag_at_speed(speeds[i]);
    }

    // Back to the operating speed, then undo the scribble.
    diag_at_speed(SPI_SPEED_HZ);
    diag_clear_offsets();

    // The sweep left the part configured by nothing in particular. Force the
    // normal bring-up path to run again rather than sampling through whatever
    // state 4 MHz happened to leave behind.
    current_bank = -1;
}
#endif  // IMU_BUS_DIAG_ENABLED

/*
  TEMP-DIAG(Q-35): read-only A/B for the RX-drain fix.

  Deliberately does not call diag_at_speed(): no spiStop/spiStart, no speed
  change, no register writes. It reads the part as bring-up left it, so the
  numbers are comparable across boots and the measurement cannot manufacture
  the fault it is looking for.

  Reading: len 1-2 have always been STABLE. The question is len 4, 6, 8 and 14.
  A count near 128/256 is the alternating stale-RX shift; 0 means the drain
  fixed it; a scattered count means something electrical is left underneath.
  REMOVE-AFTER: Q-35 closed.
*/
void bus_diag_lensweep()
{
    trace_printf("AP-K3: imu-diag lensweep-only, speed as brought up, n=%u\n",
                 (uint32_t)DIAG_READS);
    diag_dump_ctrl("lensweep");
    diag_stuck_mask();
    diag_single_byte(DIAG_READS);
    diag_length_sweep(0x00);
}
#endif  // IMU_BUS_DIAG_ANY

}  // namespace

void ChibiOS_K3::bench_imu_init()
{
    imu_present = imu_try_bringup();

#if IMU_BUS_DIAG_ENABLED
    // TEMP-DIAG(Q-35): runs once, after bring-up so the part is awake and out
    // of sleep. Costs ~2 s of boot and ~15 trace lines.
    // REMOVE-AFTER: Q-35 closed.
    if (imu_present) {
        bus_diag_sweep();
        imu_present = imu_try_bringup();
    } else {
        trace_printf("AP-K3: imu-diag skipped, bring-up failed\n");
    }
#elif IMU_BUS_DIAG_LENSWEEP
    /* TEMP-DIAG(Q-35): read-only, ~2 s, no re-bring-up needed afterwards
       because nothing here changes the bus or the part.
       REMOVE-AFTER: Q-35 closed. */
    if (imu_present) {
        bus_diag_lensweep();
    } else {
        trace_printf("AP-K3: imu-diag skipped, bring-up failed\n");
    }
#endif
}

/*
  Blocks until MCU_MCSPI0 is actually ours, or the timeout expires.

  remoteproc starts this core during the kernel's own boot, long before Linux
  userspace runs gemstone-r5f-setup.service and unbinds omap2_mcspi from
  4b00000.spi. Until that unbind lands, two masters drive the bus and every
  read answers 0x00.

  RCOutput::retry_pending() (DR-006) survives the same race by retrying
  forever, because a PWM channel can be enabled at any later time.
  AP_InertialSensor cannot: detect_backends() runs once inside setup(), and a
  probe that reads 0x00 sets backend_count=0 permanently -- the vehicle then
  falls back to software timing for the rest of the run with no way back. So
  the bus has to be ours BEFORE setup(), not merely eventually.

  Measured 2026-08-02: on a cold power cycle the probe at t=30s succeeds while
  the one during setup() fails, which is the entire difference between an INS
  backend and no INS backend.

  Bounded on purpose. If Linux never releases the bus, booting late with no
  IMU beats not booting at all, and the timeout is traced loudly rather than
  passed over. WHO_AM_I only -- no reset, no configuration, nothing written --
  both because the part belongs to the backend and because writing to a bus
  someone else is driving is how this port lost a session already.
*/
void ChibiOS_K3::wait_for_imu_bus(uint32_t timeout_ms)
{
    constexpr uint32_t POLL_MS = 250;
    const uint32_t start = AP_HAL::millis();
    uint32_t polls = 0;
    uint8_t who = 0;

    am67_spi0_imu_enable();

    while ((AP_HAL::millis() - start) < timeout_ms) {
        polls++;
        spiStart(&SPID1, &spicfg);
        spi_started = true;

        if (SPID1.ready) {
            xfer_failed = false;
            /* WHO_AM_I is bank-independent, so this needs no bank select and
               therefore writes nothing. */
            who = read_reg(REG_WHO_AM_I);
            if (!xfer_failed && (who == WHO_AM_I_VAL)) {
                trace_printf("AP-K3: imu bus released after %u ms (%u polls), WHO_AM_I=%x\n",
                             (uint32_t)(AP_HAL::millis() - start),
                             polls, (uint32_t)who);
                return;
            }
        }
        chThdSleepMilliseconds(POLL_MS);
    }

    trace_printf("AP-K3: imu bus STILL NOT OURS after %u ms (%u polls, last WHO_AM_I=%x). "
                 "Is 4b00000.spi unbound? Booting without an INS backend.\n",
                 timeout_ms, polls, (uint32_t)who);
}

/*
  TEMP-DIAG(Q-35): the length sweep when the real AP_InertialSensor backend
  owns CS3, which is the only configuration that now ships.

  Safe here and nowhere else. The caller runs this before callbacks->setup(),
  so the backend exists but its periodic callback has not been registered yet
  and nothing else is on the bus -- the interleaving that bench_imu_init()
  refuses to risk cannot happen at this point in the boot.

  Deliberately leaves imu_present false. That keeps bench_imu_update() a no-op
  for the rest of the run, so this never becomes a second master competing
  with the backend once the vehicle is looping.
  REMOVE-AFTER: Q-35 closed.
*/
void ChibiOS_K3::bench_imu_bus_diag()
{
#if IMU_BUS_DIAG_LENSWEEP
    if (imu_try_bringup()) {
        bus_diag_lensweep();
    } else {
        trace_printf("AP-K3: imu-diag skipped, bring-up failed\n");
    }
#endif
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
#if IMU_SPLIT_SAMPLE_READ
    /*
      TEMP-DIAG(Q-35): read the sample as 14 single-register transactions
      instead of one 14-byte burst.

      Measured 2026-07-31 at 250 kHz, 256 reads per length, over three boots:
      a 1-byte read is stable every single time (0/256, plus 32/32 bus checks),
      while 4 bytes and above always corrupt one bit position, which resolves
      as a coin flip. 14 bytes is the worst case in that table and it is
      exactly what this function used to issue -- which is why steady-state
      samples came back bimodal, the two clusters separated by 256 LSB on
      every axis. 256 LSB is bit 8 of the sample, i.e. bit 0 of the high byte.

      Cost is ~1.1 ms per sample against a 20 ms interval, so ~5% of the core
      at the 50 Hz bench rate. That is affordable here and NOT affordable at
      the >=1 kHz gyro rate flight needs, so this is a way to get trustworthy
      data now, not the final design. The burst has to be made to work, or the
      bus speed raised, before this can carry an AP_InertialSensor backend.
    */
    for (uint8_t i = 0; i < sizeof(raw); i++) {
        spi_read((uint8_t)(REG_ACCEL_OUT + i), &raw[i], 1);
    }
#else
    spi_read(REG_ACCEL_OUT, raw, sizeof(raw));
#endif

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
