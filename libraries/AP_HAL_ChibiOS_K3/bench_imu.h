#pragma once

/*
  TEMP-DIAG(Q-35): SPI read-length instability sweep.

  Lives in the header because two translation units need it: bench_imu.cpp
  compiles the sweep helpers, HAL_ChibiOS_K3_Class.cpp fires the one-shot from
  the main loop. One place to flip.

  Distinct from IMU_BUS_DIAG_ENABLED in bench_imu.cpp, which must NOT be used
  to check a fix: that one sweeps bus speeds, and an epoch at 1 MHz leaves its
  stuck bit in every later 250 kHz read, so it corrupts what it measures. This
  switch changes no speed and writes no configuration.
  REMOVE-AFTER: Q-35 closed.
*/
#ifndef IMU_BUS_DIAG_LENSWEEP
#define IMU_BUS_DIAG_LENSWEEP 1
#endif

namespace ChibiOS_K3
{
// Bench ICM-20948 read-out on MCU_MCSPI0 chip select 3. Called from
// HAL_ChibiOS_K3::run(): bench_imu_init() once before the vehicle's
// setup(), bench_imu_update() every main-loop tick. See bench_imu.cpp
// for scope, bus ownership and the Linux-side prerequisites.
void bench_imu_init();
void bench_imu_update();

// Blocks until MCU_MCSPI0 is released by Linux, or timeout_ms elapses.
// MUST be called before callbacks->setup(): AP_InertialSensor probes once
// and a probe on a contended bus disables the INS for the whole run.
void wait_for_imu_bus(uint32_t timeout_ms);

// TEMP-DIAG(Q-35): SPI read-length characterisation for the case where the
// AP_InertialSensor backend owns CS3, so bench_imu_init() is skipped.
// Must be called before callbacks->setup() -- see bench_imu.cpp.
// REMOVE-AFTER: Q-35 closed.
void bench_imu_bus_diag();
}
