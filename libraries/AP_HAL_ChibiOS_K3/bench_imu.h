#pragma once

namespace ChibiOS_K3 {
    // Bench ICM-20948 read-out on MCU_MCSPI0 chip select 3. Called from
    // HAL_ChibiOS_K3::run(): bench_imu_init() once before the vehicle's
    // setup(), bench_imu_update() every main-loop tick. See bench_imu.cpp
    // for scope, bus ownership and the Linux-side prerequisites.
    void bench_imu_init();
    void bench_imu_update();
}
