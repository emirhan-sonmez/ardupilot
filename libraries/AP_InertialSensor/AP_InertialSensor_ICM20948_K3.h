#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include <AP_HAL/SPIDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

/*
  ICM-20948 backend for the Gemstone O1 R5F, over MCU_MCSPI0 CS3.

  Deliberately NOT AP_InertialSensor_Invensensev2, which is the stock driver
  for this part. That driver reads the sensor FIFO in blocks of a dozen bytes
  or more, and multi-byte reads are exactly what does not work on this bus:
  measured 2026-07-31, a 1-byte read is stable over 256 reads in every epoch
  tested while any read of 4 or more bytes corrupts one bit position, which
  then resolves as a coin flip. The corruption is silent -- values stay
  plausible -- so a FIFO-based driver would produce an attitude estimate that
  is wrong in ways indistinguishable from bad tuning.

  So this backend samples the data registers one register at a time, which is
  the only transaction shape proven reliable on this hardware. It is a bench
  bring-up driver: ~1 ms per sample at 250 kHz means it cannot reach the
  >=1 kHz gyro rate stable flight needs. Replace it with the stock driver once
  multi-byte reads work (Q-35) or the bus runs faster.
*/
class AP_InertialSensor_ICM20948_K3 : public AP_InertialSensor_Backend
{
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation);

    bool update() override;
    void start() override;

private:
    AP_InertialSensor_ICM20948_K3(AP_InertialSensor &imu,
                                  AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                  enum Rotation rotation);

    bool init_sensor();
    bool select_bank(uint8_t bank);
    bool read_reg(uint8_t reg, uint8_t &value);
    bool write_reg_verified(uint8_t reg, uint8_t value);
    void sample();

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;
    enum Rotation _rotation;

    uint8_t _gyro_instance;
    uint8_t _accel_instance;

    // Cached bank selection. -1 forces the next select_bank() to issue the
    // write, which matters after any bus reconfiguration.
    int8_t _current_bank;

    // Set by sample() when a register read fails, so update() can report the
    // instance unhealthy rather than publishing the previous sample forever.
    bool _last_sample_ok;
};

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
