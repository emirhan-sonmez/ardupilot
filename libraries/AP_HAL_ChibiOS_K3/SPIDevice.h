#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>
#include "AP_HAL_ChibiOS_K3_Namespace.h"
#include "Semaphores.h"

/*
  AP_HAL SPI layer for the AM67/J722S K3 backend, over MCU_MCSPI0 (SPID1).

  This is the bridge that lets ArduPilot's own sensor drivers reach the
  onboard parts. Until this existed, hal.spi was Empty::SPIDeviceManager, so
  AP_InertialSensor registered zero backends and bench_imu.cpp had to talk to
  SPID1 directly -- its samples reached the trace buffer and nothing else.

  Bus ownership, mandatory before any of this works: MCU_MCSPI0 is the same
  controller Linux exposes as 4b00000.spi / spidev0.*, and the onboard sensors
  hang off it. Linux must be told to let go first, exactly like the PWM
  peripherals:

      echo 4b00000.spi | sudo tee /sys/bus/platform/drivers/omap2_mcspi/unbind

  Without that both masters drive the bus and reads return garbage rather than
  an error.

  One physical bus, one shared semaphore. Chip select is the McSPI channel
  number (channel n drives the SPI0_CSn pad), so switching device means
  re-running spiStart() with that device's channel/mode/speed -- done inside
  the bus lock, which every AP caller already holds via
  Device::get_semaphore().
*/

struct ChibiOS_K3::SPIDeviceDesc {
    const char *name;
    uint8_t  cs_channel;      // McSPI channel = SPI0_CSn pad
    uint8_t  mode;            // standard CPOL/CPHA encoding, 0..3
    uint32_t speed_low_hz;
    uint32_t speed_high_hz;
};

class ChibiOS_K3::SPIBus
{
public:
    ChibiOS_K3::Semaphore semaphore;

    // Applies desc/speed to the controller if it differs from what is already
    // programmed. Caller must hold `semaphore`.
    bool apply_config(const ChibiOS_K3::SPIDeviceDesc &desc, uint32_t speed_hz);

    // Half-duplex: send_len bytes out, then recv_len bytes in, under a single
    // chip-select assertion. Either length may be zero.
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len);
    bool transfer_fullduplex(const uint8_t *send, uint8_t *recv, uint32_t len);

    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb cb);
    bool adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h,
                                  uint32_t period_usec);

private:
    void thread_loop();
    bool start_thread();

    struct Callback {
        AP_HAL::Device::PeriodicCb cb;
        uint32_t period_usec;
        uint32_t next_usec;
        bool     active;
    };

    /*
      Four is enough for every device this board can populate (IMU, baro, and
      two spare) and keeps the table a fixed cost. Registration happens during
      driver start-up, never in steady state.
    */
    static const uint8_t MAX_CALLBACKS = 4;

    Callback _cb[MAX_CALLBACKS] = {};
    uint8_t  _num_cb = 0;
    bool     _thread_started = false;

    // Cached controller state, so back-to-back transfers on the same device
    // do not re-run spiStart(). 0xFF/0 mean "nothing applied yet".
    uint8_t  _cur_cs = 0xFF;
    uint8_t  _cur_mode = 0xFF;
    uint32_t _cur_speed = 0;
    bool     _spi_started = false;
};

class ChibiOS_K3::SPIDevice : public AP_HAL::SPIDevice
{
public:
    SPIDevice(ChibiOS_K3::SPIBus &bus, const ChibiOS_K3::SPIDeviceDesc &desc);

    bool set_speed(AP_HAL::Device::Speed speed) override;
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;
    bool transfer_fullduplex(const uint8_t *send, uint8_t *recv,
                             uint32_t len) override;
    AP_HAL::Semaphore *get_semaphore() override;
    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb cb) override;
    bool adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h,
                                  uint32_t period_usec) override;

private:
    ChibiOS_K3::SPIBus &_bus;
    const ChibiOS_K3::SPIDeviceDesc &_desc;
    uint32_t _speed_hz;
};

class ChibiOS_K3::SPIDeviceManager : public AP_HAL::SPIDeviceManager
{
public:
    AP_HAL::SPIDevice *get_device_ptr(const char *name) override;
    uint8_t get_count() override;
    const char *get_device_name(uint8_t idx) override;

    /*
      Reads WHO_AM_I from the IMU through the full AP_HAL path and traces the
      result. Proves this layer end to end -- chip select, mode, speed,
      Device::read_registers() semantics -- independently of bench_imu.cpp's
      direct SPID1 access, before any AP_InertialSensor backend depends on it.
      Safe to call when the part is absent: reports and returns.
    */
    void selftest();

    // Q-05: report which barometer is actually fitted on CS1. The device tree
    // says BMP390 and the board spec says LPS22DFTR; neither is a probe.
    void baro_ident();

    // TEMP-DIAG(Q-35): SPI read-length instability sweep over the AP_HAL path.
    // Safe to call mid-run -- takes the bus semaphore per transaction.
    // REMOVE-AFTER: Q-35 closed.
    void bus_length_diag();
};
