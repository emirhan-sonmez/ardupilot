#include "AP_Compass_AK09916_K3.h"

#if AP_COMPASS_AK09916_K3_ENABLED

#include <AP_InertialSensor/AP_InertialSensor_ICM20948_K3.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

/*
  Sensitivity, matching AP_Compass_AK09916 exactly so this reports the same
  units as the stock driver: 0.15 uT per LSB, then 10 milligauss per uT.
*/
static const float AK09916_ADC_RESOLUTION   = 0.15f;
static const float AK09916_MILLIGAUSS_SCALE = 10.0f;

AP_Compass_AK09916_K3::AP_Compass_AK09916_K3(enum Rotation rotation)
    : _rotation(rotation)
    , _last_counter(0)
{
}

AP_Compass_Backend *AP_Compass_AK09916_K3::probe(enum Rotation rotation)
{
    /*
      The INS driver owns the auxiliary bus and does the streaming, so it must
      already have started. It is constructed during detect_backends(), which
      runs before AP_Compass::init(), but check rather than assume: on a cold
      boot this board has previously failed to attach the INS at all when Linux
      still held the SPI controller, and a null here would be a boot-time crash
      rather than a missing compass.
    */
    if (AP_InertialSensor_ICM20948_K3::get_singleton() == nullptr) {
        return nullptr;
    }

    AP_Compass_AK09916_K3 *sensor = NEW_NOTHROW AP_Compass_AK09916_K3(rotation);
    if (sensor == nullptr) {
        return nullptr;
    }
    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_Compass_AK09916_K3::init()
{
    auto *ins = AP_InertialSensor_ICM20948_K3::get_singleton();
    Vector3f field;
    uint32_t counter = 0;

    /*
      Require a sample before registering. get_mag_field() returns false until
      one has genuinely arrived, so this refuses to register a compass that
      would only ever publish zeros -- the failure mode that made the
      barometer look healthy while it was dead.
    */
    if (!ins->get_mag_field(field, counter)) {
        return false;
    }

    /*
      Bus id describes where the part actually is: SPI bus 0, chip select 3 --
      the ICM-20948 it lives behind -- with the AK09916 device type. It is the
      ICM's address rather than the magnetometer's because that is the device
      the host can actually address; the AK09916 has no identity on this bus.
    */
    const int32_t dev_id = (int32_t)AP_HAL::Device::make_bus_id(
        AP_HAL::Device::BUS_TYPE_SPI, 0, 3, DEVTYPE_AK09916);

    if (!register_compass(dev_id)) {
        return false;
    }
    set_dev_id((uint32_t)dev_id);
    set_rotation(_rotation);

    // Onboard, inside the IMU package: never treat it as an external compass.
    set_external(false);

    _last_counter = counter;
    return true;
}

void AP_Compass_AK09916_K3::read()
{
    auto *ins = AP_InertialSensor_ICM20948_K3::get_singleton();
    Vector3f raw;
    uint32_t counter = 0;

    if (ins == nullptr || !ins->get_mag_field(raw, counter)) {
        return;
    }
    if (counter == _last_counter) {
        // Nothing new since the last call; publishing again would inflate the
        // sample count and make a stalled magnetometer look alive.
        return;
    }
    _last_counter = counter;

    raw *= AK09916_ADC_RESOLUTION;
    raw *= AK09916_MILLIGAUSS_SCALE;

    accumulate_sample(raw);
    drain_accumulated_samples();
}

#endif  // AP_COMPASS_AK09916_K3_ENABLED
