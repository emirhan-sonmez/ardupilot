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
      Deliberately does NOT check for the INS backend here.

      Copter calls AP::compass().init() at system.cpp:94 and does not reach
      startup_INS_ground() until line 168, so at this point
      AP_InertialSensor_ICM20948_K3 has not been constructed and its singleton
      is null. Two earlier attempts failed on exactly that: first requiring a
      magnetometer sample, then waiting 500 ms for one. Neither could ever
      succeed, because the object that produces samples does not exist yet --
      probe() returned nullptr, no backend was registered, and read() was never
      called. The compass QGC displayed was the remembered device id from
      stored parameters with nothing behind it.

      So register unconditionally. This is an onboard part on a board with a
      fixed sensor complement, its presence is already proven at INS start via
      the SLV4 identity read, and read() publishes nothing until real samples
      appear. A magnetometer that never streams therefore shows as an
      unhealthy compass rather than a silently absent one, which is the more
      honest failure.
    */
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
    /*
      Bus id describes where the part actually is: SPI bus 0, chip select 3 --
      the ICM-20948 it lives behind -- with the AK09916 device type. It is the
      ICM's address rather than the magnetometer's because that is the device
      the host can address; the AK09916 has no identity on this bus.
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

    _last_counter = 0;
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
