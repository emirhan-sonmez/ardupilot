#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_AK09916_K3_ENABLED

#include "AP_Compass_Backend.h"
#include <AP_Math/AP_Math.h>

/*
  AK09916 magnetometer on the Gemstone O1 R5F, reached through the ICM-20948's
  auxiliary I2C master.

  Why this exists rather than AP_Compass_AK09916::probe_ICM20948_SPI():
  that path builds an AP_AK09916_BusDriver_Auxiliary on top of
  AP_InertialSensor::get_auxiliary_bus(), which is implemented by
  AP_InertialSensor_Invensensev2. This board runs its own register-read INS
  driver (AP_InertialSensor_ICM20948_K3, written because Invensensev2 reads the
  FIFO in blocks and block reads corrupt on this bus -- Q-35), so there is no
  AuxiliaryBus to hang it from. Implementing the full AuxiliaryBus/Slave pair
  to serve a single magnetometer would be more surface than the problem
  deserves.

  Instead the INS driver already streams the AK09916 into the ICM's
  EXT_SLV_SENS_DATA registers via SLV0 and publishes the result; this backend
  is a thin adapter over that.

  ORIENTATION: ROTATION_ROLL_180, which is NOT the ICM-20948's own
  ROTATION_ROLL_180_YAW_90. The magnetometer is a separate die mounted
  differently within the same package, and conflating the two is a silent
  error that shows up only as a heading that is wrong in a way no calibration
  can fix.
*/
class AP_Compass_AK09916_K3 : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(enum Rotation rotation);

    void read() override;

    static constexpr const char *name = "AK09916_K3";

private:
    AP_Compass_AK09916_K3(enum Rotation rotation);
    bool init();

    enum Rotation _rotation;
    uint32_t _last_counter;
};

#endif  // AP_COMPASS_AK09916_K3_ENABLED
