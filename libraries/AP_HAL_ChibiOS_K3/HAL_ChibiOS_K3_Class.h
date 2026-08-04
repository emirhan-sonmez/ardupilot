#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_ChibiOS_K3_Namespace.h"

/*
  The AP_HAL::HAL implementation for the Gemstone O1 Cortex-R5F (TI AM67/K3)
  under ChibiOS/RT. Lives in the global namespace, mirroring HAL_ChibiOS /
  HAL_Empty. A single instance is created in HAL_ChibiOS_K3_Class.cpp and
  returned by AP_HAL::get_HAL().
*/
class HAL_ChibiOS_K3 : public AP_HAL::HAL
{
public:
    HAL_ChibiOS_K3();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
};
