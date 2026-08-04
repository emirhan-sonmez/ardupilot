#pragma once

#include <stdint.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Semaphores.h>
#include "AP_HAL_ChibiOS_K3_Namespace.h"

/*
  Semaphore/BinarySemaphore for the AM67/K3 ChibiOS backend.

  Like AP_HAL_ChibiOS, the ChibiOS lock objects are stored as an opaque
  uint32_t[] and cast to the real ChibiOS types inside the .cpp, so that ch.h
  is NOT pulled into every file that includes a board header.

  NOTE (S2): the .cpp is currently a no-op stub (single-threaded bring-up only).
  S3 replaces the bodies with real ChibiOS chMtxLock/chBSem* logic once the
  make-integration puts ch.h on the include path.
*/

class ChibiOS_K3::Semaphore : public AP_HAL::Semaphore
{
public:
    Semaphore();
    bool give() override;
    bool take(uint32_t timeout_ms) override;
    bool take_nonblocking() override;
protected:
    // opaque storage for a ChibiOS mutex_t (cast in the .cpp)
    uint32_t _lock[6];
};

class ChibiOS_K3::BinarySemaphore : public AP_HAL::BinarySemaphore
{
public:
    BinarySemaphore(bool initial_state=false);

    bool wait(uint32_t timeout_us) override;
    bool wait_blocking() override;
    void signal() override;
protected:
    // opaque storage for a ChibiOS binary_semaphore_t (cast in the .cpp)
    uint32_t _bsem[6];
};
