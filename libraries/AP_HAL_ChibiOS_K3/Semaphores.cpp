#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include "Semaphores.h"

using namespace ChibiOS_K3;

/*
  S2 TEMP no-op stub: no real locking. This is enough to compile, link and boot
  single-threaded during bring-up. It is NOT thread-safe.

  S3 replaces these bodies with real ChibiOS logic (chMtxObjectInit/chMtxLock/
  chMtxUnlock for Semaphore; chBSemObjectInit/chBSemWaitTimeout/chBSemSignal for
  BinarySemaphore), casting _lock/_bsem to mutex_t*/binary_semaphore_t*. That
  step requires ch.h, which the chibios_k3 make-integration puts on the include
  path.
*/

Semaphore::Semaphore()
{
    _lock[0] = 0;
}

bool Semaphore::give()
{
    return true;
}

bool Semaphore::take(uint32_t timeout_ms)
{
    (void)timeout_ms;
    return true;
}

bool Semaphore::take_nonblocking()
{
    return true;
}

BinarySemaphore::BinarySemaphore(bool initial_state) :
    AP_HAL::BinarySemaphore(initial_state)
{
    _bsem[0] = initial_state ? 1 : 0;
}

bool BinarySemaphore::wait(uint32_t timeout_us)
{
    (void)timeout_us;
    return true;
}

bool BinarySemaphore::wait_blocking()
{
    return true;
}

void BinarySemaphore::signal()
{
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
