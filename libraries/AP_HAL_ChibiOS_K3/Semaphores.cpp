#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include "Semaphores.h"
#include <ch.h>

extern const AP_HAL::HAL &hal;

using namespace ChibiOS_K3;

/*
  Real ChibiOS-backed Semaphore/BinarySemaphore. The opaque _lock/_bsem storage
  (declared in Semaphores.h) is reinterpreted as the ChibiOS types here so that
  ch.h stays out of the header. Mirrors AP_HAL_ChibiOS/Semaphores.cpp.
*/

// ---- Semaphore (priority-inheritance mutex) ----

Semaphore::Semaphore()
{
    static_assert(sizeof(_lock) >= sizeof(mutex_t), "invalid mutex size");
    mutex_t *mtx = (mutex_t *)_lock;
    chMtxObjectInit(mtx);
}

bool Semaphore::give()
{
    mutex_t *mtx = (mutex_t *)_lock;
    chMtxUnlock(mtx);
    return true;
}

bool Semaphore::take(uint32_t timeout_ms)
{
    mutex_t *mtx = (mutex_t *)_lock;
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
        chMtxLock(mtx);
        return true;
    }
    if (take_nonblocking()) {
        return true;
    }
    uint64_t start = AP_HAL::micros64();
    do {
        hal.scheduler->delay_microseconds(200);
        if (take_nonblocking()) {
            return true;
        }
    } while ((AP_HAL::micros64() - start) < timeout_ms * 1000);
    return false;
}

bool Semaphore::take_nonblocking()
{
    mutex_t *mtx = (mutex_t *)_lock;
    return chMtxTryLock(mtx);
}

// ---- BinarySemaphore ----

BinarySemaphore::BinarySemaphore(bool initial_state) :
    AP_HAL::BinarySemaphore(initial_state)
{
    static_assert(sizeof(_bsem) >= sizeof(binary_semaphore_t), "invalid bsem size");
    binary_semaphore_t *sem = (binary_semaphore_t *)_bsem;
    // ChibiOS "taken" flag is the inverse of "signalled/available".
    chBSemObjectInit(sem, !initial_state);
}

bool BinarySemaphore::wait(uint32_t timeout_us)
{
    binary_semaphore_t *sem = (binary_semaphore_t *)_bsem;
    if (timeout_us == 0) {
        return chBSemWaitTimeout(sem, TIME_IMMEDIATE) == MSG_OK;
    }
    return chBSemWaitTimeout(sem, TIME_US2I(timeout_us)) == MSG_OK;
}

bool BinarySemaphore::wait_blocking()
{
    binary_semaphore_t *sem = (binary_semaphore_t *)_bsem;
    return chBSemWait(sem) == MSG_OK;
}

void BinarySemaphore::signal()
{
    binary_semaphore_t *sem = (binary_semaphore_t *)_bsem;
    chBSemSignal(sem);
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3
