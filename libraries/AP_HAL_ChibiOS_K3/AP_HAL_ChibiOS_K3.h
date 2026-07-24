#pragma once

/* Your layer exports should depend on AP_HAL.h ONLY. */
#include <AP_HAL/AP_HAL.h>

/*
  Umbrella header for the AP_HAL_ChibiOS_K3 module: the AP_HAL backend for the
  TI AM67 / J722S Cortex-R5F running ChibiOS/RT (board id HAL_BOARD_CHIBIOS_K3).

  This is a NEW, non-STM32 backend (Strategy A). It deliberately does NOT reuse
  AP_HAL_ChibiOS, whose sources are pervasively STM32-specific; instead it sits
  on the hand-written AM67 ChibiOS port. Implementation details are exposed only
  through the ChibiOS_K3 namespace.

  All declaration and compilation is guarded by CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3.
*/

#include "HAL_ChibiOS_K3_Class.h"
