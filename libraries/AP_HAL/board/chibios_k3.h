#pragma once

/*
  Board feature definitions for the TI AM67 / J722S Cortex-R5F running
  ChibiOS/RT, driven by the AP_HAL_ChibiOS_K3 backend.

  This is a NEW, non-STM32 AP_HAL backend (Strategy A): it bypasses the
  STM32-only ChibiOS hwdef generator and sits on the hand-written AM67
  ChibiOS port (os/hal/ports/TI/AM67, board T3_GEMSTONE_O1_R5F).

  WORK IN PROGRESS (Phase 2 / S1): values below are provisional. The
  HAL_Semaphore/HAL_BinarySemaphore typedefs currently point at the Empty
  backend and will move to ChibiOS_K3:: once that backend lands.
*/

#define HAL_BOARD_NAME "GEMSTONE-O1-R5F"

// R5F @ ~800MHz with DDR backing store: treat as a fast, large-memory target.
#define HAL_CPU_CLASS HAL_CPU_CLASS_1000
#define HAL_MEM_CLASS HAL_MEM_CLASS_1000

/*
  Onboard barometer: ST LPS22DF on MCU_MCSPI0 CS1, read from the ID register
  on hardware 2026-08-02 (WHO_AM_I=0xb4). Named devices live in
  AP_HAL_ChibiOS_K3/SPIDevice.cpp's device_table.
*/
#ifndef HAL_GEMSTONE_BARO_NAME
#define HAL_GEMSTONE_BARO_NAME "lps22df"
#endif

#ifndef HAL_STORAGE_SIZE
#define HAL_STORAGE_SIZE            16384
#endif
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE

// No IMU wired to the HAL yet (sensor drivers arrive during driver hardening).
#define HAL_INS_DEFAULT HAL_INS_NONE

// Board directory macros (no real filesystem yet; these satisfy AP_Logger /
// AP_Terrain / storage code paths until a storage backend lands).
#ifndef HAL_BOARD_STORAGE_DIRECTORY
#define HAL_BOARD_STORAGE_DIRECTORY "APM"
#endif
#ifndef HAL_BOARD_LOG_DIRECTORY
#define HAL_BOARD_LOG_DIRECTORY "APM/logs"
#endif
#ifndef HAL_BOARD_TERRAIN_DIRECTORY
#define HAL_BOARD_TERRAIN_DIRECTORY "APM/terrain"
#endif

#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_CHIBIOS_K3_GEMSTONE_O1

// Currently linked/run from DDR; generous limit until the memory map is finalised.
#ifndef HAL_PROGRAM_SIZE_LIMIT_KB
#define HAL_PROGRAM_SIZE_LIMIT_KB 2048
#endif

#define HAL_HAVE_BOARD_VOLTAGE 0
#define HAL_HAVE_SERVO_VOLTAGE 0
#define HAL_HAVE_SAFETY_SWITCH 0

/*
  No logging backend by default.

  This board has no filesystem and no dataflash chip, so AP_Logger's default
  selection falls all the way through to Backend_Type::MAVLINK. That backend
  then gets created, _next_backend becomes non-zero, logging_present() returns
  true, and the prearm logging check runs -- and fails, permanently, with
  "PreArm: Logging failed". The vehicle cannot arm because of a log target that
  does not exist on this hardware.

  Setting LOG_BACKEND_TYPE=0 in a ground station fixes it per-board and has to
  be redone on every fresh storage image, which is a trap for whoever sets this
  board up next. Default it off here instead. Anyone who does want MAVLink
  logging can still enable it with the parameter; the backend is still compiled
  in.
*/
#define HAL_LOGGING_BACKENDS_DEFAULT 0

// Concrete Semaphore types for this board (mirrors AP_HAL/board/chibios.h).
// Guarded by __cplusplus: this board header is also reached by C translation
// units (e.g. the Lua sources via lua_common_defs.h -> AP_HAL_Boards.h), and the
// Semaphore header is C++. Without the guard those C files fail to compile.
#ifdef __cplusplus
#include <AP_HAL_ChibiOS_K3/Semaphores.h>
#define HAL_Semaphore ChibiOS_K3::Semaphore
#define HAL_BinarySemaphore ChibiOS_K3::BinarySemaphore
#endif
