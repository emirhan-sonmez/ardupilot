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

#ifndef HAL_STORAGE_SIZE
#define HAL_STORAGE_SIZE            16384
#endif
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE

// No IMU wired to the HAL yet (sensor drivers arrive during driver hardening).
#define HAL_INS_DEFAULT HAL_INS_NONE

#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_CHIBIOS_K3_GEMSTONE_O1

// Currently linked/run from DDR; generous limit until the memory map is finalised.
#ifndef HAL_PROGRAM_SIZE_LIMIT_KB
#define HAL_PROGRAM_SIZE_LIMIT_KB 2048
#endif

#define HAL_HAVE_BOARD_VOLTAGE 0
#define HAL_HAVE_SERVO_VOLTAGE 0
#define HAL_HAVE_SAFETY_SWITCH 0

// TODO(S3): switch to ChibiOS_K3::Semaphore / BinarySemaphore once the real
// (ChibiOS-mutex-backed) implementations exist in AP_HAL_ChibiOS_K3.
#define HAL_Semaphore Empty::Semaphore
#define HAL_BinarySemaphore Empty::BinarySemaphore
