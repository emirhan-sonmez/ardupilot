/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    xhalconf.h
 * @brief   XHAL configuration header for the Gemstone O1 R5F board.
 * @details Replaces the classic halconf.h. Only the driver classes
 *          AP_HAL_ChibiOS_K3 actually consumes are enabled; platform.mk's
 *          smart build reads this file to decide which LLDs to compile, so a
 *          FALSE here removes the driver from the image rather than merely
 *          leaving it unused.
 *
 * @addtogroup XHAL_CONF
 * @{
 */

#ifndef XHALCONF_H
#define XHALCONF_H

#define __CHIBIOS_XHAL_CONF__
#define __CHIBIOS_XHAL_CONF_VER_1_0__

#include "xmcuconf.h"

/*===========================================================================*/
/* HAL general settings.                                                     */
/*===========================================================================*/

#define HAL_USE_MUTUAL_EXCLUSION            TRUE
#define HAL_USE_REGISTRY                    TRUE

/*===========================================================================*/
/* HAL enabled drivers.                                                      */
/*===========================================================================*/

/*
 * I2C is FALSE, unlike the classic halconf.h it replaces. AP_HAL_ChibiOS_K3
 * has no I2CDevice backend at all -- hal.i2c is Empty::I2CDeviceManager, and
 * the AK09916 magnetometer is reached through the ICM-20948's auxiliary I2C
 * master over SPI, not through the SoC's I2C controller. The classic build
 * compiled MCU_I2C0 for nothing.
 *
 * WDG is FALSE for a harder reason: the MCU RTI counter's clock source is not
 * running (MCU_RTI0 is device-tree "reserved", so nothing issues its TI-SCI
 * clock-enable), and the driver correctly refuses to arm against a 0 Hz
 * measurement. Enabling it would ship a watchdog that cannot bite. Turn this
 * on in the same change that closes the RTICLK gap, not before.
 */
#define HAL_USE_PAL                         FALSE
#define HAL_USE_ADC                         FALSE
#define HAL_USE_DAC                         FALSE
#define HAL_USE_CAN                         FALSE
#define HAL_USE_EFL                         FALSE
#define HAL_USE_ETH                         FALSE
#define HAL_USE_GPT                         FALSE
#define HAL_USE_I2C                         FALSE
#define HAL_USE_I2S                         FALSE
#define HAL_USE_ICU                         FALSE
#define HAL_USE_MMC_SPI                     FALSE
#define HAL_USE_PWM                         TRUE
#define HAL_USE_RTC                         FALSE
#define HAL_USE_SDC                         FALSE
#define HAL_USE_SIO                         TRUE
#define HAL_USE_SPI                         TRUE
#define HAL_USE_TRNG                        FALSE
#define HAL_USE_USB                         FALSE
#define HAL_USE_WDG                         FALSE
#define HAL_USE_WSPI                        FALSE

/*===========================================================================*/
/* PWM driver settings.                                                      */
/*===========================================================================*/

#define PWM_USE_CONFIGURATIONS              FALSE

/*===========================================================================*/
/* SIO driver settings.                                                      */
/*===========================================================================*/

/*
 * Buffering is required, not optional, on this board. ChibiOS_K3::RCInput
 * drains iBus from a software queue sized to ride out a long main-loop
 * iteration; the UART's own hardware FIFO is 64 bytes, about 15 ms of iBus at
 * 115200, and losing bytes corrupts framing in a way AP_RCProtocol reports as
 * stale-but-valid channels. See RCInput.h on the queue size.
 *
 * STREAMS_INTERFACE follows SYNCHRONIZATION and gives hal_buffered_sio_c its
 * asynchronous_channel_i, which is what chnReadTimeout() needs.
 */
#define SIO_DEFAULT_BITRATE                 115200
#define SIO_USE_SYNCHRONIZATION             TRUE
#define SIO_USE_STREAMS_INTERFACE           SIO_USE_SYNCHRONIZATION
#define SIO_USE_BUFFERING                   TRUE
#define SIO_USE_CONFIGURATIONS              FALSE

/*===========================================================================*/
/* SPI driver settings.                                                      */
/*===========================================================================*/

/*
 * ASSERT_ON_ERROR is FALSE where the classic halconf.h had it TRUE. On a
 * flight controller a failed transfer must be reported to the caller, not
 * turned into a kernel panic in mid-air; SPIDevice.cpp already treats a
 * timed-out exchange as a failed transfer and says so through the trace
 * buffer. Every transfer on this port is polled anyway, so the asserting path
 * is not even reached.
 */
#define SPI_USE_SYNCHRONIZATION             TRUE
#define SPI_USE_ASSERT_ON_ERROR             FALSE
#define SPI_USE_CONFIGURATIONS              FALSE

#endif /* XHALCONF_H */

/** @} */
