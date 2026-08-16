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
 * @file    xmcuconf.h
 * @brief   AM67 (J722S) R5F XHAL driver configuration for ArduPilot.
 * @details The R5F runs as a RemoteProc slave: clocks, power domains and pin
 *          multiplexing belong to the Linux host, so there is no clock tree
 *          configuration here. Only peripheral assignment and interrupt
 *          priorities are the firmware's to choose.
 *
 *          Peripheral selection differs from the ChibiOS demo's xmcuconf.h on
 *          purpose. This enables exactly what AP_HAL_ChibiOS_K3 consumes:
 *          both eHRPWM instances (the demo only needs one), and neither I2C
 *          nor RTI. See xhalconf.h for why those two are off.
 *
 * @addtogroup XHAL_CONF
 * @{
 */

#ifndef XMCUCONF_H
#define XMCUCONF_H

#define AM67_MCUCONF

/*
 * VIM priorities, 0 is the most urgent and 15 the least. Same-priority lines
 * cannot preempt each other, so peripherals share a level below the system
 * tick, which keeps its default of 0.
 */

/*
 * SIO driver system settings.
 *
 * UART1 is the board's only physical UART (40-pin header pins 8 TX / 10 RX).
 * Its RX side carries iBus and belongs to ChibiOS_K3::RCInput; TX has had no
 * pad since the console pin was reassigned to EHRPWM0_B (DR-016).
 */
#define AM67_SIO_USE_UART1                  TRUE
#define AM67_SIO_UART1_IRQ_PRIORITY         8U

/*
 * SPI driver system settings.
 *
 * MCU_MCSPI0 carries both onboard sensors: the ICM-20948 on channel 3 and the
 * barometer on channel 1. Linux must be unbound from 4b00000.spi first, see
 * SPIDevice.h.
 */
#define AM67_SPI_USE_MCSPI0                 TRUE
#define AM67_SPI_MCSPI0_IRQ_PRIORITY        8U

/*
 * PWM driver system settings.
 *
 * PWMD1 = EPWM0 (channels A/B -> header pins 29 and 8), PWMD2 = EPWM1
 * (channels A/B -> pins 31 and 33). Four motor outputs on two instances, see
 * RCOutput.cpp's CHAN table.
 *
 * The eCAP instances (PWMD3..PWMD5) stay disabled: they are APWM-output-only
 * in this port, RCOutput does not use them, and ECAP0's pad is not muxed by
 * the board's overlay.
 */
#define AM67_PWM_USE_EPWM0                  TRUE
#define AM67_PWM_USE_EPWM1                  TRUE

#endif /* XMCUCONF_H */

/** @} */
