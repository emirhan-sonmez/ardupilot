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
 * @file    mcuconf.h
 * @brief   AM67 (J722S) R5F driver configuration.
 */

#ifndef MCUCONF_H
#define MCUCONF_H

#define AM67_MCUCONF

/*
 * SERIAL driver system settings.
 */
#define AM67_SERIAL_USE_UART1               TRUE
#define AM67_SERIAL_UART1_IRQ_PRIORITY      0x8U

/*
 * SPI driver system settings.
 */
#define AM67_SPI_USE_MCSPI0                 TRUE
#define AM67_SPI_MCSPI0_IRQ_PRIORITY        0x8U

/*
 * I2C driver system settings.
 */
#define AM67_I2C_USE_MCU_I2C0               TRUE
#define AM67_I2C_MCU_I2C0_IRQ_PRIORITY      0x8U

#endif /* MCUCONF_H */
