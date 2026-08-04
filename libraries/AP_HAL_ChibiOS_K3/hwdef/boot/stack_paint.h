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
 * @file    stack_paint.h
 * @brief   SYS (main thread) stack high-water measurement.
 * @details Paints the unused portion of the SYS stack with a known pattern
 *          once at boot; later scans from the base upward for the deepest
 *          byte still carrying the pattern to report peak usage. Q-25 was a
 *          silent overflow of this stack with every ChibiOS stack detector
 *          off -- this is a permanent, board-scoped early warning.
 */

#ifndef STACK_PAINT_H
#define STACK_PAINT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
void stack_paint_init(void);
uint32_t stack_paint_highwater(void);
uint32_t stack_paint_total(void);
#ifdef __cplusplus
}
#endif

#endif /* STACK_PAINT_H */
