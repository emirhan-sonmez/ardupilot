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

#include "stack_paint.h"

#define STACK_PAINT_PATTERN 0xAAU

/* Linker symbols (rules_stacks.ld), addresses not values -- take &sym. */
extern uint32_t __sys_stack_base__;
extern uint32_t __sys_stack_end__;

/*
  Called once, as early as possible in run(). The stack grows down from
  __sys_stack_end__; by the time this runs, crt0 and the call chain down to
  here have already used some of it. `marker` sits inside this function's own
  frame, i.e. at or above the current SP, so everything below it down to the
  base is guaranteed never touched yet and safe to paint.

  The fill loop uses a volatile pointer rather than memset(): the linked
  libc memset() silently faults the core when called this early (before
  scheduler->init() / chSysInit()), with no trace output and no crash report
  (M9, no watchdog). A plain non-volatile byte loop reproduces the identical
  silent fault -- GCC -O2 recognizes the sequential-store idiom and quietly
  substitutes the same memset() call. volatile defeats that recognition and
  forces genuine per-byte stores. Root cause of why memset() itself faults
  this early is still open; this sidesteps it rather than explains it.
*/
void stack_paint_init(void)
{
    volatile uint32_t marker;
    uint8_t *base = (uint8_t *)&__sys_stack_base__;
    uint8_t *safe_top = (uint8_t *)&marker;

    if (safe_top > base) {
        volatile uint8_t *p = base;
        while (p < safe_top) {
            *p++ = (uint8_t)STACK_PAINT_PATTERN;
        }
    }
}

/* Bytes between the deepest disturbed byte and the stack top -- the peak
   depth ever reached since stack_paint_init(). Scans from the base (deepest
   possible) upward for the first still-intact pattern byte. */
uint32_t stack_paint_highwater(void)
{
    const uint8_t *base = (const uint8_t *)&__sys_stack_base__;
    const uint8_t *end = (const uint8_t *)&__sys_stack_end__;
    const uint8_t *p = base;

    while ((p < end) && (*p == (uint8_t)STACK_PAINT_PATTERN)) {
        p++;
    }
    return (uint32_t)(end - p);
}

uint32_t stack_paint_total(void)
{
    const uint8_t *base = (const uint8_t *)&__sys_stack_base__;
    const uint8_t *end = (const uint8_t *)&__sys_stack_end__;

    return (uint32_t)(end - base);
}
