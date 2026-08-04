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
 * @file    trace.h
 * @brief   RemoteProc trace buffer logging.
 * @details Messages are readable on the Linux host through debugfs:
 *          /sys/kernel/debug/remoteproc/remoteprocN/trace0
 */

#ifndef TRACE_H
#define TRACE_H

#include <stdarg.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
void trace_init(void);
void trace_printf(const char *fmt, ...);
/* va_list variant, for forwarding varargs already captured by a caller
   (e.g. AP_HAL::panic(const char *errormsg, ...)). Same minimal format
   subset as trace_printf: %s, %c, %d, %u, %x, %%. */
void trace_vprintf(const char *fmt, va_list ap);
/* Total bytes discarded by buffer compaction. Non-zero means the log now
   shown by debugfs is a tail, not the whole run -- report it alongside the
   log so a truncated capture is never mistaken for a complete one. */
uint32_t trace_bytes_dropped(void);
/* Number of compactions performed. Q-32 correlates the hang with compaction
   count, but that count has only ever been INFERRED from trace_bytes_dropped
   arithmetic. Reporting it directly makes the correlation observed rather
   than reconstructed, and distinguishes "died at the Nth compaction" from
   "died N seconds in". */
uint32_t trace_compaction_count(void);
#ifdef __cplusplus
}
#endif

#endif /* TRACE_H */
