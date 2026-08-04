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
 * @file    trace.c
 * @brief   RemoteProc trace buffer logging.
 * @details Append-only character buffer in a non-cacheable DDR window, the
 *          Linux remoteproc core exposes it through debugfs. Formatting is
 *          intentionally minimal: %s, %c, %d, %u, %x and %% only.
 */

#include <stdarg.h>
#include <stdint.h>

#include "board.h"
#include "trace.h"

/* Plain CPSR-based critical section, usable from any context including
   before kernel initialization, the ARMv7-R port has no recursive locks.*/
static inline uint32_t irq_save(void)
{
    uint32_t cpsr;

    __asm volatile ("mrs %0, cpsr" : "=r" (cpsr));
    __asm volatile ("cpsid i" ::: "memory");
    return cpsr;
}

static inline void irq_restore(uint32_t cpsr)
{

    if ((cpsr & 0x80U) == 0U) {
        __asm volatile ("cpsie i" ::: "memory");
    }
}

__attribute__((section(".trace"), used))
static char trace_buffer[AM67_TRACEBUF_SIZE];

/* The final 0x100 bytes hold the bring-up boot markers (see board.c), the
   log never writes into or clears that area.*/
#define TRACE_BOOTMARK_SIZE     0x100U
#define TRACE_LOG_SIZE          (AM67_TRACEBUF_SIZE - TRACE_BOOTMARK_SIZE)

static uint32_t trace_pos;
static uint32_t trace_dropped;
static uint32_t trace_compactions;

/*
  Discard the oldest half of the log so recording can continue.

  This buffer used to be strictly append-only: it filled after roughly three
  minutes of steady-state logging and everything after that was silently lost.
  That is the worst possible failure mode for diagnosing anything that goes
  wrong minutes into a run, because the log stops well before the symptom and
  gives no indication it has stopped -- a stalled firmware and a full buffer
  look identical from Linux.

  Compaction rather than a true ring buffer: the Linux remoteproc core exposes
  this as a flat character array, so a wrapping buffer would hand the reader a
  log spliced together at an arbitrary byte. Dropping the front keeps the
  surviving log in chronological order and readable with plain cat, at the cost
  of one 8 KiB byte copy per fill (~1.5 minutes apart at the current logging
  rate). Resumes at a line boundary so the log never starts mid-line.

  Runs with interrupts disabled (the caller holds irq_save), and this buffer
  lives in a NON-CACHEABLE DDR window, so every access goes to memory. Copied a
  word at a time rather than a byte at a time for that reason: byte-wise, 8 KiB
  of uncached DDR is milliseconds of interrupts-off, which is long enough to
  overflow the 64-byte iBus RX queue (~15.4ms of data) and drop receiver
  bytes -- a logging routine must not be able to cost RC frames. Do not lower
  the compaction threshold, and do not revert the word copy.
*/
static void trace_compact(void)
{
    static const char marker[] = "[trace: oldest half dropped]\n";
    uint32_t keep_from = TRACE_LOG_SIZE / 2U;
    uint32_t i;
    uint32_t j = 0U;

    while ((keep_from < trace_pos) && (trace_buffer[keep_from] != '\n')) {
        keep_from++;
    }
    if (keep_from < trace_pos) {
        keep_from++;                        /* skip the newline itself */
    }
    trace_dropped += keep_from;
    trace_compactions++;

    for (i = 0U; i < (sizeof(marker) - 1U); i++) {
        trace_buffer[j++] = marker[i];
    }

    /* Pad the marker so the destination and the source share an alignment phase.
       The bulk move below can only reach a word boundary when j and keep_from are
       congruent mod 4: its guard clears only once BOTH are word-aligned, and it
       advances them in lockstep. Start them out of phase and the guard never
       clears, so the whole 8 KiB "word-wise" move silently degrades to a
       byte-at-a-time copy of NON-CACHEABLE DDR with interrupts disabled -- the
       exact millisecond-scale interrupts-off cost this routine exists to avoid.
       Whether it happens depends on where the line boundary landed, so the
       degradation is intermittent and invisible. At most three pad bytes, and j
       (~30) can never overrun keep_from (>= half the buffer). */
    while ((j & 3U) != (keep_from & 3U)) {
        trace_buffer[j++] = ' ';
    }

    /* Word-wise bulk move of the surviving tail, with byte-wise heads/tails for
       whatever does not land on a 4-byte boundary. Source and destination cannot
       overlap destructively here: j is ~30 and keep_from is at least half the
       buffer, so the read pointer always stays ahead of the write pointer. */
    {
        uint32_t src = keep_from;
        uint32_t n = trace_pos - keep_from;

        while ((n > 0U) && (((src | j) & 3U) != 0U)) {
            trace_buffer[j++] = trace_buffer[src++];
            n--;
        }
        while (n >= 4U) {
            *(uint32_t *)(void *)&trace_buffer[j] =
                *(const uint32_t *)(const void *)&trace_buffer[src];
            j += 4U;
            src += 4U;
            n -= 4U;
        }
        while (n > 0U) {
            trace_buffer[j++] = trace_buffer[src++];
            n--;
        }
    }

    trace_pos = j;

    /* Clear the freed tail so the debugfs reader, which hands back the whole
       buffer rather than stopping at trace_pos, cannot show stale text after the
       live log. Word-wise for the same uncached-DDR reason as the copy. */
    while ((j < TRACE_LOG_SIZE) && ((j & 3U) != 0U)) {
        trace_buffer[j++] = '\0';
    }
    while ((j + 4U) <= TRACE_LOG_SIZE) {
        *(uint32_t *)(void *)&trace_buffer[j] = 0U;
        j += 4U;
    }
    while (j < TRACE_LOG_SIZE) {
        trace_buffer[j++] = '\0';
    }
}

static void trace_putc(char c)
{

    /* Last byte stays zero as terminator.*/
    if (trace_pos >= (TRACE_LOG_SIZE - 1U)) {
        trace_compact();
    }
    trace_buffer[trace_pos++] = c;
}

uint32_t trace_bytes_dropped(void)
{

    return trace_dropped;
}

uint32_t trace_compaction_count(void)
{

    return trace_compactions;
}

static void trace_puts(const char *s)
{

    while (*s != '\0') {
        trace_putc(*s++);
    }
}

static void trace_putu(uint32_t value, uint32_t base)
{
    char digits[11];
    uint32_t i = 0U;

    do {
        uint32_t d = value % base;
        digits[i++] = (d < 10U) ? (char)('0' + d) : (char)('a' + d - 10U);
        value /= base;
    } while (value != 0U);

    while (i > 0U) {
        trace_putc(digits[--i]);
    }
}

void trace_init(void)
{
    uint32_t i;

    for (i = 0U; i < TRACE_LOG_SIZE; i++) {
        trace_buffer[i] = '\0';
    }
    trace_pos = 0U;
}

void trace_printf(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    trace_vprintf(fmt, ap);
    va_end(ap);
}

void trace_vprintf(const char *fmt, va_list ap)
{
    uint32_t sts;

    sts = irq_save();

    while (*fmt != '\0') {
        if (*fmt != '%') {
            trace_putc(*fmt++);
            continue;
        }

        fmt++;
        switch (*fmt++) {
        case 's':
            trace_puts(va_arg(ap, const char *));
            break;
        case 'c':
            trace_putc((char)va_arg(ap, int));
            break;
        case 'u':
            trace_putu(va_arg(ap, uint32_t), 10U);
            break;
        case 'x':
            trace_putu(va_arg(ap, uint32_t), 16U);
            break;
        case 'd': {
            int32_t value = va_arg(ap, int32_t);
            if (value < 0) {
                trace_putc('-');
                value = -value;
            }
            trace_putu((uint32_t)value, 10U);
            break;
        }
        case '%':
            trace_putc('%');
            break;
        default:
            /* Unknown specifier, printed as-is to keep the log readable.*/
            trace_putc('%');
            trace_putc(*(fmt - 1));
            break;
        }
    }

    irq_restore(sts);
}
