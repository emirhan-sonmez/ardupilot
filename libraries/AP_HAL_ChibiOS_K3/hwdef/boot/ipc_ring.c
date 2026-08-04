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
 * @file    ipc_ring.c
 * @brief   Shared-memory MAVLink transport between the R5F and Linux.
 * @details See ipc_ring.h for the wire contract. Design notes that belong
 *          with the implementation:
 *
 *          NO CACHE MAINTENANCE ANYWHERE. The .ipc section sits inside the
 *          1 MB window MPU region 4 maps NORMAL_NONCACHE | SHARED | XN
 *          (board.c), so every access already goes to memory. If that region
 *          is ever narrowed or its attributes changed, this transport breaks
 *          silently and in a way that looks like data corruption -- the MPU
 *          setup and this file are coupled.
 *
 *          NO ATOMIC READ-MODIFY-WRITE. Each mutable header field has exactly
 *          one writer (see ipc_ring.h), so plain loads and stores plus a
 *          barrier are sufficient and correct. This is deliberate on both
 *          sides: LDREX/STREX are not architecturally guaranteed on
 *          Device-type memory, and the Linux side maps this window through
 *          /dev/mem where it may well be Device-nGnRnE.
 *
 *          ORDERING RULE, both directions: fill the payload, DMB, then
 *          publish the new index. The consumer reads the index first, DMB,
 *          then reads the payload. Getting this backwards produces a reader
 *          that sees a valid index pointing at bytes that have not landed --
 *          rare, load-dependent, and extremely unpleasant to debug.
 *
 *          Single-threaded by construction on the R5F side: everything here
 *          runs from the main loop (register_timer_process() is a no-op on
 *          this port, see Scheduler.cpp), so there is no locking against
 *          another R5F context. If a timer thread is ever introduced, this
 *          assumption must be revisited before it touches the rings.
 */

#include <stdint.h>

#include "ipc_ring.h"
#include "trace.h"

/*
  NOLOAD in the linker script, placed at IPC_RING_BASE.

  Note what that does and does not mean: remoteproc zeroes this window on
  every firmware load anyway (rproc_elf_load_segments() memsets the
  memsz-beyond-filesz tail of each segment), so the contents do NOT persist
  across a reload and the epoch below normally restarts at 1. NOLOAD is used
  because there is no initialised image worth carrying in the ELF, not to
  preserve state.
*/
__attribute__((section(".ipc"), used, aligned(4)))
static uint8_t ipc_area[IPC_RING_TOTAL_SIZE];

static ipc_ring_hdr_t *const hdr = (ipc_ring_hdr_t *)(void *)ipc_area;
static uint8_t *const tx_data = &ipc_area[IPC_RING_TX_OFFSET];
static uint8_t *const rx_data = &ipc_area[IPC_RING_RX_OFFSET];

static uint8_t ipc_ready;

/*
  The ordering barrier described in the file header.

  DMB on the target, a compiler+CPU fence off it. The host branch exists so
  the ring logic can be unit-tested natively (test/test_ipc_ring.c) -- this
  transport cannot be exercised on hardware without the board, and the wrap,
  full-ring and resync paths are exactly the kind of thing that is cheap to
  get wrong and expensive to debug over a trace buffer.
*/
static inline void ipc_dmb(void)
{

#if defined(__arm__) || defined(__aarch64__)
    __asm volatile ("dmb" ::: "memory");
#else
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
#endif
}

/*
  Guard against a consumer index that cannot be real.

  The host writes tx_tail and rx_head. A daemon that crashed mid-update, a
  stale daemon from before a firmware reload, or an uninitialised carveout can
  leave either of them arbitrary. Without this check a bogus tail makes
  "used" enormous, ipc_ring_write() then reports no space forever, and the
  link dies in a way that looks like the ring is permanently full.

  Treating an impossible value as "empty" is the safe reading: it costs at
  worst a few bytes of duplicated or dropped MAVLink, which the protocol
  already tolerates via checksums, and it self-heals as soon as the host
  publishes a sane index again.
*/
static inline uint32_t ipc_used(uint32_t head, uint32_t tail)
{
    uint32_t used = head - tail;

    if (used > IPC_RING_DATA_SIZE) {
        return 0U;
    }
    return used;
}

void ipc_ring_init(void)
{
    uint32_t previous_epoch = 0U;

    if (ipc_ready != 0U) {
        /* AP_SerialManager and GCS_MAVLINK::init() between them open a serial
           port up to four times during boot. Re-running the reset below under a
           live daemon would tear the stream, so later calls do nothing.*/
        return;
    }

    /* Read the old epoch before invalidating, so that where the header DID
       survive (a warm re-init within one firmware load) the value strictly
       increases instead of looking unchanged.

       After a remoteproc reload it will not have survived -- the window is
       zeroed, magic fails this test, and the epoch restarts at 1. That is why
       epoch is a hint for the Linux side and not its resync trigger: the
       daemon must key off an invalidated magic, or off a producer index that
       has fallen behind its own consumer index, both of which are unambiguous.*/
    if ((hdr->magic == IPC_RING_MAGIC) && (hdr->version == IPC_RING_VERSION)) {
        previous_epoch = hdr->epoch;
    }

    /* Invalidate first: a daemon polling right now must not be allowed to read
       a half-rebuilt header and believe it. */
    hdr->magic = 0U;
    ipc_dmb();

    hdr->version    = IPC_RING_VERSION;
    hdr->hdr_size   = IPC_RING_HDR_SIZE;
    hdr->epoch      = previous_epoch + 1U;
    hdr->tx_offset  = IPC_RING_TX_OFFSET;
    hdr->tx_size    = IPC_RING_DATA_SIZE;
    hdr->rx_offset  = IPC_RING_RX_OFFSET;
    hdr->rx_size    = IPC_RING_DATA_SIZE;
    hdr->tx_head    = 0U;
    hdr->tx_tail    = 0U;
    hdr->rx_head    = 0U;
    hdr->rx_tail    = 0U;
    hdr->tx_refused = 0U;
    hdr->rx_refused = 0U;
    hdr->r5f_alive  = 0U;
    hdr->host_alive = 0U;

    /* Everything above must be visible before the magic that declares it
       valid. */
    ipc_dmb();
    hdr->magic = IPC_RING_MAGIC;

    ipc_ready = 1U;

    trace_printf("AP-K3: ipc ring at %x epoch=%u size=%u/dir\n",
                 (uint32_t)IPC_RING_BASE, hdr->epoch,
                 (uint32_t)IPC_RING_DATA_SIZE);
}

uint32_t ipc_ring_write(const uint8_t *buf, uint32_t len)
{
    uint32_t head, tail, space, n, i;

    if (ipc_ready == 0U) {
        return 0U;
    }

    head  = hdr->tx_head;                 /* ours */
    tail  = hdr->tx_tail;                 /* host's */
    space = IPC_RING_DATA_SIZE - ipc_used(head, tail);

    n = (len < space) ? len : space;

    for (i = 0U; i < n; i++) {
        tx_data[(head + i) & IPC_RING_DATA_MASK] = buf[i];
    }

    /* Payload before index. See the ordering rule in the file header. */
    ipc_dmb();
    hdr->tx_head = head + n;

    if (n < len) {
        hdr->tx_refused += (len - n);
    }

    return n;
}

uint32_t ipc_ring_read(uint8_t *buf, uint32_t len)
{
    uint32_t head, tail, avail, n, i;

    if (ipc_ready == 0U) {
        return 0U;
    }

    head  = hdr->rx_head;                 /* host's */
    tail  = hdr->rx_tail;                 /* ours */
    avail = ipc_used(head, tail);

    /* Index before payload: the producer published the index last, so having
       read it we must not let the payload reads float above it. */
    ipc_dmb();

    n = (len < avail) ? len : avail;

    for (i = 0U; i < n; i++) {
        buf[i] = rx_data[(tail + i) & IPC_RING_DATA_MASK];
    }

    ipc_dmb();
    hdr->rx_tail = tail + n;

    return n;
}

uint32_t ipc_ring_tx_space(void)
{

    if (ipc_ready == 0U) {
        return 0U;
    }
    return IPC_RING_DATA_SIZE - ipc_used(hdr->tx_head, hdr->tx_tail);
}

uint32_t ipc_ring_tx_pending(void)
{

    if (ipc_ready == 0U) {
        return 0U;
    }
    return ipc_used(hdr->tx_head, hdr->tx_tail);
}

uint32_t ipc_ring_rx_available(void)
{

    if (ipc_ready == 0U) {
        return 0U;
    }
    return ipc_used(hdr->rx_head, hdr->rx_tail);
}

void ipc_ring_discard_rx(void)
{

    if (ipc_ready == 0U) {
        return;
    }
    hdr->rx_tail = hdr->rx_head;
}

void ipc_ring_tick(void)
{

    if (ipc_ready == 0U) {
        return;
    }
    hdr->r5f_alive++;
}

uint32_t ipc_ring_tx_refused(void)
{

    if (ipc_ready == 0U) {
        return 0U;
    }
    return hdr->tx_refused;
}

uint32_t ipc_ring_host_alive(void)
{

    if (ipc_ready == 0U) {
        return 0U;
    }
    return hdr->host_alive;
}
