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
 * @file    ipc_ring.h
 * @brief   Shared-memory MAVLink transport between the R5F and Linux.
 * @details A pair of single-producer/single-consumer byte rings in the
 *          non-cacheable DDR window at 0xA1120000, carrying the MAVLink 2
 *          stream to a Linux bridge daemon that forwards it to UDP 14550.
 *
 *          THIS FILE IS THE WIRE CONTRACT. The Linux side (gem-mavbridge)
 *          has its own copy of these constants and the header layout; the
 *          two must be changed together, and IPC_RING_VERSION must be
 *          bumped whenever the layout changes so a stale daemon refuses to
 *          attach rather than misreading the buffer.
 */

#ifndef IPC_RING_H
#define IPC_RING_H

#include <stdint.h>

/*
  Placement. The linker script (AM67_R5F.ld) puts the .ipc section at this
  address; it is inside the 1 MB window MPU region 4 already maps as
  NORMAL_NONCACHE | SHARED | XN, which is why no cache maintenance is needed
  anywhere in this transport. Do not move it above 0xA1200000.
*/
#define IPC_RING_BASE           0xA1120000U
#define IPC_RING_TOTAL_SIZE     0x10000U    /* 64 KiB, matches ddr_ipc */

#define IPC_RING_MAGIC          0x474D4156U /* 'GMAV' */
#define IPC_RING_VERSION        1U

/*
  Per-direction payload size. MUST be a power of two: the indices are
  free-running 32-bit counters and are reduced to an offset with a mask, which
  is what makes "full" and "empty" distinguishable without a spare slot.

  8 KiB is about 0.7 s of MAVLink at a typical telemetry stream rate, and
  ~128x the 64-byte TX queue this transport replaces. It exists to absorb the
  210-221 ms main-loop stalls recorded as Q-36 without losing a byte.
*/
#define IPC_RING_DATA_SIZE      8192U
#define IPC_RING_DATA_MASK      (IPC_RING_DATA_SIZE - 1U)

#define IPC_RING_HDR_SIZE       256U
#define IPC_RING_TX_OFFSET      0x1000U     /* R5F -> Linux payload */
#define IPC_RING_RX_OFFSET      0x3000U     /* Linux -> R5F payload */

/**
 * @brief   Control block at IPC_RING_BASE.
 * @details Every field is a 32-bit little-endian word, naturally aligned, and
 *          each mutable field has exactly ONE writer -- that is what makes the
 *          transport lock-free without any atomic read-modify-write. Both
 *          cores are little-endian, so no byte swapping is involved.
 *
 *          Do not add a field in the middle. Append, and bump
 *          IPC_RING_VERSION.
 */
typedef struct {
    volatile uint32_t magic;        /* R5F:  IPC_RING_MAGIC once valid       */
    volatile uint32_t version;      /* R5F:  IPC_RING_VERSION                */
    volatile uint32_t hdr_size;     /* R5F:  IPC_RING_HDR_SIZE               */
    volatile uint32_t epoch;        /* R5F:  bumped on every ipc_ring_init() */
    volatile uint32_t tx_offset;    /* R5F:  IPC_RING_TX_OFFSET              */
    volatile uint32_t tx_size;      /* R5F:  IPC_RING_DATA_SIZE              */
    volatile uint32_t rx_offset;    /* R5F:  IPC_RING_RX_OFFSET              */
    volatile uint32_t rx_size;      /* R5F:  IPC_RING_DATA_SIZE              */
    volatile uint32_t tx_head;      /* R5F:   producer index, R5F -> Linux   */
    volatile uint32_t tx_tail;      /* HOST:  consumer index, R5F -> Linux   */
    volatile uint32_t rx_head;      /* HOST:  producer index, Linux -> R5F   */
    volatile uint32_t rx_tail;      /* R5F:   consumer index, Linux -> R5F   */
    volatile uint32_t tx_refused;   /* R5F:  bytes a caller was NOT given room for */
    volatile uint32_t rx_refused;   /* HOST: bytes the host could not deliver */
    volatile uint32_t r5f_alive;    /* R5F:  liveness counter                */
    volatile uint32_t host_alive;   /* HOST: liveness counter                */
} ipc_ring_hdr_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Establish the control block and empty both rings.
 * @details Idempotent per boot: the first call initialises, later calls
 *          return immediately. That matters because AP_SerialManager and
 *          GCS_MAVLINK::init() between them call a serial port's begin()
 *          up to four times during boot, and resetting the indices under a
 *          live Linux daemon would tear the stream.
 */
void ipc_ring_init(void);

/**
 * @brief   Queue bytes for Linux.
 * @return  Number of bytes accepted; may be less than @p len, including 0.
 *          A short write is reported to the caller, not silently dropped --
 *          the AP_HAL contract requires callers to cope, and GCS_MAVLink
 *          checks txspace() first.
 */
uint32_t ipc_ring_write(const uint8_t *buf, uint32_t len);

/**
 * @brief   Take bytes sent by Linux.
 * @return  Number of bytes copied out, 0 if the ring is empty.
 */
uint32_t ipc_ring_read(uint8_t *buf, uint32_t len);

/** @brief  Bytes that ipc_ring_write() would accept right now. */
uint32_t ipc_ring_tx_space(void);

/** @brief  Bytes waiting to be read from Linux. */
uint32_t ipc_ring_rx_available(void);

/** @brief  Bytes still queued towards Linux (i.e. not yet consumed). */
uint32_t ipc_ring_tx_pending(void);

/** @brief  Discard everything Linux has sent but we have not read. */
void ipc_ring_discard_rx(void);

/**
 * @brief   Advance the R5F liveness counter.
 * @details Called from the periodic health report, not from the data path.
 *          It is the daemon's only way to tell "firmware stopped" from
 *          "firmware is running but has nothing to say", which otherwise
 *          look identical from the Linux side.
 */
void ipc_ring_tick(void);

/**
 * @brief   Running total of bytes callers were refused for lack of room.
 * @details Non-zero means the Linux daemon is not draining fast enough, or
 *          is not running at all. Reported in the periodic health line.
 */
uint32_t ipc_ring_tx_refused(void);

/**
 * @brief   The host liveness counter, as last written by the daemon.
 * @details The R5F never writes it. A value that stops changing means the
 *          bridge died while the firmware kept running -- which looks
 *          exactly like a Wi-Fi outage from QGC's side, so distinguishing
 *          them from the trace log is worth the four bytes.
 */
uint32_t ipc_ring_host_alive(void);

#ifdef __cplusplus
}
#endif

#endif /* IPC_RING_H */
