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
 * @file    ipc_storage.h
 * @brief   Persistent parameter storage shared between the R5F and Linux.
 * @details M4. Replaces Empty::Storage, which reads back zeros and discards
 *          every write, so configuration appears to save and silently does
 *          not.
 *
 *          THIS FILE IS THE WIRE CONTRACT. The Linux side (gem-storaged) has
 *          its own copy of these constants; change them together and bump
 *          IPC_STORAGE_VERSION whenever the layout moves, so a stale daemon
 *          refuses to attach rather than persisting a misread image.
 *
 * @par Why not the onboard EEPROM
 *      There is a 24c32 (4096 bytes) at 0x51, but it sits on WKUP_I2C0
 *      alongside the TPS65219 PMIC at 0x40 -- the device that owns the
 *      board's regulators. Driving that bus from the R5F would dual-master
 *      the power controller while Linux is actively using it. 4 KiB also
 *      forces ArduPilot's smallest storage layout: parameters only, no
 *      mission, fence or rally. A file on eMMC costs neither.
 *
 * @par Why not the SD card
 *      The SD/eMMC controller belongs to Linux; the R5F cannot reach it
 *      directly either. Once the transport is IPC, the backing medium is the
 *      daemon's choice, and a file on the existing eMMC beats removable
 *      media on an airframe.
 *
 * @par What this costs
 *      Parameter persistence now depends on Linux being alive. Acceptable
 *      because Linux already hosts the MAVLink bridge -- if it is down there
 *      is no telemetry either -- and because nothing in flight needs a
 *      storage WRITE. The R5F keeps flying on the image already in RAM.
 */

#ifndef IPC_STORAGE_H
#define IPC_STORAGE_H

#include <stdint.h>

/*
  Placement. Sits in the same 64 KiB non-cacheable window as the MAVLink
  rings (ipc_ring.h), immediately after them: IPC_RING_RX_OFFSET 0x3000 plus
  8 KiB of payload ends at 0x5000, and the window runs to 0x10000. No cache
  maintenance is needed anywhere here for the same reason as the rings --
  MPU region 4 maps this as NORMAL_NONCACHE | SHARED | XN.
*/
#define IPC_STORAGE_CTRL_OFFSET 0x5000U
#define IPC_STORAGE_DATA_OFFSET 0x6000U

#define IPC_STORAGE_MAGIC       0x47535452U /* 'GSTR' */
#define IPC_STORAGE_VERSION     1U

/*
  16 KiB, which is what StorageManager.h keys its full layout off
  (HAL_STORAGE_SIZE >= 16384): parameters, mission, fence and rally rather
  than parameters alone. Ends at 0x6000 + 0x4000 = 0xA000, comfortably inside
  the window.
*/
#define IPC_STORAGE_SIZE        16384U

/*
  There is deliberately no dirty-line bitmap. An earlier draft had one, with
  the R5F setting bits and the daemon clearing them -- which is two writers on
  one word and a real race, for no benefit: the daemon persists to a file, a
  16 KiB write costs about a millisecond, and parameter writes are rare and
  bursty. Rewriting the whole image on change keeps every field
  single-writer, which is the property that makes this transport lock-free.
*/

/**
 * @brief   Control block at IPC_RING_BASE + IPC_STORAGE_CTRL_OFFSET.
 * @details As with the MAVLink rings, every mutable field has exactly ONE
 *          writer, which is what removes the need for atomics on either side.
 *          Do not insert fields; append and bump IPC_STORAGE_VERSION.
 *
 *          The protocol is deliberately a pair of free-running sequence
 *          counters rather than a request/response exchange. The R5F never
 *          blocks on Linux, the daemon can restart at any moment without a
 *          handshake, and a missed wakeup costs latency instead of
 *          correctness -- the next flush sees the same dirty bits.
 */
typedef struct {
    volatile uint32_t magic;       /* R5F:  IPC_STORAGE_MAGIC once valid      */
    volatile uint32_t version;     /* R5F:  IPC_STORAGE_VERSION               */
    volatile uint32_t size;        /* R5F:  IPC_STORAGE_SIZE                  */

    /*
      Load handshake. The daemon fills the image from its file, then publishes
      load_seq. The R5F treats the image as authoritative only once
      host_ready is set AND load_seq has changed since boot; until then it
      serves zeros, exactly as Empty::Storage did, so a missing daemon
      degrades to today's behaviour instead of to garbage parameters.
    */
    volatile uint32_t host_ready;  /* HOST: 1 once the image is populated     */
    volatile uint32_t load_seq;    /* HOST: bumped after each full load       */

    /*
      Writeback. The R5F bumps dirty_seq after changing the image; the daemon
      snapshots it, writes the whole image out, then copies that snapshot into
      saved_seq. saved_seq == dirty_seq means everything the R5F has written is
      on disk -- that, not write_block() returning, is what a "parameters
      saved" indication should key off.

      Snapshot-before-write is load-bearing on the daemon side: taking it
      afterwards would let a write that arrived mid-flush be marked persisted
      when it was not.
    */
    volatile uint32_t dirty_seq;   /* R5F:  bumped after each image change    */
    volatile uint32_t saved_seq;   /* HOST: last dirty_seq durably persisted  */

    volatile uint32_t host_alive;  /* HOST: liveness counter, as the rings do */
    volatile uint32_t write_errs;  /* HOST: failed file writes, sticky        */
} ipc_storage_hdr_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Publish the control block. Idempotent per boot.
 * @details Does NOT wait for the daemon. Reads before the image is loaded
 *          return zeros rather than blocking the boot.
 */
void ipc_storage_init(void);

/**
 * @brief   True once Linux has populated the image.
 */
bool ipc_storage_ready(void);

/**
 * @brief   Bounded wait for the daemon to publish the image.
 * @details AP_HAL::Storage::init() runs before the vehicle reads any
 *          parameter, and a read that lands before the image arrives is
 *          indistinguishable from a genuinely empty EEPROM -- ArduPilot
 *          would format it and write defaults over the real values. Same
 *          class of one-shot race as the IMU probe (see wait_for_imu_bus).
 *
 * @return  True if the image arrived, false on timeout.
 */
bool ipc_storage_wait_ready(uint32_t timeout_ms);

/**
 * @brief   Copy bytes out of the shared image.
 */
void ipc_storage_read(uint32_t offset, uint8_t *dst, uint32_t len);

/**
 * @brief   Copy bytes into the shared image and mark the lines dirty.
 */
void ipc_storage_write(uint32_t offset, const uint8_t *src, uint32_t len);

/**
 * @brief   True when every write so far is durably on disk.
 */
bool ipc_storage_synced(void);

#ifdef __cplusplus
}
#endif

#endif /* IPC_STORAGE_H */
