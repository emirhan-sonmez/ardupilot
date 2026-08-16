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
 * @file    hwdef/boot/am67_mailbox.h
 * @brief   OMAP-style mailbox doorbell to the A53, for remoteproc control.
 * @details A single 32-bit word in each direction, nothing more. This is not
 *          an IPC transport: RPMsg would additionally need virtio vrings and
 *          an RSC_VDEV entry in the resource table. The one job here is to
 *          answer the kernel's shutdown request so `remoteproc stop` completes
 *          instead of timing out.
 *
 *          Register layout is the OMAP4/TYPE2 variant, which is what
 *          "ti,am64-mailbox" selects in the Linux omap-mailbox driver
 *          (of_match -> am654_data -> MBOX_INTR_CFG_TYPE2). Base address, FIFO
 *          directions, user index and IRQ number are in the board's board.h
 *          with their provenance.
 *
 *          Why this lives in ArduPilot rather than in the ChibiOS tree: it was
 *          part of the classic AM67 HAL port, and the XHAL port that replaced
 *          that has no mailbox driver -- deliberately, since a remoteproc
 *          doorbell is neither a HAL driver class nor something a ChibiOS
 *          reviewer would accept as a vendor module. It sits here beside
 *          ipc_ring.c and ipc_storage.c, the other two pieces of this board's
 *          Linux-facing plumbing.
 */

#ifndef AM67_MAILBOX_H
#define AM67_MAILBOX_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Messages exchanged with the Linux k3_r5_remoteproc driver.
 *
 * These were read out of the running kernel module by disassembly, not from a
 * header, because they exist in no upstream source: mainline's
 * ti_k3_r5_remoteproc.c has no mailbox in its stop path at all, and the OMAP
 * enum (drivers/remoteproc/omap_remoteproc.h) stops at RP_MBOX_END_MSG
 * 0xFFFFFF14.
 *
 * Do NOT substitute 0xFFFFFF11/0xFFFFFF12 here. Those are RP_MBOX_SUSPEND_SYSTEM
 * and RP_MBOX_SUSPEND_ACK; answering them leaves `stop` timing out exactly as
 * before, with nothing to indicate why.
 */
#define RP_MBOX_CRASH           0xFFFFFF02U
#define RP_MBOX_ECHO_REQUEST    0xFFFFFF03U
#define RP_MBOX_ECHO_REPLY      0xFFFFFF04U
#define RP_MBOX_SHUTDOWN        0xFFFFFF14U  /* kernel -> us, on stop        */
#define RP_MBOX_SHUTDOWN_ACK    0xFFFFFF15U  /* us -> kernel, releases its
                                                wait_for_completion_timeout  */

/**
 * @brief   Inbound message callback.
 * @note    Runs in ISR context. Must not block.
 * @return  True if a reschedule is required on ISR exit.
 */
typedef bool (*mailbox_cb_t)(uint32_t msg);

#ifdef __cplusplus
extern "C" {
#endif
  /* Arm the RX interrupt and install the inbound callback. Safe to call with
     a NULL callback, which just drains and discards. */
  void mailbox_init(mailbox_cb_t cb);

  /* Queue one word to Linux. Non-blocking: returns false if the TX FIFO is
     full rather than spinning, because callers include ISR context. */
  bool mailbox_send(uint32_t msg);
#ifdef __cplusplus
}
#endif

#endif /* AM67_MAILBOX_H */
