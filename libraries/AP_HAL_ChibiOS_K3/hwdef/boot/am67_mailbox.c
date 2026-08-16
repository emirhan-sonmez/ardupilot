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
 * @file    hwdef/boot/am67_mailbox.c
 * @brief   OMAP-style mailbox doorbell driver.
 * @details Relocated out of the ChibiOS AM67 HAL port when that port was
 *          rewritten against XHAL; see am67_mailbox.h for why it lives here
 *          now. The register logic is unchanged. The one substantive
 *          difference is where the interrupt is registered: the VIM used to be
 *          a HAL-private peripheral reached through am67_vim.h, and under XHAL
 *          it belongs to the ARMv7-R AM67 sub-port, so registration goes
 *          through port_platform.h's vimSetHandler()/vimSetPriority()/
 *          vimEnableInterrupt(). Same handler signature, same semantics.
 *
 * @addtogroup MAILBOX
 * @{
 */

#include "ch.h"
#include "hal.h"

#include "port_platform.h"

#include "am67_mailbox.h"

/* OMAP4/TYPE2 register layout, per Linux drivers/mailbox/omap-mailbox.c.
   "ti,am64-mailbox" maps to am654_data, which is MBOX_INTR_CFG_TYPE2. */
#define MAILBOX_MESSAGE(m)          (0x040U + (4U * (m)))
#define MAILBOX_FIFOSTATUS(m)       (0x080U + (4U * (m)))
#define MAILBOX_MSGSTATUS(m)        (0x0C0U + (4U * (m)))
#define MAILBOX_IRQSTATUS(u)        (0x104U + (0x10U * (u)))
#define MAILBOX_IRQENABLE(u)        (0x108U + (0x10U * (u)))
#define MAILBOX_IRQENABLE_CLR(u)    (0x10CU + (0x10U * (u)))

#define MAILBOX_IRQ_NEWMSG(m)       (1U << (2U * (m)))

/* Bounded drain. The FIFO is 4 deep on this IP, so anything beyond a handful
   means the status bit is not clearing and we would otherwise spin inside the
   ISR with interrupts masked -- the failure this port has already been bitten
   by twice (Q-25's self-deadlock, Q-32's livelock). Give up and let the level
   re-assert rather than never returning. */
#define MAILBOX_DRAIN_LIMIT         8U

static mailbox_cb_t mbox_cb;

static inline volatile uint32_t *mbox_reg(uint32_t offset) {

  return (volatile uint32_t *)(AM67_MAILBOX_BASE + offset);
}

static bool mailbox_irq_handler(void *arg) {
  bool preemption_required = false;
  uint32_t drained = 0U;

  (void)arg;

  /* Drain every queued word before clearing the status. The interrupt is
     level-sensitive on a "FIFO not empty" condition, so leaving a message
     behind and clearing the status just re-enters immediately. */
  while ((*mbox_reg(MAILBOX_MSGSTATUS(AM67_MAILBOX_RX_FIFO)) != 0U) &&
         (drained < MAILBOX_DRAIN_LIMIT)) {
    uint32_t msg = *mbox_reg(MAILBOX_MESSAGE(AM67_MAILBOX_RX_FIFO));
    drained++;

    if (mbox_cb != NULL) {
      if (mbox_cb(msg)) {
        preemption_required = true;
      }
    }
  }

  /* Write-1-to-clear, then read back. Same discipline as the tick driver: the
     write is posted, and the VIM sees a level, so returning before it lands
     re-enters the handler. */
  *mbox_reg(MAILBOX_IRQSTATUS(AM67_MAILBOX_USER)) =
      MAILBOX_IRQ_NEWMSG(AM67_MAILBOX_RX_FIFO);
  (void)*mbox_reg(MAILBOX_IRQSTATUS(AM67_MAILBOX_USER));

  return preemption_required;
}

bool mailbox_send(uint32_t msg) {

  /* Never block. This is called from the shutdown path inside an ISR, and a
     spin here with a Linux peer that has stopped reading would hang the core
     in exactly the situation the mailbox exists to recover from. */
  if (*mbox_reg(MAILBOX_FIFOSTATUS(AM67_MAILBOX_TX_FIFO)) != 0U) {
    return false;
  }

  *mbox_reg(MAILBOX_MESSAGE(AM67_MAILBOX_TX_FIFO)) = msg;
  return true;
}

void mailbox_init(mailbox_cb_t cb) {

  mbox_cb = cb;

  /* Mask our user's sources, then clear anything stale. remoteproc reloads the
     firmware without resetting the mailbox IP, so a message left over from a
     previous image is entirely possible. */
  *mbox_reg(MAILBOX_IRQENABLE_CLR(AM67_MAILBOX_USER)) = 0xFFFFFFFFU;
  *mbox_reg(MAILBOX_IRQSTATUS(AM67_MAILBOX_USER)) = 0xFFFFFFFFU;

  while (*mbox_reg(MAILBOX_MSGSTATUS(AM67_MAILBOX_RX_FIFO)) != 0U) {
    (void)*mbox_reg(MAILBOX_MESSAGE(AM67_MAILBOX_RX_FIFO));
  }

  /* Priority 0x8, the level shared by the other peripherals in this port.
     Deliberately NOT 0x0: the DMTIMER system tick keeps 0x0 precisely so that
     no peripheral ISR can delay it, after same-priority contention with UART1
     produced intermittent hangs that needed a power cycle. Mailbox traffic is a
     handful of words per firmware load and is not latency sensitive. */
  vimSetHandler(AM67_MAILBOX_IRQ, mailbox_irq_handler, NULL);
  vimSetPriority(AM67_MAILBOX_IRQ, 0x8U);
  vimEnableInterrupt(AM67_MAILBOX_IRQ);

  *mbox_reg(MAILBOX_IRQENABLE(AM67_MAILBOX_USER)) =
      MAILBOX_IRQ_NEWMSG(AM67_MAILBOX_RX_FIFO);
}

/** @} */
