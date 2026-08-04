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
 * @file    ipc_storage.c
 * @brief   R5F side of the shared-memory parameter store. See ipc_storage.h
 *          for the wire contract and for why the backing store is a Linux
 *          file rather than the onboard EEPROM.
 */

#include <string.h>
#include "ch.h"
#include "ipc_ring.h"
#include "ipc_storage.h"
#include "trace.h"

static ipc_storage_hdr_t *const hdr =
    (ipc_storage_hdr_t *)(IPC_RING_BASE + IPC_STORAGE_CTRL_OFFSET);

static uint8_t *const image =
    (uint8_t *)(IPC_RING_BASE + IPC_STORAGE_DATA_OFFSET);

static bool initialised;

void ipc_storage_init(void)
{

    if (initialised) {
        return;
    }
    initialised = true;

    /*
      Deliberately does NOT clear the image, and does not touch host_ready,
      load_seq or saved_seq -- those belong to the daemon. Zeroing here would
      destroy a perfectly good image on a firmware reload while Linux keeps
      running, which is precisely the case this transport exists to survive.

      magic is published last: the daemon uses it as the signal that the rest
      of the block is meaningful, so every field it describes must already be
      in place.
    */
    hdr->version = IPC_STORAGE_VERSION;
    hdr->size    = IPC_STORAGE_SIZE;
    hdr->magic   = IPC_STORAGE_MAGIC;

    trace_printf("AP-K3: storage ipc published at %x, %u bytes, host_ready=%u\n",
                 (uint32_t)(IPC_RING_BASE + IPC_STORAGE_CTRL_OFFSET),
                 (uint32_t)IPC_STORAGE_SIZE, hdr->host_ready);
}

bool ipc_storage_ready(void)
{

    return (hdr->magic == IPC_STORAGE_MAGIC) && (hdr->host_ready != 0U);
}

bool ipc_storage_wait_ready(uint32_t timeout_ms)
{
    const uint32_t poll_ms = 50U;
    uint32_t waited = 0U;

    if (ipc_storage_ready()) {
        return true;
    }

    while (waited < timeout_ms) {
        chThdSleepMilliseconds(poll_ms);
        waited += poll_ms;
        if (ipc_storage_ready()) {
            trace_printf("AP-K3: storage image arrived after %u ms, load_seq=%u\n",
                         waited, hdr->load_seq);
            return true;
        }
    }

    /*
      Loud on purpose. Serving zeros looks exactly like a blank EEPROM to
      ArduPilot, which will format it and write defaults -- so a silent
      timeout here is how a good parameter file gets overwritten.
    */
    trace_printf("AP-K3: storage daemon ABSENT after %u ms. Serving zeros; "
                 "parameters will NOT persist and may be overwritten.\n",
                 timeout_ms);
    return false;
}

void ipc_storage_read(uint32_t offset, uint8_t *dst, uint32_t len)
{

    if ((offset >= IPC_STORAGE_SIZE) || (len > (IPC_STORAGE_SIZE - offset))) {
        memset(dst, 0, len);
        return;
    }
    memcpy(dst, &image[offset], len);
}

void ipc_storage_write(uint32_t offset, const uint8_t *src, uint32_t len)
{

    if ((offset >= IPC_STORAGE_SIZE) || (len > (IPC_STORAGE_SIZE - offset))) {
        return;
    }

    /*
      No-op writes are common: AP_Param re-saves values that have not changed,
      and every bump of dirty_seq costs the daemon a 16 KiB file write. Compare
      first.
    */
    if (memcmp(&image[offset], src, len) == 0) {
        return;
    }

    memcpy(&image[offset], src, len);

    /*
      Ordering matters. The payload must be visible before the sequence number
      that advertises it, or the daemon can persist a half-written image. This
      window is mapped NORMAL_NONCACHE | SHARED, so a DMB is enough -- there is
      no cache to clean.
    */
    __asm__ volatile ("dmb" ::: "memory");
    hdr->dirty_seq++;
}

bool ipc_storage_synced(void)
{

    return ipc_storage_ready() && (hdr->saved_seq == hdr->dirty_seq);
}
