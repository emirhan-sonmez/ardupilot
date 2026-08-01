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

/*
 * T3 Gemstone O1 (TI AM67A/J722S) Cortex-R5F memory setup.
 *
 * The image is loaded by the Linux k3-r5 remoteproc driver, addresses are
 * device addresses as seen by the R5F core and must match the reserved
 * memory carveouts in the host device tree.
 */

__und_stack_size__ = DEFINED(__und_stack_size__) ? __und_stack_size__ : 0x100;
__abt_stack_size__ = DEFINED(__abt_stack_size__) ? __abt_stack_size__ : 0x100;
__fiq_stack_size__ = DEFINED(__fiq_stack_size__) ? __fiq_stack_size__ : 0x100;
__irq_stack_size__ = DEFINED(__irq_stack_size__) ? __irq_stack_size__ : 0x400;
__svc_stack_size__ = DEFINED(__svc_stack_size__) ? __svc_stack_size__ : 0x100;

/* Q-25: 0x800 (2048 B) is not enough for ArduPlane's main-thread call chain
   (AP_AHRS::update() -> DCM::update() -> drift_correction() alone measures
   ~1248 B at the -O0 build waf uses for AP objects); the overflow silently
   corrupted the SVC/IRQ stacks below it with every debug detector in
   chconf.h left off, which is why the loop died after exactly one
   iteration with no trace line and no fault. 0x2000 (8192 B) leaves >15k
   of the 23.4k free TCM untouched; still short of upstream ArduPilot's own
   PROCESS_STACK default (0x1C00) but comfortable headroom over the
   measured chain. Revisit if NavEKF3_core::CovariancePrediction (952 B
   alone) ends up in the same call path.*/
__sys_stack_size__ = DEFINED(__sys_stack_size__) ? __sys_stack_size__ : 0x2000;

/* Core 1 is unused on this board (single R5F, no SMP boot path) but
   rules_stacks_c1.ld mirrors every __c1_*_stack_size__ default onto its
   c0 counterpart, so leaving this undefined would silently double the
   sys-stack TCM cost for a core that never runs. Pin it back down. */
__c1_sys_stack_size__ = DEFINED(__c1_sys_stack_size__) ? __c1_sys_stack_size__ : 0x100;

MEMORY
{
    /* The MCU domain core has a single usable TCM, mapped at address 0 by
       ti,loczrama = <0> (the ATCM lands at 0x41010000 and is left disabled
       by ti,atcm-enable = <0>). Vectors, the pre-MPU boot code and the
       banked mode stacks therefore share it.

       It is split into two regions so that the vectors are guaranteed to
       land at address 0, where the Cortex-R5 fetches its reset vector (it
       has no VBAR). A single region would not do: the sections below are
       allocated in script order, so .tcm_probe would take address 0 and
       push the vectors out of reach.

       The top 64 bytes are left out of both regions, reserved for the boot
       witness and fault blocks written by board.c and for the temporary
       boot stack that grows down from there.*/
    tcm_vec   (rwx) : org = 0x00000000, len = 256
    tcm       (rwx) : org = 0x00000100, len = 32k - 256 - 64
    ddr_rsc   (rw)  : org = 0xA1100000, len = 4k    /* resource table      */
    ddr_trace (rw)  : org = 0xA1110000, len = 16k   /* remoteproc trace    */
    /* MAVLink IPC rings shared with Linux (ipc_ring.c). Deliberately inside
       the 1 MB window MPU region 4 already maps NORMAL_NONCACHE | SHARED |
       XN (board.c), the same window the trace buffer lives in -- so this
       needs no MPU change and inherits an attribute set proven correct on
       every boot. Must stay below 0xA1200000 (region 4's end) and must be
       covered by a host device-tree reserved-memory node, see Q-37.*/
    ddr_ipc   (rw)  : org = 0xA1120000, len = 64k   /* MAVLink IPC rings   */
    ddr       (rwx) : org = 0xA1240000, len = 14080k /* code, data, heap   */

    /* Unused regions referenced by the generic memory rules.*/
    flash0 (rx) : org = 0x00000000, len = 0
    flash1 (rx) : org = 0x00000000, len = 0
    flash2 (rx) : org = 0x00000000, len = 0
    flash3 (rx) : org = 0x00000000, len = 0
    flash4 (rx) : org = 0x00000000, len = 0
    flash5 (rx) : org = 0x00000000, len = 0
    flash6 (rx) : org = 0x00000000, len = 0
    flash7 (rx) : org = 0x00000000, len = 0
    ram0   (wx) : org = 0x00000000, len = 0
    ram1   (wx) : org = 0x00000000, len = 0
    ram2   (wx) : org = 0x00000000, len = 0
    ram3   (wx) : org = 0x00000000, len = 0
    ram4   (wx) : org = 0x00000000, len = 0
    ram5   (wx) : org = 0x00000000, len = 0
    ram6   (wx) : org = 0x00000000, len = 0
    ram7   (wx) : org = 0x00000000, len = 0
}

/* For each data/text section two region are defined, a virtual region
   and a load region (_LMA suffix).*/

/* Region to be used for exception vectors.*/
REGION_ALIAS("VECTORS_FLASH", tcm_vec);
REGION_ALIAS("VECTORS_FLASH_LMA", tcm_vec);

/* Region to be used for constructors and destructors.*/
REGION_ALIAS("XTORS_FLASH", ddr);
REGION_ALIAS("XTORS_FLASH_LMA", ddr);

/* Region to be used for code text.*/
REGION_ALIAS("TEXT_FLASH", ddr);
REGION_ALIAS("TEXT_FLASH_LMA", ddr);

/* Region to be used for read only data.*/
REGION_ALIAS("RODATA_FLASH", ddr);
REGION_ALIAS("RODATA_FLASH_LMA", ddr);

/* Region to be used for various.*/
REGION_ALIAS("VARIOUS_FLASH", ddr);
REGION_ALIAS("VARIOUS_FLASH_LMA", ddr);

/* Region to be used for RAM(n) initialization data.*/
REGION_ALIAS("RAM_INIT_FLASH_LMA", ddr);

/* RAM region to be used for the banked mode stacks.*/
REGION_ALIAS("STACKS_RAM", tcm);
REGION_ALIAS("C1_STACKS_RAM", tcm);

/* RAM region to be used for data segment.*/
REGION_ALIAS("DATA_RAM", ddr);
REGION_ALIAS("DATA_RAM_LMA", ddr);

/* RAM region to be used for BSS segment.*/
REGION_ALIAS("BSS_RAM", ddr);

/* RAM region to be used for the default heap.*/
REGION_ALIAS("HEAP_RAM", ddr);

/* RemoteProc sections at fixed device addresses.*/
SECTIONS
{
    .resource_table : ALIGN(4)
    {
        KEEP(*(.resource_table))
    } > ddr_rsc

    .trace (NOLOAD) : ALIGN(4)
    {
        KEEP(*(.trace))
    } > ddr_trace

    /* NOLOAD like .trace: there is nothing meaningful to load, the header is
       established at run time by ipc_ring_init().

       Do NOT read this as "the contents survive a firmware reload". They do
       not: Linux's rproc_elf_load_segments() memsets the (memsz - filesz)
       tail of every PT_LOAD segment, so remoteproc zeroes this whole window
       on each load and the epoch restarts at 1. The Linux bridge must
       therefore resync on an invalidated magic or an impossible index pair,
       not on the epoch alone -- see gem-mavbridge.*/
    .ipc (NOLOAD) : ALIGN(4)
    {
        KEEP(*(.ipc))
    } > ddr_ipc

    /* Bring-up boot probe, runs from TCM before any DDR execution, the
       banked mode stacks are allocated after it by rules_stacks.ld.*/
    .tcm_probe : ALIGN(4)
    {
        KEEP(*(.tcm_probe))
    } > tcm
}

/* Stack rules.
 *
 * Deliberately NOT `INCLUDE rules_stacks.ld`. That file emits all six banked
 * stacks into a single .stacks section in STACKS_RAM, which puts the sys stack
 * -- and the sys stack IS the main thread's stack -- in TCM along with the
 * exception stacks. TCM is internal to the R5F and is therefore invisible to
 * Linux: /dev/mem can read every byte of the DDR carveout but cannot reach TCM
 * at all.
 *
 * That mattered on 2026-08-01. Q-32 turned out to be a livelock in the main
 * thread (still scheduled, but never leaving one inner retry loop), and the
 * post-mortem technique that identified the state of every other thread from
 * Linux -- walk the ChibiOS registry over /dev/mem, scrape saved LRs off the
 * stack, resolve with addr2line -- worked for spi_bus and log_io precisely
 * because their stacks are heap-allocated in DDR. Main was the one thread whose
 * call chain could not be recovered, which is the only reason the loop is still
 * unidentified.
 *
 * So the sections are split: the exception stacks stay in TCM, and only the sys
 * stack moves to DDR, where a stalled main thread can be read post-mortem with
 * no firmware change and no debugger.
 *
 * Safe pre-MPU: remoteproc loads .text into DDR and the core executes from DDR
 * from its first instruction, so DDR is reachable well before board.c configures
 * the MPU.
 *
 * Cost is small: board.c maps the DDR region NORMAL_WBWA (write-back
 * write-allocate, MPU region 3), so stack accesses are cached, unlike the
 * deliberately non-cacheable trace/IPC window at 0xA1100000 (region 4). The
 * spi_bus and log_io threads already run on heap-allocated DDR stacks, so this
 * makes main consistent with every other thread rather than special. Measured on
 * hardware: loop rate unchanged at exactly 100 Hz, dtmax 11-12ms -> 13ms.
 *
 * Caveat for the post-mortem technique: because this region IS cached, a
 * /dev/mem read from Linux sees DDR, not the R5F's dirty D-cache lines. In a
 * stall that persists for minutes the frames of interest have long since been
 * written back, and every decode so far has been self-consistent -- but a
 * conclusion drawn from a stack read should be corroborated against the source,
 * not trusted on its own.
 */
SECTIONS
{
    .stacks (NOLOAD) :
    {
        . = ALIGN(8);
        __und_stack_base__ = .;
        . += __und_stack_size__;
        . = ALIGN(8);
        __und_stack_end__ = .;

        __abt_stack_base__ = .;
        . += __abt_stack_size__;
        . = ALIGN(8);
        __abt_stack_end__ = .;

        __fiq_stack_base__ = .;
        . += __fiq_stack_size__;
        . = ALIGN(8);
        __fiq_stack_end__ = .;

        __irq_stack_base__ = .;
        . += __irq_stack_size__;
        . = ALIGN(8);
        __irq_stack_end__ = .;

        __svc_stack_base__ = .;
        . += __svc_stack_size__;
        . = ALIGN(8);
        __svc_stack_end__ = .;
    } > tcm

    .sysstack (NOLOAD) :
    {
        . = ALIGN(8);
        __sys_stack_base__ = .;
        __main_thread_stack_base__ = .;
        . += __sys_stack_size__;
        . = ALIGN(8);
        __sys_stack_end__ = .;
        __main_thread_stack_end__ = .;
    } > ddr
}

INCLUDE rules_stacks_c1.ld

/* Code rules inclusion.*/
INCLUDE rules_code.ld

/* Data rules inclusion.*/
INCLUDE rules_data.ld

/* Memory rules inclusion.*/
INCLUDE rules_memory.ld
