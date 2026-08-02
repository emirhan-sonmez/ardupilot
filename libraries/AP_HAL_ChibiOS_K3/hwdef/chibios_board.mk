##############################################################################
# ChibiOS static-library build for the AP_HAL_ChibiOS_K3 board (Gemstone O1 R5F).
#
# Produces libch.a from the AM67 ChibiOS port, reusing the EXACT source lists,
# fragments, compiler flags and linker script of the working
# demos/various/RT-GEMSTONE-O1-R5F demo. main.c and the demo-only UART helper
# are intentionally omitted; ArduPilot provides the application and main().
#
# Driven by Tools/ardupilotwaf/chibios_k3.py via `make lib` and `make pass`.
# Expects CHIBIOS, BUILDDIR and HWDEF_DIR to be passed in on the command line.
##############################################################################

# ---- global options (mirror the RT-GEMSTONE-O1-R5F Makefile) ----
USE_OPT ?= -O2 -ggdb -fomit-frame-pointer -falign-functions=16
USE_COPT ?=
USE_CPPOPT ?= -fno-rtti
USE_LINK_GC ?= yes
USE_LDOPT ?= --entry=_vectors
USE_LTO ?= no
USE_THUMB ?= no
USE_VERBOSE_COMPILE ?= no
USE_SMART_BUILD ?= yes

USE_SYSTEM_STACKSIZE ?= 0x800
USE_IRQ_STACKSIZE ?= 0x400
USE_FIQ_STACKSIZE ?= 0x100
USE_SUPERVISOR_STACKSIZE ?= 0x100
USE_UND_STACKSIZE ?= 0x100
USE_ABT_STACKSIZE ?= 0x100
USE_FPU ?= hard

# Project name -> lib$(PROJECT).a == libch.a
PROJECT = ch

# CHIBIOS (external AM67 port), BUILDDIR and HWDEF_DIR are passed in by waf.
CONFDIR  := $(HWDEF_DIR)/cfg
BUILDDIR ?= ./build
DEPDIR   := $(BUILDDIR)/.dep

# Target settings.
MCU = cortex-r5

# ---- ChibiOS fragments: EXACT demo set ----
include $(CHIBIOS)/os/license/license.mk
include $(CHIBIOS)/os/common/startup/ARMCRx/compilers/GCC/mk/startup_armcr5.mk
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/TI/AM67/platform.mk
include $(CHIBIOS)/os/hal/boards/T3_GEMSTONE_O1_R5F/board.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv7-R/compilers/GCC/mk/port.mk

# Linker script (used for the AP final link, not for the lib archive).
LDSCRIPT = $(HWDEF_DIR)/AM67_R5F.ld

# ChibiOS sources + board boot infra (resource table, remoteproc trace,
# stack painting, and the shared-memory MAVLink rings to Linux).
CSRC = $(ALLCSRC) \
       $(HWDEF_DIR)/boot/rsc_table.c \
       $(HWDEF_DIR)/boot/trace.c \
       $(HWDEF_DIR)/boot/stack_paint.c \
       $(HWDEF_DIR)/boot/ipc_ring.c \
       $(HWDEF_DIR)/boot/ipc_storage.c \
       $(CHIBIOS)/os/hal/ports/TI/AM67/am67_wdt.c
CPPSRC  = $(ALLCPPSRC)
ACSRC   =
ACPPSRC =
TCSRC   =
TCPPSRC =
ASMSRC  = $(ALLASMSRC)
ASMXSRC = $(ALLXASMSRC)

INCDIR = $(CONFDIR) $(ALLINC) $(HWDEF_DIR)/boot

AOPT = -marm
TOPT = -mthumb -DTHUMB
CWARN   = -Wall -Wextra -Wundef -Wstrict-prototypes
CPPWARN = -Wall -Wextra -Wundef

# AM67A/J722S R5FSS feature set (from the demo).
UDEFS = -DARMCR5_HAS_MPU=1      \
        -DARMCR5_MPU_REGIONS=16 \
        -DARMCR5_HAS_ICACHE=1   \
        -DARMCR5_HAS_DCACHE=1   \
        -DARMCR5_HAS_DTCM=1     \
        -DARMCR5_HAS_ECC=1
ifneq ($(USE_FPU),no)
  UDEFS += -DARMCR5_HAS_FPU=1
endif

# Cortex-R5 has no VBAR: fixed vector base at 0, crt0 must not write VBAR.
UADEFS  = -DCRT0_VBAR_INIT=0
UINCDIR =
ULIBDIR =
ULIBS   =

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCRx/compilers/GCC
include $(RULESPATH)/mk/arm-none-eabi.mk
include $(RULESPATH)/mk/rules.mk

# ---- extra targets used by chibios_k3.py ----
# Emit the include-dir list so the AP objects that use ch.h (system.cpp,
# syscalls.c) can find the ChibiOS headers. Paths are absolute (CHIBIOS/HWDEF_DIR
# are passed absolute), one per line.
.PHONY: pass
pass:
	@mkdir -p $(BUILDDIR)
	@echo "$(INCDIR) $(DINCDIR) $(UINCDIR)" | tr ' ' '\n' | sed '/^$$/d' | sort -u > $(BUILDDIR)/include_dirs
	@echo Wrote $(BUILDDIR)/include_dirs
