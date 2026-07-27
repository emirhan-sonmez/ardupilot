# encoding: utf-8
"""
Minimal waf tool for the GemstoneO1R5F board (TI AM67/J722S Cortex-R5F).

Unlike chibios.py this does NOT use the STM32 hwdef generator. It builds the
external AM67 ChibiOS port into libch.a (reusing the exact fragments/flags/linker
script of the working RT-GEMSTONE-O1-R5F demo, via hwdef/chibios_board.mk) and
links the ArduPilot objects against it to produce a Cortex-R5F ELF.

Link ordering: AP objects reference ChibiOS symbols and ChibiOS crt0 references
main() (provided by AP), so libch.a is linked AFTER the objects via STLIB
(-L<dir> -lch). The entry symbol _vectors is pulled from libch.a by --entry.
"""

import os
import subprocess

from waflib import Errors
from waflib.TaskGen import after_method
from waflib.TaskGen import feature


def configure(cfg):
    cfg.find_program('make', var='MAKE')

    env = cfg.env

    # Locate the external AM67 ChibiOS port.
    ch_root = os.environ.get('GEMSTONE_CHIBIOS_ROOT', '')
    if not ch_root:
        ch_root = os.path.normpath(
            os.path.join(cfg.srcnode.abspath(), '..', 'chibiOS-port', 'ChibiOS'))
    if not os.path.isdir(ch_root):
        cfg.fatal('AM67 ChibiOS port not found at %r; set GEMSTONE_CHIBIOS_ROOT'
                  % ch_root)

    hwdef = cfg.srcnode.find_dir('libraries/AP_HAL_ChibiOS_K3/hwdef').abspath()

    env.CH_K3_ROOT = ch_root
    env.CH_K3_HWDEF = hwdef
    env.CH_K3_BOARD_MK = os.path.join(hwdef, 'chibios_board.mk')
    env.CH_K3_LDSCRIPT = os.path.join(hwdef, 'AM67_R5F.ld')
    env.CH_K3_LDRULES = os.path.join(
        ch_root, 'os', 'common', 'startup', 'ARMCRx', 'compilers', 'GCC', 'ld')

    cfg.msg('AM67 ChibiOS port', ch_root)

    # Generate the ChibiOS include-dir list now (via `make pass`) and add it to
    # INCLUDES at LOW priority (appended last, so AP headers win any name clash).
    # This lets the backend files that use ch.h (system/Semaphores/Scheduler/UART)
    # find the ChibiOS headers.
    builddir = cfg.bldnode.make_node('libch')
    builddir.mkdir()
    make = env.MAKE[0] if isinstance(env.MAKE, list) else env.MAKE
    subprocess.check_call([
        make, '-r', '-f', env.CH_K3_BOARD_MK, 'pass',
        'CHIBIOS=' + ch_root, 'HWDEF_DIR=' + hwdef,
        'BUILDDIR=' + builddir.abspath()])
    inc_dirs = [d.strip() for d in
                builddir.make_node('include_dirs').read().splitlines() if d.strip()]
    env.CH_K3_INCLUDES = inc_dirs
    env.INCLUDES = list(env.INCLUDES) + inc_dirs
    cfg.msg('ChibiOS include dirs', str(len(inc_dirs)))

    env.AP_PROGRAM_FEATURES += ['ch_k3_program']

    # libap.a (AP) and libch.a (ChibiOS) reference each other (AP -> ChibiOS
    # symbols; ChibiOS crt0 -> main). Wrap the static libs in a group so the
    # linker re-scans them until stable, independent of order. No dynamic libs in
    # a bare-metal link, so repurposing the STLIB/SHLIB markers is safe.
    env.STLIB_MARKER = '-Wl,--start-group'
    env.SHLIB_MARKER = '-Wl,--end-group'

    # R5F machine flags: must match the libch.a compile so libgcc/libc multilib
    # selection agrees at link time.
    mflags = ['-mcpu=cortex-r5', '-mfloat-abi=hard', '-mfpu=vfpv3-d16']

    # Link flags mirroring the demo LDFLAGS (ARM rules.mk): ChibiOS provides the
    # startup, so -nostartfiles; the ldscript pulls in rules_*.ld from LDRULES.
    env.CH_K3_LINKFLAGS = mflags + [
        '-nostartfiles',
        # -L must precede --script: ld resolves the script's INCLUDE directives
        # (rules_stacks.ld etc.) against the -L paths seen so far.
        '-Wl,-L' + env.CH_K3_LDRULES,
        '-Wl,--script=' + env.CH_K3_LDSCRIPT,
        '-Wl,--entry=_vectors',
        '-Wl,--gc-sections',
        '-Wl,--no-warn-mismatch',
        # Force the newlib syscall/C++-runtime object (k3_syscalls.c) to be
        # pulled from libap.a at scan time. Otherwise libc (added last by gcc)
        # references _sbrk/_write/... but the defining archive member was already
        # passed. Pulling one symbol pulls the whole object (all syscalls +
        # __dso_handle + _fini).
        '-Wl,-u,_sbrk',
        # Force the RemoteProc resource table (rsc_table.c) to be pulled from
        # libch.a. Nothing in ArduPilot references it, so without this the
        # archive member is never linked and the ELF ships with no
        # .resource_table section -> the K3 remoteproc core will not start the
        # firmware (and no trace0 appears). The ld script KEEP()s the section
        # once the object is in the link.
        '-Wl,-u,resource_table',
    ]


def build(bld):
    env = bld.env
    builddir = bld.bldnode.make_node('libch')
    libch = builddir.make_node('libch.a')
    include_dirs = builddir.make_node('include_dirs')

    rule = ('"${MAKE}" -r -f "%s" lib pass '
            'CHIBIOS="%s" HWDEF_DIR="%s" BUILDDIR="%s" -j%d') % (
        env.CH_K3_BOARD_MK, env.CH_K3_ROOT, env.CH_K3_HWDEF,
        builddir.abspath(), bld.options.jobs)

    bld(
        name='libch_k3',
        rule=rule,
        target=[libch, include_dirs],
        group='dynamic_sources',
        always=True,
    )


@feature('ch_k3_program')
@after_method('apply_link')
def ch_k3_link(self):
    if not getattr(self, 'link_task', None):
        return

    bld = self.bld
    env = self.env
    builddir = bld.bldnode.make_node('libch')
    libch = builddir.make_node('libch.a')

    # Guard against appending twice if the feature runs for multiple programs.
    if not env.CH_K3_WIRED:
        env.append_value('LINKFLAGS', env.CH_K3_LINKFLAGS)
        # Emit a map file next to the ELF for inspection.
        env.append_value('LINKFLAGS',
                         ['-Wl,-Map=' + self.link_task.outputs[0].abspath() + '.map,--cref'])
        # libch.a AFTER the objects: STLIB expands to -L<dir> -lch post-SRC.
        env.append_value('STLIBPATH', [builddir.abspath()])
        env.append_value('STLIB', ['ch'])
        env.CH_K3_WIRED = True

    # The link must wait for libch.a to be built and rebuild if it changes.
    self.link_task.dep_nodes.append(libch)
    try:
        libtg = bld.get_tgen_by_name('libch_k3')
        libtg.post()
        for t in libtg.tasks:
            self.link_task.set_run_after(t)
    except Errors.WafError:
        pass
