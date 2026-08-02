#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3

#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <stddef.h>

/*
  Minimal newlib syscall layer for the AM67/K3 bring-up.

  These resolve the libc syscalls (malloc -> _sbrk, printf -> _write, ...) using
  the existing ChibiOS/startup solution and honest bare-metal behaviour:

    _sbrk   hands out the ChibiOS heap region (__heap_base__ .. __heap_end__,
            defined by the ChibiOS linker rules in rules_memory.ld).
    _write  discards output (no console wired yet -> a null console, NOT a fake
            file). M3 routes this to the real UART console.
    others  honest minimal stubs; _exit halts the core.

  There is deliberately no fake Linux / filesystem behaviour here.
*/

/* Heap region symbols from the ChibiOS linker rules (rules_memory.ld). */
extern char __heap_base__;
extern char __heap_end__;

/* File scope so k3_heap_remaining() can report against it. */
static char *k3_heap_break = 0;

void *_sbrk(ptrdiff_t incr)
{
    char *prev;
    char *next;

    if (k3_heap_break == 0) {
        k3_heap_break = &__heap_base__;
    }
    char *heap = k3_heap_break;
    prev = heap;
    next = heap + incr;
    if (next > &__heap_end__) {
        errno = ENOMEM;
        return (void *)-1;
    }
    k3_heap_break = next;
    return prev;
}

/*
  Bytes left in the sbrk arena.

  ArduPilot asks via Util::available_memory(), and the answer gates real
  behaviour rather than being informational: AP_NavEKF3 refuses to start unless
  it can see sizeof(NavEKF3_core)*cores + 4096 bytes free. AP_HAL::Util's base
  implementation returns a hardcoded 4096, so any port that does not override
  it disables EKF3 outright regardless of how much memory the board has. That
  is what kept EKF3 off this board until 2026-08-02, on 14 MB of DDR.

  Reports the unhanded-out remainder rather than the largest free block:
  nothing here ever frees back to sbrk, so the two are identical and the
  simpler answer cannot drift out of step with the allocator.
*/
size_t k3_heap_remaining(void)
{
    const char *cur = (k3_heap_break == 0) ? &__heap_base__ : k3_heap_break;
    if (cur >= &__heap_end__) {
        return 0;
    }
    return (size_t)(&__heap_end__ - cur);
}

int _write(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    /* No console yet; report the bytes as consumed so libc does not stall. */
    return len;
}

int _read(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    (void)len;
    return 0; /* EOF */
}

int _close(int file)
{
    (void)file;
    return -1;
}

int _lseek(int file, int ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;
    return 0;
}

int _fstat(int file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR; /* treat stdio as a character device */
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

int _getpid(void)
{
    return 1;
}

void _exit(int status)
{
    (void)status;
    while (1) {
    }
}

/*
  C++ runtime bits normally supplied by crtbegin/crti, which -nostartfiles drops
  (ChibiOS provides its own startup). __dso_handle backs __cxa_atexit for static
  destructors; _fini is the (empty) finalisation hook. Keeping them in this
  object means forcing it in (-u _sbrk) also resolves the C++ runtime.
*/
void *__dso_handle = (void *)0;

void _fini(void)
{
}

#endif /* CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS_K3 */
