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

void *_sbrk(ptrdiff_t incr)
{
    static char *heap = 0;
    char *prev;
    char *next;

    if (heap == 0) {
        heap = &__heap_base__;
    }
    prev = heap;
    next = heap + incr;
    if (next > &__heap_end__) {
        errno = ENOMEM;
        return (void *)-1;
    }
    heap = next;
    return prev;
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
