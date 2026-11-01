#include <stdio.h>

/* GCC/newlib _write uses __io_putchar; the legacy UI routes fputc to UART5. */
int __io_putchar(int ch) { return fputc(ch, stdout); }
