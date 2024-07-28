/* Public domain.  */
#include <stddef.h>
#include "ms_section.h"

BOOTROM_FUNCTION char *
strcpy(char *dst, const char *src)
{
    char *d = dst;
    const char *s = src;

    while ((*d++ = *s++))
        ;
    return dst;
}
