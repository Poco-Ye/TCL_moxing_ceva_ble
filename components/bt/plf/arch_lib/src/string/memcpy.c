/* Public domain.  */
#include <stddef.h>
#include "ms_section.h"

BOOTROM_FUNCTION void *
memcpy (void *dest, const void *src, size_t len)
{
  char *d = dest;
  const char *s = src;
  while (len--)
    *d++ = *s++;
  return dest;
}