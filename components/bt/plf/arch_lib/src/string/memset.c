/* Public domain.  */
#include <stddef.h>
#include "ms_section.h"

BOOTROM_FUNCTION void *
memset (void *dest, int val, size_t len)
{
  unsigned char *ptr = dest;
  while (len-- > 0)
    *ptr++ = val;
  return dest;
}