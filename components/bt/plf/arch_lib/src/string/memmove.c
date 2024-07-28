
#include "ms_section.h"
/* Public domain.  */
#include <stddef.h>

BOOTROM_FUNCTION void *
memmove (void *dest,  void *src, size_t len)
{
  char *d = dest;
   char *s = src;
  if (d < s)
    while (len--)
      *d++ = *s++;
  else
    {
      char *lasts = s + (len-1);
      char *lastd = d + (len-1);
      while (len--)
        *lastd-- = *lasts--;
    }
  return dest;
}