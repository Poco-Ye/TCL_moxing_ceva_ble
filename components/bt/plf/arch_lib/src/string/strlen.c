/* Public domain.  */

#include "ms_section.h"
#include <string.h>

BOOTROM_FUNCTION size_t
strlen (const char *str)
{
  const char *s1 = str;
  
  while (*s1)
    s1++;

  return s1 - str;
}
