/**
 ****************************************************************************************
 *
 * @file string.h
 *
 * @brief generic string operations for BlueGRiP platform.
 *
 ****************************************************************************************
 */

#ifndef _STRING_H
#define _STRING_H

#include <stddef.h>

int memcmp(const void *, const void *, size_t);
void *   memcpy(void *, const void *, size_t);
void *   memset(void  *, int, size_t);
char *strcpy(char *dst, const char *src);
size_t   strlen(const char *);

#endif
