/**
 ****************************************************************************************
 *
 * @file stdio.h
 *
 * @brief generic string printf for BlueGRiP platform, based on Public domain code.
 *
 ****************************************************************************************
 */

#ifndef _STDIO_H
#define _STDIO_H

typedef struct __FILE FILE;

#include <stddef.h>

int     sprintf(char *, const char *, ...);
int     fputc(int, FILE *);

/* FILE type.
   There are different variations on this for different types of stream IO -
   but all have the inch or outch function as the first member. */
struct __FILEout {
    void (*outch) (void* f, int c);
};

struct __FILEin {
    int (*inch) (void* f);    
    /* character pushed back by ungetc or -1 if none */
    int ungetc;
};
    
struct __FILE {
    union {
	struct __FILEout out;
	struct __FILEin  in;
    };
};

#endif
