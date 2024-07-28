//--------------------------------------------------------------------
// Copyright (c) 2019 by TCL Research.
// All rights reserved.
// TCL Research Confidential Proprietary.
//--------------------------------------------------------------------
// Project name: soc_com 
// Verif name tb_soc
// File name: type_def.h
// Author: jiangd
// Dates: 2021-10-14 09:51:38
// Version: V1.0
//-------------------------------------------------------------------
// Purpose: Basic type definitions
//-------------------------------------------------------------------
#ifndef TYPE_DEF__H
#define TYPE_DEF__H

    #include <stddef.h>

#ifndef RISCV_N307
#define  RISCV_N307 1
#endif

    typedef unsigned int        UINT32;
    typedef unsigned short      UINT16;
    typedef unsigned char       UINT8;

    #define inl(addr)           (*((volatile UINT32 *)(addr)))
    #define inw(addr)           (*((volatile UINT16 *)(addr)))
    #define inb(addr)           (*((volatile UINT8  *)(addr)))

    #define outl(addr, val)     (*((volatile UINT32 *)(addr)) = (val))
    #define outw(addr, val)     (*((volatile UINT16 *)(addr)) = (val))
    #define outb(addr, val)     (*((volatile UINT8  *)(addr)) = (val))

    //for interrupt application
    #ifdef CORTEX_M3
        #include "SSE050.h"
    #elif RISCV_N307
        #include "NUCLEI_N.h"

        typedef enum {
            CPU_INTR_LEVEL      = 0,
            CPU_INTR_EDGE_POS   = 1,
            CPU_INTR_EDGE_NEG   = 3
        } cpu_intr_trig_type;

    #endif

#endif
