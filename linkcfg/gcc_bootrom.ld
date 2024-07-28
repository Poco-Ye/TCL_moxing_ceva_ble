/*
 * Copyright (c) 2019 Nuclei Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/******************************************************************************
 * @file     gcc_NUCLEI_N.ld
 * @brief    GNU Linker Script for Nuclei N based device
 * @version  V1.10
 * @date     30. July 2021
 ******************************************************************************/

/*********** Use Configuration Wizard in Context Menu *************************/

OUTPUT_ARCH( "riscv" )
/********************* Flash Configuration ************************************
 * <h> Flash Configuration
 * <o0> Flash Base Address <0x0-0xFFFFFFFF:8>
 * <o1> Flash Size (in Bytes) <0x0-0xFFFFFFFF:8>
 * </h>
 */
/* waived by jiangde
__ROM_BASE = 0x20000000;
__ROM_SIZE = 0x00400000;
 */

/*--------------------- ILM RAM Configuration ---------------------------
 * <h> ILM RAM Configuration
 * <o0> ILM RAM Base Address    <0x0-0xFFFFFFFF:8>
 * <o1> ILM RAM Size (in Bytes) <0x0-0xFFFFFFFF:8>
 * </h>
 */
/* waived by jiangde
__ILM_RAM_BASE = 0x80000000;
__ILM_RAM_SIZE = 0x00010000;
 */

/*--------------------- Embedded RAM Configuration ---------------------------
 * <h> RAM Configuration
 * <o0> RAM Base Address    <0x0-0xFFFFFFFF:8>
 * <o1> RAM Size (in Bytes) <0x0-0xFFFFFFFF:8>
 * </h>
*/
/* waived by jiangde
__RAM_BASE = 0x90000000;
__RAM_SIZE = 0x00010000;
 */

/********************* Stack / Heap Configuration ****************************
 * <h> Stack / Heap Configuration
 * <o0> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
 * <o1> Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
 * </h>
 */
/* waived by jiangde
__STACK_SIZE = 0x00000800;
__HEAP_SIZE  = 0x00000800;
 */

/**************************** end of configuration section ********************/

/* Define base address and length of flash and ram */
MEMORY
{
  /* modified by jiangde
  flash (rxai!w) : ORIGIN = __ROM_BASE, LENGTH = __ROM_SIZE
  ram (wxa!ri) : ORIGIN = __RAM_BASE, LENGTH = __RAM_SIZE
   */
  rom (rx!w)  : ORIGIN = 0x00000000, LENGTH = 0x4000  /* 16K */
  ram (rwx)   : ORIGIN = 0x10002000, LENGTH = 0x2000  /* 8K */
  flash (rwx) : ORIGIN = 0x20000000, LENGTH = 0x10000 /* 64K */
}
/* Linker script to place sections and symbol values. Should be used together
 * with other linker script that defines memory regions FLASH,ILM and RAM.
 * It references following symbols, which must be defined in code:
 *   _start : Entry of reset handler
 *
 * It defines following symbols, which code can use without definition:
 *   _ilm_lma
 *   _ilm
 *   __etext
 *   _etext
 *   etext
 *   _eilm
 *   __preinit_array_start
 *   __preinit_array_end
 *   __init_array_start
 *   __init_array_end
 *   __fini_array_start
 *   __fini_array_end
 *   _data_lma
 *   _edata
 *   edata
 *   __data_end__
 *   __bss_start
 *   __fbss
 *   _end
 *   end
 *   __heap_end
 *   __StackLimit
 *   __StackTop
 *   __STACK_SIZE
 */
/* Define entry label of program */
ENTRY(_start)
SECTIONS
{
  /* modified by jiangde
  __STACK_SIZE = DEFINED(__STACK_SIZE) ? __STACK_SIZE : 2K;
   */
  __STACK_SIZE = DEFINED(__STACK_SIZE) ? __STACK_SIZE : 0x200;

  .init           :
  {
    /* vector table locate at flash */
    *(.vtable)
    KEEP (*(SORT_NONE(.init)))
  } >rom AT>rom

  .ilalign         :
  {
    . = ALIGN(4);
    /* Create a section label as _ilm_lma which located at flash */
    PROVIDE( _ilm_lma = . );
  } >rom AT>rom

  .ialign         :
  {
    /* Create a section label as _ilm which located at flash */
    PROVIDE( _ilm = . );
  } >rom AT>rom

  /* Code section located at flash */
  .text           :
  {
    *(.text.unlikely .text.unlikely.*)
    *(.text.startup .text.startup.*)
    *(.text .text.*)
    *(.gnu.linkonce.t.*)
  } >rom AT>rom

  .rodata : ALIGN(4)
  {
    . = ALIGN(4);
    *(.rdata)
    *(.rodata .rodata.*)
    *(.gnu.linkonce.r.*)
    . = ALIGN(8);
    *(.srodata.cst16)
    *(.srodata.cst8)
    *(.srodata.cst4)
    *(.srodata.cst2)
    *(.srodata .srodata.*)
  } >rom AT>rom

  .fini           :
  {
    KEEP (*(SORT_NONE(.fini)))
  } >rom AT>rom

  . = ALIGN(4);

  PROVIDE (__etext = .);
  PROVIDE (_etext = .);
  PROVIDE (etext = .);
  PROVIDE( _eilm = . );


  .preinit_array  :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >rom AT>rom

  .init_array     :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
    KEEP (*(.init_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .ctors))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >rom AT>rom

  .fini_array     :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT_BY_INIT_PRIORITY(.fini_array.*) SORT_BY_INIT_PRIORITY(.dtors.*)))
    KEEP (*(.fini_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .dtors))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >rom AT>rom

  .ctors          :
  {
    /* gcc uses crtbegin.o to find the start of
     * the constructors, so we make sure it is
     * first.  Because this is a wildcard, it
     * doesn't matter if the user does not
     * actually link against crtbegin.o; the
     * linker won't look for a file to match a
     * wildcard.  The wildcard also means that it
     * doesn't matter which directory crtbegin.o
     * is in.
     */
    KEEP (*crtbegin.o(.ctors))
    KEEP (*crtbegin?.o(.ctors))
    /* We don't want to include the .ctor section from
     * the crtend.o file until after the sorted ctors.
     * The .ctor section from the crtend file contains the
     * end of ctors marker and it must be last
     */
    KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
  } >rom AT>rom

  .dtors          :
  {
    KEEP (*crtbegin.o(.dtors))
    KEEP (*crtbegin?.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
  } >rom AT>rom

  .lalign         :
  {
    . = ALIGN(4);
    PROVIDE( _data_lma = . );
  } >rom AT>rom

  .dalign         :
  {
    . = ALIGN(4);
    PROVIDE( _data = . );
  } >ram AT>rom

  /* Define data section virtual address is ram and physical address is flash */
  .data          :
  {
    *(.data .data.*)
    *(.gnu.linkonce.d.*)
    . = ALIGN(8);
    PROVIDE( __global_pointer$ = . + 0x800 );
    *(.sdata .sdata.* .sdata*)
    *(.gnu.linkonce.s.*)
  } >ram AT>rom

  . = ALIGN(4);
  PROVIDE( _edata = . );
  PROVIDE( edata = . );

  PROVIDE( _fbss = . );
  PROVIDE( __bss_start = . );
  .bss            :
  {
    *(.sbss*)
    *(.gnu.linkonce.sb.*)
    *(.bss .bss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN(4);
  } >ram AT>ram

  /* added by jiangde */
  .heap (COPY):
  {
     *(.heap*)
     __HeapLimit = .;
  } > ram

  . = ALIGN(8);
  PROVIDE( _end = . );
  PROVIDE( end = . );
  /* Define stack and head location at ram */
  .stack ORIGIN(ram) + LENGTH(ram) - __STACK_SIZE :
  {
    PROVIDE( _heap_end = . );
    __StackLimit = .;
    . = __STACK_SIZE;
    __StackTop = .;
    PROVIDE( _sp = . );
  } >ram AT>ram

  /* added by jiangde */
  /* Check if data + heap + stack exceeds RAM limit */
  ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")
}
