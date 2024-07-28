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


OUTPUT_ARCH( "riscv" )

ENTRY( _start )

MEMORY
{
  flash (rwx) : ORIGIN = 0x20000000, LENGTH = 0x80000 /* 512K */

  rwip_ram (rwx)   : ORIGIN = 0x0FFE0000, LENGTH = 0x2000  /* 8K */
  ram (rwx)   : ORIGIN = 0x0FFE2000, LENGTH = 0x36000  /* 220K */
  mdata (rwx)   : ORIGIN = 0x10030100, LENGTH = 0x700  /* 1752 */
  retram (rwx)   : ORIGIN = 0x10030800, LENGTH = 0x800  /* 2K */  
  rom (rwx)   : ORIGIN = 0x00001000, LENGTH = 0x1F000  /* 124K */
}

ENTRY(_start)
SECTIONS
{
  __stack_size = DEFINED(__stack_size) ? __stack_size : 2K;

  .init           :
  {
    *(.vtable)
    KEEP (*(SORT_NONE(.init)))
  } >flash AT>flash

  .ilalign         :
  {
    . = ALIGN(4);
    PROVIDE( _ilm_lma = . );
  } >flash AT>flash

  .ialign         :
  {
    PROVIDE( _ilm = . );
  } >flash AT>flash

/****************  rwip   **********************/

  /* used by the startup to initialize data */
  _sirom = LOADADDR(.func.rom);
  /* Initialized func.rom sections goes into ROM, load LMA copy after code */
  .func.rom    :
  {
    . = ALIGN(4);
    _srom = .;        /* create a global symbol at rom start */
    KEEP(*libceva_ble.a:*(.text*))
    . = ALIGN(4);
    KEEP(*libceva_ble.a:*(.rodata* .srodata*))
    . = ALIGN(4);
    *(.func.rom)          /* .func.rom sections */
    . = ALIGN(4);
    _erom = .;        /* define a global symbol at rom end */
  } >rom AT>rom
  
  _rwip_start = LOADADDR(.rwip);
  .rwip          :ALIGN(4)
  {
    . = ALIGN(4);
    _srwip_section = .;
    KEEP(*libceva_ble.a:*(.sdata* .data*))

	. = ALIGN(4);
	_srwip_bss_section = .;
    KEEP(*libceva_ble.a:*(COMMON .sbss* .bss*))

	. = ALIGN(4);
     _erwip_section = .;
  } >rwip_ram AT>flash
  
/*************  rwip end  *******************/

  .text           :
  {
    *(.text.unlikely .text.unlikely.*)
    *(.text.startup .text.startup.*)
    *(.text .text.*)
    *(.gnu.linkonce.t.*)
  } >flash AT>flash

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
  } >flash AT>flash

  .fini           :
  {
    KEEP (*(SORT_NONE(.fini)))
  } >flash AT>flash

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
  } >flash AT>flash

  .init_array     :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
    KEEP (*(.init_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .ctors))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >flash AT>flash

  .fini_array     :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT_BY_INIT_PRIORITY(.fini_array.*) SORT_BY_INIT_PRIORITY(.dtors.*)))
    KEEP (*(.fini_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .dtors))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >flash AT>flash

  .ctors          :
  {
    /* gcc uses crtbegin.o to find the start of
       the constructors, so we make sure it is
       first.  Because this is a wildcard, it
       doesn't matter if the user does not
       actually link against crtbegin.o; the
       linker won't look for a file to match a
       wildcard.  The wildcard also means that it
       doesn't matter which directory crtbegin.o
       is in.  */
    KEEP (*crtbegin.o(.ctors))
    KEEP (*crtbegin?.o(.ctors))
    /* We don't want to include the .ctor section from
       the crtend.o file until after the sorted ctors.
       The .ctor section from the crtend file contains the
       end of ctors marker and it must be last */
    KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
  } >flash AT>flash

  .dtors          :
  {
    KEEP (*crtbegin.o(.dtors))
    KEEP (*crtbegin?.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
  } >flash AT>flash

  .lalign         :
  {
    . = ALIGN(4);
    PROVIDE( _data_lma = . );
  } >flash AT>flash

  .dalign         :
  {
    . = ALIGN(4);
    PROVIDE( _data = . );
  } >ram AT>flash

  .data          :
  {
    *(.data .data.*)
    *(.gnu.linkonce.d.*)
    . = ALIGN(8);
    PROVIDE( __global_pointer$ = . + 0x1000 );
    *(.sdata .sdata.* .sdata*)
    *(.gnu.linkonce.s.*)
    . = ALIGN(4);
    *(.rwip.data .rwip.const)
  } >ram AT>flash

  . = ALIGN(4);
  PROVIDE( _edata = . );
  PROVIDE( edata = . );
  
  _sifunc = LOADADDR(.func);

  /* Initialized func sections goes into ROM, load LMA copy after code */
  .func :
  {
    . = ALIGN(4);
    _sfunc = .;        /* create a global symbol at func start */
    *(.func)          /* .func sections */
    . = ALIGN(4);
    *(.rwip.func)
    . = ALIGN(4);
    _efunc = .;        /* define a global symbol at func end */
  } >ram AT>flash

  . = ALIGN(4);
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
    *(.rwip.bss)
    . = ALIGN(4);
  } >ram AT>ram

  . = ALIGN(8);
  PROVIDE( _end = . );
  PROVIDE( end = . );
  . = ALIGN(4);
  
  .retdata :
  {
    . = ALIGN(4);
    *(.retram.data)           /* .retention.data sections */
    *(.retram.data*)          /* .retention.data* sections */

    . = ALIGN(4);
  } >mdata AT>flash
  
  /* used by the startup to initialize data */
  _sifunc_retram = LOADADDR(.func_ret);

  /* Initialized func sections goes into ROM, load LMA copy after code */
  .func_ret :
  {
    . = ALIGN(4);
    _sfunc_retram = .;     /* create a global symbol at .func.retram start */
    *(.func_retram)        /* .func.retram sections */
    . = ALIGN(4);
    _efunc_retram = .;            /* define a global symbol at .func.retram end */
  } >retram AT>flash
  
  /* Define stack and head location at ram */
  .stack ORIGIN(ram) + LENGTH(ram) - __stack_size :
  {
    PROVIDE( _heap_end = . );
    __StackLimit = .;
    . = __stack_size;
    __StackTop = .;
    PROVIDE( _sp = . );
  } >ram AT>ram


}
