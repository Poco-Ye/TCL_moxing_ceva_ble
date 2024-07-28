/**
****************************************************************************************
*
* @file reg_access.h
*
* @brief File implementing the basic primitives for register accesses
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/

#ifndef REG_ACCESS_H_
#define REG_ACCESS_H_

// #define REG_VM_VEH // peterlee

/**
****************************************************************************************
* @addtogroup REG REG_ACCESS
* @ingroup DRIVERS
*
* @brief Basic primitives for register access.
*
* @{
****************************************************************************************
*/


/*
* INCLUDE FILES
****************************************************************************************
*/
#include <string.h>           // string functions


/*
* DEFINES
****************************************************************************************
*/

/*
* MACROS
****************************************************************************************
*/
#ifndef WIN32
/// Macro to read a platform register
#define REG_PL_RD(addr)              (*(volatile uint32_t *)(addr))

/// Macro to write a platform register
#define REG_PL_WR(addr, value)       (*(volatile uint32_t *)(addr)) = (value)

/// Macro to read a common ip register
#define REG_IP_RD(addr)              (*(volatile uint32_t *)(addr))

/// Macro to write a common ip register
#define REG_IP_WR(addr, value)       (*(volatile uint32_t *)(addr)) = (value)

/// Macro to read a BLE register
#define REG_BLE_RD(addr)             (*(volatile uint32_t *)(addr))

/// Macro to write a BLE register
#define REG_BLE_WR(addr, value)      (*(volatile uint32_t *)(addr)) = (value)

/// Macro to read a BLE control structure field (16-bit wide)
#define EM_BLE_RD(addr)              (*(volatile uint16_t *)(addr))

/// Macro to write a BLE control structure field (16-bit wide)
#define EM_BLE_WR(addr, value)       (*(volatile uint16_t *)(addr)) = (value)

/// Macro to read a BT register
#define REG_BT_RD(addr)              (*(volatile uint32_t *)(addr))

/// Macro to write a BT register
#define REG_BT_WR(addr, value)       (*(volatile uint32_t *)(addr)) = (value)

/// Macro to read a BT control structure field (16-bit wide)
#define EM_BT_RD(addr)               (*(volatile uint16_t *)(addr))

/// Macro to write a BT control structure field (16-bit wide)
#define EM_BT_WR(addr, value)        (*(volatile uint16_t *)(addr)) = (value)

/// Macro to read accelerometer register
#define REG_ACC_RD(addr)             (accel_reg_rd((uint8_t)addr))
/// Macro to write accelerometer register
#define REG_ACC_WR(addr, value)      (accel_reg_wr((uint8_t)addr, (uint8_t)value))

/// Macro to read RTC register
#define REG_RTC_RD(addr)             (rtc_reg_rd((uint8_t)addr))
/// Macro to write RTC register
#define REG_RTC_WR(addr, value)      (rtc_reg_wr((uint8_t)addr, (uint8_t)value))

/// Macro to read a EM field (16-bit wide)
#define EM_RD(addr)                  (*(volatile uint16_t *)(addr))

/// Macro to write a EM field (16-bit wide)
#define EM_WR(addr, value)           (*(volatile uint16_t *)(addr)) = (value)

#else // REG_VM_VEH

#define REG_VM_W			(1 << 31)
#define REG_VM_VP			(1 << 30)

extern uint32_t reg_vm_rw(const char* desc, uint32_t addr, uint32_t value, uint32_t size_f);

#define REG_PL_RD(addr)              reg_vm_rw(#addr ", " __FUNCTION__, addr, 0, 4) // (*(volatile uint32_t *)(addr))
#define REG_PL_WR(addr, value)       reg_vm_rw(#addr ", " __FUNCTION__, addr, value, 4 | REG_VM_W) // (*(volatile uint32_t *)(addr)) = (value)
#define REG_IP_RD(addr)              reg_vm_rw(#addr ", " __FUNCTION__, addr, 0, 4) // (*(volatile uint32_t *)(addr))
#define REG_IP_WR(addr, value)       reg_vm_rw(#addr ", " __FUNCTION__, addr, value, 4 | REG_VM_W) // (*(volatile uint32_t *)(addr)) = (value)
#define REG_BLE_RD(addr)             reg_vm_rw(#addr ", " __FUNCTION__, addr, 0, 4) // (*(volatile uint32_t *)(addr))
#define REG_BLE_WR(addr, value)      reg_vm_rw(#addr ", " __FUNCTION__, addr, value, 4 | REG_VM_W) // (*(volatile uint32_t *)(addr)) = (value)
#define EM_BLE_RD(addr)              (uint16_t)reg_vm_rw(#addr ", " __FUNCTION__, addr, 0, 2 ) // (*(volatile uint16_t *)(addr))
#define EM_BLE_WR(addr, value)       reg_vm_rw(#addr ", " __FUNCTION__, addr, value, 2 | REG_VM_W) // (*(volatile uint16_t *)(addr)) = (value)
#define EM_RD(addr)                  (uint16_t)reg_vm_rw(#addr ", " __FUNCTION__, addr, 0, 2) // (*(volatile uint16_t *)(addr))
#define EM_WR(addr, value)           reg_vm_rw(#addr ", " __FUNCTION__, addr, value, 2 | REG_VM_W) // (*(volatile uint16_t *)(addr)) = (value)

#endif // REG_VM_VEH

/*
* INCLUDE FILES
* ****************************************************************************************
*/

/*
* These included below the above macros, to overcome an include file nesting issue about ASSERT_ERR.
* Ref. arch/ll/riscv32-gcc/ll.h for detail. TODO - clean
*/

#if (defined(CFG_BT_EMB) || defined(CFG_BLE_EMB))
#include "co_utils.h"
#include "em_map.h"       // EM Map

/*
* FUNCTION DECLARATIONS
****************************************************************************************
*/
/// retrieve EM address from system address
__INLINE uint16_t em_addr_get(void *sys_addr)
{
	return ((uint16_t)(((uint32_t)sys_addr) - EM_BASE_ADDR));
}

/// Read bytes from EM
__INLINE void em_rd(void *sys_addr, uint16_t em_addr, uint16_t len)
{
#ifndef WIN32
	memcpy(sys_addr, (void *)(em_addr + EM_BASE_ADDR), len);
#else
	reg_vm_rw(__func__, em_addr + EM_BASE_ADDR, (uint32_t)sys_addr, len | REG_VM_VP);
#endif
}
/// Write bytes to EM
__INLINE void em_wr(void const *sys_addr, uint16_t em_addr, uint16_t len)
{
#ifndef WIN32
	memcpy((void *)(em_addr + EM_BASE_ADDR), sys_addr, len);
#else
	reg_vm_rw(__func__, em_addr + EM_BASE_ADDR, (uint32_t)sys_addr, len | REG_VM_W | REG_VM_VP);
#endif
}

// copy two exchange memory area
__INLINE void em_cpy(uint16_t dst_em_addr, uint16_t src_em_addr, uint16_t len)
{
#ifndef WIN32
	memcpy((void *)(dst_em_addr + EM_BASE_ADDR), (void *)(src_em_addr + EM_BASE_ADDR), len);
#else
	reg_vm_rw(__func__, dst_em_addr + EM_BASE_ADDR, src_em_addr + EM_BASE_ADDR, len | REG_VM_W | REG_VM_VP);
#endif
}


/// Fill an EM space with the same value
__INLINE void em_set(int value, uint16_t em_addr, uint16_t len)
{
#ifndef WIN32
	memset((void *)(em_addr + EM_BASE_ADDR), value, len);
#else
	for (int i = 0; i < len; i++)
		reg_vm_rw(__func__, em_addr + EM_BASE_ADDR + i, value, 1 | REG_VM_W);
#endif
}

/// Read 24-bits value from EM
__INLINE uint32_t em_rd24p(uint16_t em_addr)
{
#ifndef WIN32
	return co_read24p((void *)(em_addr + EM_BASE_ADDR));
#else
	return reg_vm_rw(__func__, em_addr + EM_BASE_ADDR, 0, 3);
#endif
}
/// Write 24-bits value to EM
__INLINE void em_wr24p(uint16_t em_addr, uint32_t value)
{
#ifndef WIN32
	co_write24p((void *)(em_addr + EM_BASE_ADDR), value);
#else
	reg_vm_rw(__func__, em_addr + EM_BASE_ADDR, value, 3 | REG_VM_W);
#endif
}

/// Read 32-bits value from EM
__INLINE uint32_t em_rd32p(uint16_t em_addr)
{
#ifndef WIN32
	return co_read32p((void *)(em_addr + EM_BASE_ADDR));
#else
	return reg_vm_rw(__func__, em_addr + EM_BASE_ADDR, 0, 4);
#endif
}
/// Write 32-bits value to EM
__INLINE void em_wr32p(uint16_t em_addr, uint32_t value)
{
#ifndef WIN32
	co_write32p((void *)(em_addr + EM_BASE_ADDR), value);
#else
	reg_vm_rw(__func__, em_addr + EM_BASE_ADDR, value, 4 | REG_VM_W);
#endif
}

/// Read 16-bits value from EM
__INLINE uint16_t em_rd16p(uint16_t em_addr)
{
#ifndef WIN32
	return co_read16p((void *)(em_addr + EM_BASE_ADDR));
#else
	return reg_vm_rw(__func__, em_addr + EM_BASE_ADDR, 0, 2);
#endif
}
/// Write 16-bits value to EM
__INLINE void em_wr16p(uint16_t em_addr, uint16_t value)
{
#ifndef WIN32
	co_write16p((void *)(em_addr + EM_BASE_ADDR), value);
#else
	reg_vm_rw(__func__, em_addr + EM_BASE_ADDR, value, 2 | REG_VM_W);
#endif
}

/// Read 8-bits value from EM
__INLINE uint16_t em_rd8p(uint16_t em_addr)
{
#ifndef WIN32
	return *((uint8_t *)(em_addr + EM_BASE_ADDR));
#else
	return reg_vm_rw(__func__, em_addr + EM_BASE_ADDR, 0, 1);
#endif
}
/// Write 8-bits value to EM
__INLINE void em_wr8p(uint16_t em_addr, uint8_t value)
{
#ifndef WIN32
	*(uint8_t *)(em_addr + EM_BASE_ADDR) = value;
#else
	reg_vm_rw(__func__, em_addr + EM_BASE_ADDR, value, 1 | REG_VM_W);
#endif
}


#endif // (defined(CFG_BT_EMB) || defined(CFG_BLE_EMB))

/// @} REG

#endif // REG_ACCESS_H_
