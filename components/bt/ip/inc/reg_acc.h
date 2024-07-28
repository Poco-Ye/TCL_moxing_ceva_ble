#ifndef _REG_ACC_H_
#define _REG_ACC_H_

#include <stdint.h>
#include "_reg_acc.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_ACC_COUNT 31

#define REG_ACC_DECODING_MASK 0x0000001F

/**
 * @brief XOUTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00               XOUT_L   0x0
 * </pre>
 */
#define ACC_XOUTL_ADDR   0x00000000
#define ACC_XOUTL_OFFSET 0x00000000
#define ACC_XOUTL_INDEX  0x00000000
#define ACC_XOUTL_RESET  0x00000000

__INLINE uint8_t acc_xoutl_get(void)
{
    return REG_ACC_RD(ACC_XOUTL_ADDR);
}

// field definitions
#define ACC_XOUT_L_MASK   ((uint8_t)0x000000FF)
#define ACC_XOUT_L_LSB    0
#define ACC_XOUT_L_WIDTH  ((uint8_t)0x00000008)

#define ACC_XOUT_L_RST    0x0

__INLINE uint8_t acc_xoutl_xout_l_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_XOUTL_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief XOUTM register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  01:00               XOUT_M   0x0
 * </pre>
 */
#define ACC_XOUTM_ADDR   0x00000001
#define ACC_XOUTM_OFFSET 0x00000001
#define ACC_XOUTM_INDEX  0x00000001
#define ACC_XOUTM_RESET  0x00000000

__INLINE uint8_t acc_xoutm_get(void)
{
    return REG_ACC_RD(ACC_XOUTM_ADDR);
}

// field definitions
#define ACC_XOUT_M_MASK   ((uint8_t)0x00000003)
#define ACC_XOUT_M_LSB    0
#define ACC_XOUT_M_WIDTH  ((uint8_t)0x00000002)

#define ACC_XOUT_M_RST    0x0

__INLINE uint8_t acc_xoutm_xout_m_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_XOUTM_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x00000003)) == 0);
    return (localVal >> 0);
}

/**
 * @brief YOUTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00               YOUT_L   0x0
 * </pre>
 */
#define ACC_YOUTL_ADDR   0x00000002
#define ACC_YOUTL_OFFSET 0x00000002
#define ACC_YOUTL_INDEX  0x00000002
#define ACC_YOUTL_RESET  0x00000000

__INLINE uint8_t acc_youtl_get(void)
{
    return REG_ACC_RD(ACC_YOUTL_ADDR);
}

// field definitions
#define ACC_YOUT_L_MASK   ((uint8_t)0x000000FF)
#define ACC_YOUT_L_LSB    0
#define ACC_YOUT_L_WIDTH  ((uint8_t)0x00000008)

#define ACC_YOUT_L_RST    0x0

__INLINE uint8_t acc_youtl_yout_l_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_YOUTL_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief YOUTM register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  01:00               YOUT_M   0x0
 * </pre>
 */
#define ACC_YOUTM_ADDR   0x00000003
#define ACC_YOUTM_OFFSET 0x00000003
#define ACC_YOUTM_INDEX  0x00000003
#define ACC_YOUTM_RESET  0x00000000

__INLINE uint8_t acc_youtm_get(void)
{
    return REG_ACC_RD(ACC_YOUTM_ADDR);
}

// field definitions
#define ACC_YOUT_M_MASK   ((uint8_t)0x00000003)
#define ACC_YOUT_M_LSB    0
#define ACC_YOUT_M_WIDTH  ((uint8_t)0x00000002)

#define ACC_YOUT_M_RST    0x0

__INLINE uint8_t acc_youtm_yout_m_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_YOUTM_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x00000003)) == 0);
    return (localVal >> 0);
}

/**
 * @brief ZOUTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00               ZOUT_L   0x0
 * </pre>
 */
#define ACC_ZOUTL_ADDR   0x00000004
#define ACC_ZOUTL_OFFSET 0x00000004
#define ACC_ZOUTL_INDEX  0x00000004
#define ACC_ZOUTL_RESET  0x00000000

__INLINE uint8_t acc_zoutl_get(void)
{
    return REG_ACC_RD(ACC_ZOUTL_ADDR);
}

// field definitions
#define ACC_ZOUT_L_MASK   ((uint8_t)0x000000FF)
#define ACC_ZOUT_L_LSB    0
#define ACC_ZOUT_L_WIDTH  ((uint8_t)0x00000008)

#define ACC_ZOUT_L_RST    0x0

__INLINE uint8_t acc_zoutl_zout_l_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_ZOUTL_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief ZOUTM register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  01:00               ZOUT_M   0x0
 * </pre>
 */
#define ACC_ZOUTM_ADDR   0x00000005
#define ACC_ZOUTM_OFFSET 0x00000005
#define ACC_ZOUTM_INDEX  0x00000005
#define ACC_ZOUTM_RESET  0x00000000

__INLINE uint8_t acc_zoutm_get(void)
{
    return REG_ACC_RD(ACC_ZOUTM_ADDR);
}

// field definitions
#define ACC_ZOUT_M_MASK   ((uint8_t)0x00000003)
#define ACC_ZOUT_M_LSB    0
#define ACC_ZOUT_M_WIDTH  ((uint8_t)0x00000002)

#define ACC_ZOUT_M_RST    0x0

__INLINE uint8_t acc_zoutm_zout_m_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_ZOUTM_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x00000003)) == 0);
    return (localVal >> 0);
}

/**
 * @brief XOUT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00                 XOUT   0x0
 * </pre>
 */
#define ACC_XOUT_ADDR   0x00000006
#define ACC_XOUT_OFFSET 0x00000006
#define ACC_XOUT_INDEX  0x00000006
#define ACC_XOUT_RESET  0x00000000

__INLINE uint8_t acc_xout_get(void)
{
    return REG_ACC_RD(ACC_XOUT_ADDR);
}

// field definitions
#define ACC_XOUT_MASK   ((uint8_t)0x000000FF)
#define ACC_XOUT_LSB    0
#define ACC_XOUT_WIDTH  ((uint8_t)0x00000008)

#define ACC_XOUT_RST    0x0

__INLINE uint8_t acc_xout_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_XOUT_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief YOUT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00                 YOUT   0x0
 * </pre>
 */
#define ACC_YOUT_ADDR   0x00000007
#define ACC_YOUT_OFFSET 0x00000007
#define ACC_YOUT_INDEX  0x00000007
#define ACC_YOUT_RESET  0x00000000

__INLINE uint8_t acc_yout_get(void)
{
    return REG_ACC_RD(ACC_YOUT_ADDR);
}

// field definitions
#define ACC_YOUT_MASK   ((uint8_t)0x000000FF)
#define ACC_YOUT_LSB    0
#define ACC_YOUT_WIDTH  ((uint8_t)0x00000008)

#define ACC_YOUT_RST    0x0

__INLINE uint8_t acc_yout_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_YOUT_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief ZOUT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00                 ZOUT   0x0
 * </pre>
 */
#define ACC_ZOUT_ADDR   0x00000008
#define ACC_ZOUT_OFFSET 0x00000008
#define ACC_ZOUT_INDEX  0x00000008
#define ACC_ZOUT_RESET  0x00000000

__INLINE uint8_t acc_zout_get(void)
{
    return REG_ACC_RD(ACC_ZOUT_ADDR);
}

// field definitions
#define ACC_ZOUT_MASK   ((uint8_t)0x000000FF)
#define ACC_ZOUT_LSB    0
#define ACC_ZOUT_WIDTH  ((uint8_t)0x00000008)

#define ACC_ZOUT_RST    0x0

__INLINE uint8_t acc_zout_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_ZOUT_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief STATUS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     02                 PERR   0
 *     01                 DOVR   0
 *     00                 DRDY   0
 * </pre>
 */
#define ACC_STATUS_ADDR   0x00000009
#define ACC_STATUS_OFFSET 0x00000009
#define ACC_STATUS_INDEX  0x00000009
#define ACC_STATUS_RESET  0x00000000

__INLINE uint8_t acc_status_get(void)
{
    return REG_ACC_RD(ACC_STATUS_ADDR);
}

// field definitions
#define ACC_PERR_BIT    ((uint8_t)0x00000004)
#define ACC_PERR_POS    2
#define ACC_DOVR_BIT    ((uint8_t)0x00000002)
#define ACC_DOVR_POS    1
#define ACC_DRDY_BIT    ((uint8_t)0x00000001)
#define ACC_DRDY_POS    0

#define ACC_PERR_RST    0x0
#define ACC_DOVR_RST    0x0
#define ACC_DRDY_RST    0x0

__INLINE void acc_status_unpack(uint8_t* perr, uint8_t* dovr, uint8_t* drdy)
{
    uint8_t localVal = REG_ACC_RD(ACC_STATUS_ADDR);

    *perr = (localVal & ((uint8_t)0x00000004)) >> 2;
    *dovr = (localVal & ((uint8_t)0x00000002)) >> 1;
    *drdy = (localVal & ((uint8_t)0x00000001)) >> 0;
}

__INLINE uint8_t acc_status_perr_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_STATUS_ADDR);
    return ((localVal & ((uint8_t)0x00000004)) >> 2);
}

__INLINE uint8_t acc_status_dovr_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_STATUS_ADDR);
    return ((localVal & ((uint8_t)0x00000002)) >> 1);
}

__INLINE uint8_t acc_status_drdy_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_STATUS_ADDR);
    return ((localVal & ((uint8_t)0x00000001)) >> 0);
}

/**
 * @brief DETECT_SRC register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     07                  LDX   0
 *     06                  LDY   0
 *     05                  LDZ   0
 *     04                  PDX   0
 *     03                  PDY   0
 *     02                  PDZ   0
 *     01                 INT2   0
 *     00                 INT1   0
 * </pre>
 */
#define ACC_DETECT_SRC_ADDR   0x0000000A
#define ACC_DETECT_SRC_OFFSET 0x0000000A
#define ACC_DETECT_SRC_INDEX  0x0000000A
#define ACC_DETECT_SRC_RESET  0x00000000

__INLINE uint8_t acc_detect_src_get(void)
{
    return REG_ACC_RD(ACC_DETECT_SRC_ADDR);
}

// field definitions
#define ACC_LDX_BIT     ((uint8_t)0x00000080)
#define ACC_LDX_POS     7
#define ACC_LDY_BIT     ((uint8_t)0x00000040)
#define ACC_LDY_POS     6
#define ACC_LDZ_BIT     ((uint8_t)0x00000020)
#define ACC_LDZ_POS     5
#define ACC_PDX_BIT     ((uint8_t)0x00000010)
#define ACC_PDX_POS     4
#define ACC_PDY_BIT     ((uint8_t)0x00000008)
#define ACC_PDY_POS     3
#define ACC_PDZ_BIT     ((uint8_t)0x00000004)
#define ACC_PDZ_POS     2
#define ACC_INT2_BIT    ((uint8_t)0x00000002)
#define ACC_INT2_POS    1
#define ACC_INT1_BIT    ((uint8_t)0x00000001)
#define ACC_INT1_POS    0

#define ACC_LDX_RST     0x0
#define ACC_LDY_RST     0x0
#define ACC_LDZ_RST     0x0
#define ACC_PDX_RST     0x0
#define ACC_PDY_RST     0x0
#define ACC_PDZ_RST     0x0
#define ACC_INT2_RST    0x0
#define ACC_INT1_RST    0x0

__INLINE void acc_detect_src_unpack(uint8_t* ldx, uint8_t* ldy, uint8_t* ldz, uint8_t* pdx, uint8_t* pdy, uint8_t* pdz, uint8_t* int2, uint8_t* int1)
{
    uint8_t localVal = REG_ACC_RD(ACC_DETECT_SRC_ADDR);

    *ldx = (localVal & ((uint8_t)0x00000080)) >> 7;
    *ldy = (localVal & ((uint8_t)0x00000040)) >> 6;
    *ldz = (localVal & ((uint8_t)0x00000020)) >> 5;
    *pdx = (localVal & ((uint8_t)0x00000010)) >> 4;
    *pdy = (localVal & ((uint8_t)0x00000008)) >> 3;
    *pdz = (localVal & ((uint8_t)0x00000004)) >> 2;
    *int2 = (localVal & ((uint8_t)0x00000002)) >> 1;
    *int1 = (localVal & ((uint8_t)0x00000001)) >> 0;
}

__INLINE uint8_t acc_detect_src_ldx_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_DETECT_SRC_ADDR);
    return ((localVal & ((uint8_t)0x00000080)) >> 7);
}

__INLINE uint8_t acc_detect_src_ldy_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_DETECT_SRC_ADDR);
    return ((localVal & ((uint8_t)0x00000040)) >> 6);
}

__INLINE uint8_t acc_detect_src_ldz_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_DETECT_SRC_ADDR);
    return ((localVal & ((uint8_t)0x00000020)) >> 5);
}

__INLINE uint8_t acc_detect_src_pdx_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_DETECT_SRC_ADDR);
    return ((localVal & ((uint8_t)0x00000010)) >> 4);
}

__INLINE uint8_t acc_detect_src_pdy_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_DETECT_SRC_ADDR);
    return ((localVal & ((uint8_t)0x00000008)) >> 3);
}

__INLINE uint8_t acc_detect_src_pdz_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_DETECT_SRC_ADDR);
    return ((localVal & ((uint8_t)0x00000004)) >> 2);
}

__INLINE uint8_t acc_detect_src_int2_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_DETECT_SRC_ADDR);
    return ((localVal & ((uint8_t)0x00000002)) >> 1);
}

__INLINE uint8_t acc_detect_src_int1_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_DETECT_SRC_ADDR);
    return ((localVal & ((uint8_t)0x00000001)) >> 0);
}

/**
 * @brief I2C_ADDR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     07               I2CDIS   0
 *  06:00                 DVAD   0x1D
 * </pre>
 */
#define ACC_I2C_ADDR_ADDR   0x0000000D
#define ACC_I2C_ADDR_OFFSET 0x0000000D
#define ACC_I2C_ADDR_INDEX  0x0000000D
#define ACC_I2C_ADDR_RESET  0x0000001D

__INLINE uint8_t acc_i2c_addr_get(void)
{
    return REG_ACC_RD(ACC_I2C_ADDR_ADDR);
}

__INLINE void acc_i2c_addr_set(uint8_t value)
{
    REG_ACC_WR(ACC_I2C_ADDR_ADDR, value);
}

// field definitions
#define ACC_I2CDIS_BIT    ((uint8_t)0x00000080)
#define ACC_I2CDIS_POS    7
#define ACC_DVAD_MASK     ((uint8_t)0x0000007F)
#define ACC_DVAD_LSB      0
#define ACC_DVAD_WIDTH    ((uint8_t)0x00000007)

#define ACC_I2CDIS_RST    0x0
#define ACC_DVAD_RST      0x1D

__INLINE void acc_i2c_addr_unpack(uint8_t* i2cdis, uint8_t* dvad)
{
    uint8_t localVal = REG_ACC_RD(ACC_I2C_ADDR_ADDR);

    *i2cdis = (localVal & ((uint8_t)0x00000080)) >> 7;
    *dvad = (localVal & ((uint8_t)0x0000007F)) >> 0;
}

__INLINE uint8_t acc_i2c_addr_i2cdis_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_I2C_ADDR_ADDR);
    return ((localVal & ((uint8_t)0x00000080)) >> 7);
}

__INLINE void acc_i2c_addr_i2cdis_setf(uint8_t i2cdis)
{
    ASSERT_ERR((((uint8_t)i2cdis << 7) & ~((uint8_t)0x00000080)) == 0);
    REG_ACC_WR(ACC_I2C_ADDR_ADDR, (uint8_t)i2cdis << 7);
}

__INLINE uint8_t acc_i2c_addr_dvad_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_I2C_ADDR_ADDR);
    return ((localVal & ((uint8_t)0x0000007F)) >> 0);
}

/**
 * @brief USR_INFO register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00                   UI   0x0
 * </pre>
 */
#define ACC_USR_INFO_ADDR   0x0000000E
#define ACC_USR_INFO_OFFSET 0x0000000E
#define ACC_USR_INFO_INDEX  0x0000000E
#define ACC_USR_INFO_RESET  0x00000000

__INLINE uint8_t acc_usr_info_get(void)
{
    return REG_ACC_RD(ACC_USR_INFO_ADDR);
}

// field definitions
#define ACC_UI_MASK   ((uint8_t)0x000000FF)
#define ACC_UI_LSB    0
#define ACC_UI_WIDTH  ((uint8_t)0x00000008)

#define ACC_UI_RST    0x0

__INLINE uint8_t acc_usr_info_ui_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_USR_INFO_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief ID register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00                   ID   0x0
 * </pre>
 */
#define ACC_ID_ADDR   0x0000000F
#define ACC_ID_OFFSET 0x0000000F
#define ACC_ID_INDEX  0x0000000F
#define ACC_ID_RESET  0x00000000

__INLINE uint8_t acc_id_get(void)
{
    return REG_ACC_RD(ACC_ID_ADDR);
}

// field definitions
#define ACC_ID_MASK   ((uint8_t)0x000000FF)
#define ACC_ID_LSB    0
#define ACC_ID_WIDTH  ((uint8_t)0x00000008)

#define ACC_ID_RST    0x0

__INLINE uint8_t acc_id_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_ID_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief OFFSET_DRIFT_XL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00                XOFFL   0x0
 * </pre>
 */
#define ACC_OFFSET_DRIFT_XL_ADDR   0x00000010
#define ACC_OFFSET_DRIFT_XL_OFFSET 0x00000010
#define ACC_OFFSET_DRIFT_XL_INDEX  0x00000010
#define ACC_OFFSET_DRIFT_XL_RESET  0x00000000

__INLINE uint8_t acc_offset_drift_xl_get(void)
{
    return REG_ACC_RD(ACC_OFFSET_DRIFT_XL_ADDR);
}

__INLINE void acc_offset_drift_xl_set(uint8_t value)
{
    REG_ACC_WR(ACC_OFFSET_DRIFT_XL_ADDR, value);
}

// field definitions
#define ACC_XOFFL_MASK   ((uint8_t)0x000000FF)
#define ACC_XOFFL_LSB    0
#define ACC_XOFFL_WIDTH  ((uint8_t)0x00000008)

#define ACC_XOFFL_RST    0x0

__INLINE uint8_t acc_offset_drift_xl_xoffl_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_OFFSET_DRIFT_XL_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

__INLINE void acc_offset_drift_xl_xoffl_setf(uint8_t xoffl)
{
    ASSERT_ERR((((uint8_t)xoffl << 0) & ~((uint8_t)0x000000FF)) == 0);
    REG_ACC_WR(ACC_OFFSET_DRIFT_XL_ADDR, (uint8_t)xoffl << 0);
}

/**
 * @brief OFFSET_DRIFT_XM register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  02:00                XOFFM   0x0
 * </pre>
 */
#define ACC_OFFSET_DRIFT_XM_ADDR   0x00000011
#define ACC_OFFSET_DRIFT_XM_OFFSET 0x00000011
#define ACC_OFFSET_DRIFT_XM_INDEX  0x00000011
#define ACC_OFFSET_DRIFT_XM_RESET  0x00000000

__INLINE uint8_t acc_offset_drift_xm_get(void)
{
    return REG_ACC_RD(ACC_OFFSET_DRIFT_XM_ADDR);
}

__INLINE void acc_offset_drift_xm_set(uint8_t value)
{
    REG_ACC_WR(ACC_OFFSET_DRIFT_XM_ADDR, value);
}

// field definitions
#define ACC_XOFFM_MASK   ((uint8_t)0x00000007)
#define ACC_XOFFM_LSB    0
#define ACC_XOFFM_WIDTH  ((uint8_t)0x00000003)

#define ACC_XOFFM_RST    0x0

__INLINE uint8_t acc_offset_drift_xm_xoffm_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_OFFSET_DRIFT_XM_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x00000007)) == 0);
    return (localVal >> 0);
}

__INLINE void acc_offset_drift_xm_xoffm_setf(uint8_t xoffm)
{
    ASSERT_ERR((((uint8_t)xoffm << 0) & ~((uint8_t)0x00000007)) == 0);
    REG_ACC_WR(ACC_OFFSET_DRIFT_XM_ADDR, (uint8_t)xoffm << 0);
}

/**
 * @brief OFFSET_DRIFT_YL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00                YOFFL   0x0
 * </pre>
 */
#define ACC_OFFSET_DRIFT_YL_ADDR   0x00000012
#define ACC_OFFSET_DRIFT_YL_OFFSET 0x00000012
#define ACC_OFFSET_DRIFT_YL_INDEX  0x00000012
#define ACC_OFFSET_DRIFT_YL_RESET  0x00000000

__INLINE uint8_t acc_offset_drift_yl_get(void)
{
    return REG_ACC_RD(ACC_OFFSET_DRIFT_YL_ADDR);
}

__INLINE void acc_offset_drift_yl_set(uint8_t value)
{
    REG_ACC_WR(ACC_OFFSET_DRIFT_YL_ADDR, value);
}

// field definitions
#define ACC_YOFFL_MASK   ((uint8_t)0x000000FF)
#define ACC_YOFFL_LSB    0
#define ACC_YOFFL_WIDTH  ((uint8_t)0x00000008)

#define ACC_YOFFL_RST    0x0

__INLINE uint8_t acc_offset_drift_yl_yoffl_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_OFFSET_DRIFT_YL_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

__INLINE void acc_offset_drift_yl_yoffl_setf(uint8_t yoffl)
{
    ASSERT_ERR((((uint8_t)yoffl << 0) & ~((uint8_t)0x000000FF)) == 0);
    REG_ACC_WR(ACC_OFFSET_DRIFT_YL_ADDR, (uint8_t)yoffl << 0);
}

/**
 * @brief OFFSET_DRIFT_YM register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  02:00                YOFFM   0x0
 * </pre>
 */
#define ACC_OFFSET_DRIFT_YM_ADDR   0x00000013
#define ACC_OFFSET_DRIFT_YM_OFFSET 0x00000013
#define ACC_OFFSET_DRIFT_YM_INDEX  0x00000013
#define ACC_OFFSET_DRIFT_YM_RESET  0x00000000

__INLINE uint8_t acc_offset_drift_ym_get(void)
{
    return REG_ACC_RD(ACC_OFFSET_DRIFT_YM_ADDR);
}

__INLINE void acc_offset_drift_ym_set(uint8_t value)
{
    REG_ACC_WR(ACC_OFFSET_DRIFT_YM_ADDR, value);
}

// field definitions
#define ACC_YOFFM_MASK   ((uint8_t)0x00000007)
#define ACC_YOFFM_LSB    0
#define ACC_YOFFM_WIDTH  ((uint8_t)0x00000003)

#define ACC_YOFFM_RST    0x0

__INLINE uint8_t acc_offset_drift_ym_yoffm_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_OFFSET_DRIFT_YM_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x00000007)) == 0);
    return (localVal >> 0);
}

__INLINE void acc_offset_drift_ym_yoffm_setf(uint8_t yoffm)
{
    ASSERT_ERR((((uint8_t)yoffm << 0) & ~((uint8_t)0x00000007)) == 0);
    REG_ACC_WR(ACC_OFFSET_DRIFT_YM_ADDR, (uint8_t)yoffm << 0);
}

/**
 * @brief OFFSET_DRIFT_ZL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00                ZOFFL   0x0
 * </pre>
 */
#define ACC_OFFSET_DRIFT_ZL_ADDR   0x00000014
#define ACC_OFFSET_DRIFT_ZL_OFFSET 0x00000014
#define ACC_OFFSET_DRIFT_ZL_INDEX  0x00000014
#define ACC_OFFSET_DRIFT_ZL_RESET  0x00000000

__INLINE uint8_t acc_offset_drift_zl_get(void)
{
    return REG_ACC_RD(ACC_OFFSET_DRIFT_ZL_ADDR);
}

__INLINE void acc_offset_drift_zl_set(uint8_t value)
{
    REG_ACC_WR(ACC_OFFSET_DRIFT_ZL_ADDR, value);
}

// field definitions
#define ACC_ZOFFL_MASK   ((uint8_t)0x000000FF)
#define ACC_ZOFFL_LSB    0
#define ACC_ZOFFL_WIDTH  ((uint8_t)0x00000008)

#define ACC_ZOFFL_RST    0x0

__INLINE uint8_t acc_offset_drift_zl_zoffl_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_OFFSET_DRIFT_ZL_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

__INLINE void acc_offset_drift_zl_zoffl_setf(uint8_t zoffl)
{
    ASSERT_ERR((((uint8_t)zoffl << 0) & ~((uint8_t)0x000000FF)) == 0);
    REG_ACC_WR(ACC_OFFSET_DRIFT_ZL_ADDR, (uint8_t)zoffl << 0);
}

/**
 * @brief OFFSET_DRIFT_ZM register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  02:00                ZOFFM   0x0
 * </pre>
 */
#define ACC_OFFSET_DRIFT_ZM_ADDR   0x00000015
#define ACC_OFFSET_DRIFT_ZM_OFFSET 0x00000015
#define ACC_OFFSET_DRIFT_ZM_INDEX  0x00000015
#define ACC_OFFSET_DRIFT_ZM_RESET  0x00000000

__INLINE uint8_t acc_offset_drift_zm_get(void)
{
    return REG_ACC_RD(ACC_OFFSET_DRIFT_ZM_ADDR);
}

__INLINE void acc_offset_drift_zm_set(uint8_t value)
{
    REG_ACC_WR(ACC_OFFSET_DRIFT_ZM_ADDR, value);
}

// field definitions
#define ACC_ZOFFM_MASK   ((uint8_t)0x00000007)
#define ACC_ZOFFM_LSB    0
#define ACC_ZOFFM_WIDTH  ((uint8_t)0x00000003)

#define ACC_ZOFFM_RST    0x0

__INLINE uint8_t acc_offset_drift_zm_zoffm_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_OFFSET_DRIFT_ZM_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x00000007)) == 0);
    return (localVal >> 0);
}

__INLINE void acc_offset_drift_zm_zoffm_setf(uint8_t zoffm)
{
    ASSERT_ERR((((uint8_t)zoffm << 0) & ~((uint8_t)0x00000007)) == 0);
    REG_ACC_WR(ACC_OFFSET_DRIFT_ZM_ADDR, (uint8_t)zoffm << 0);
}

/**
 * @brief MODE_CNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     06                 DRPD   0
 *     05                SPI3W   0
 *     04                 STON   0
 *  03:02                 GLVL   0x0
 *  01:00                 MODE   0x0
 * </pre>
 */
#define ACC_MODE_CNTL_ADDR   0x00000016
#define ACC_MODE_CNTL_OFFSET 0x00000016
#define ACC_MODE_CNTL_INDEX  0x00000016
#define ACC_MODE_CNTL_RESET  0x00000000

__INLINE uint8_t acc_mode_cntl_get(void)
{
    return REG_ACC_RD(ACC_MODE_CNTL_ADDR);
}

__INLINE void acc_mode_cntl_set(uint8_t value)
{
    REG_ACC_WR(ACC_MODE_CNTL_ADDR, value);
}

// field definitions
#define ACC_DRPD_BIT     ((uint8_t)0x00000040)
#define ACC_DRPD_POS     6
#define ACC_SPI3W_BIT    ((uint8_t)0x00000020)
#define ACC_SPI3W_POS    5
#define ACC_STON_BIT     ((uint8_t)0x00000010)
#define ACC_STON_POS     4
#define ACC_GLVL_MASK    ((uint8_t)0x0000000C)
#define ACC_GLVL_LSB     2
#define ACC_GLVL_WIDTH   ((uint8_t)0x00000002)
#define ACC_MODE_MASK    ((uint8_t)0x00000003)
#define ACC_MODE_LSB     0
#define ACC_MODE_WIDTH   ((uint8_t)0x00000002)

#define ACC_DRPD_RST     0x0
#define ACC_SPI3W_RST    0x0
#define ACC_STON_RST     0x0
#define ACC_GLVL_RST     0x0
#define ACC_MODE_RST     0x0

__INLINE void acc_mode_cntl_pack(uint8_t drpd, uint8_t spi3w, uint8_t ston, uint8_t glvl, uint8_t mode)
{
    ASSERT_ERR((((uint8_t)drpd << 6) & ~((uint8_t)0x00000040)) == 0);
    ASSERT_ERR((((uint8_t)spi3w << 5) & ~((uint8_t)0x00000020)) == 0);
    ASSERT_ERR((((uint8_t)ston << 4) & ~((uint8_t)0x00000010)) == 0);
    ASSERT_ERR((((uint8_t)glvl << 2) & ~((uint8_t)0x0000000C)) == 0);
    ASSERT_ERR((((uint8_t)mode << 0) & ~((uint8_t)0x00000003)) == 0);
    REG_ACC_WR(ACC_MODE_CNTL_ADDR,  ((uint8_t)drpd << 6) | ((uint8_t)spi3w << 5) | ((uint8_t)ston << 4) | ((uint8_t)glvl << 2) | ((uint8_t)mode << 0));
}

__INLINE void acc_mode_cntl_unpack(uint8_t* drpd, uint8_t* spi3w, uint8_t* ston, uint8_t* glvl, uint8_t* mode)
{
    uint8_t localVal = REG_ACC_RD(ACC_MODE_CNTL_ADDR);

    *drpd = (localVal & ((uint8_t)0x00000040)) >> 6;
    *spi3w = (localVal & ((uint8_t)0x00000020)) >> 5;
    *ston = (localVal & ((uint8_t)0x00000010)) >> 4;
    *glvl = (localVal & ((uint8_t)0x0000000C)) >> 2;
    *mode = (localVal & ((uint8_t)0x00000003)) >> 0;
}

__INLINE uint8_t acc_mode_cntl_drpd_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_MODE_CNTL_ADDR);
    return ((localVal & ((uint8_t)0x00000040)) >> 6);
}

__INLINE void acc_mode_cntl_drpd_setf(uint8_t drpd)
{
    ASSERT_ERR((((uint8_t)drpd << 6) & ~((uint8_t)0x00000040)) == 0);
    REG_ACC_WR(ACC_MODE_CNTL_ADDR, (REG_ACC_RD(ACC_MODE_CNTL_ADDR) & ~((uint8_t)0x00000040)) | ((uint8_t)drpd << 6));
}

__INLINE uint8_t acc_mode_cntl_spi3w_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_MODE_CNTL_ADDR);
    return ((localVal & ((uint8_t)0x00000020)) >> 5);
}

__INLINE void acc_mode_cntl_spi3w_setf(uint8_t spi3w)
{
    ASSERT_ERR((((uint8_t)spi3w << 5) & ~((uint8_t)0x00000020)) == 0);
    REG_ACC_WR(ACC_MODE_CNTL_ADDR, (REG_ACC_RD(ACC_MODE_CNTL_ADDR) & ~((uint8_t)0x00000020)) | ((uint8_t)spi3w << 5));
}

__INLINE uint8_t acc_mode_cntl_ston_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_MODE_CNTL_ADDR);
    return ((localVal & ((uint8_t)0x00000010)) >> 4);
}

__INLINE void acc_mode_cntl_ston_setf(uint8_t ston)
{
    ASSERT_ERR((((uint8_t)ston << 4) & ~((uint8_t)0x00000010)) == 0);
    REG_ACC_WR(ACC_MODE_CNTL_ADDR, (REG_ACC_RD(ACC_MODE_CNTL_ADDR) & ~((uint8_t)0x00000010)) | ((uint8_t)ston << 4));
}

__INLINE uint8_t acc_mode_cntl_glvl_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_MODE_CNTL_ADDR);
    return ((localVal & ((uint8_t)0x0000000C)) >> 2);
}

__INLINE void acc_mode_cntl_glvl_setf(uint8_t glvl)
{
    ASSERT_ERR((((uint8_t)glvl << 2) & ~((uint8_t)0x0000000C)) == 0);
    REG_ACC_WR(ACC_MODE_CNTL_ADDR, (REG_ACC_RD(ACC_MODE_CNTL_ADDR) & ~((uint8_t)0x0000000C)) | ((uint8_t)glvl << 2));
}

__INLINE uint8_t acc_mode_cntl_mode_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_MODE_CNTL_ADDR);
    return ((localVal & ((uint8_t)0x00000003)) >> 0);
}

__INLINE void acc_mode_cntl_mode_setf(uint8_t mode)
{
    ASSERT_ERR((((uint8_t)mode << 0) & ~((uint8_t)0x00000003)) == 0);
    REG_ACC_WR(ACC_MODE_CNTL_ADDR, (REG_ACC_RD(ACC_MODE_CNTL_ADDR) & ~((uint8_t)0x00000003)) | ((uint8_t)mode << 0));
}

/**
 * @brief INTRST register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     01             CLR_INT2   0
 *     00             CLR_INT1   0
 * </pre>
 */
#define ACC_INTRST_ADDR   0x00000017
#define ACC_INTRST_OFFSET 0x00000017
#define ACC_INTRST_INDEX  0x00000017
#define ACC_INTRST_RESET  0x00000000

__INLINE uint8_t acc_intrst_get(void)
{
    return REG_ACC_RD(ACC_INTRST_ADDR);
}

__INLINE void acc_intrst_set(uint8_t value)
{
    REG_ACC_WR(ACC_INTRST_ADDR, value);
}

// field definitions
#define ACC_CLR_INT2_BIT    ((uint8_t)0x00000002)
#define ACC_CLR_INT2_POS    1
#define ACC_CLR_INT1_BIT    ((uint8_t)0x00000001)
#define ACC_CLR_INT1_POS    0

#define ACC_CLR_INT2_RST    0x0
#define ACC_CLR_INT1_RST    0x0

__INLINE void acc_intrst_pack(uint8_t clrint2, uint8_t clrint1)
{
    ASSERT_ERR((((uint8_t)clrint2 << 1) & ~((uint8_t)0x00000002)) == 0);
    ASSERT_ERR((((uint8_t)clrint1 << 0) & ~((uint8_t)0x00000001)) == 0);
    REG_ACC_WR(ACC_INTRST_ADDR,  ((uint8_t)clrint2 << 1) | ((uint8_t)clrint1 << 0));
}

__INLINE void acc_intrst_unpack(uint8_t* clrint2, uint8_t* clrint1)
{
    uint8_t localVal = REG_ACC_RD(ACC_INTRST_ADDR);

    *clrint2 = (localVal & ((uint8_t)0x00000002)) >> 1;
    *clrint1 = (localVal & ((uint8_t)0x00000001)) >> 0;
}

__INLINE uint8_t acc_intrst_clr_int2_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_INTRST_ADDR);
    return ((localVal & ((uint8_t)0x00000002)) >> 1);
}

__INLINE void acc_intrst_clr_int2_setf(uint8_t clrint2)
{
    ASSERT_ERR((((uint8_t)clrint2 << 1) & ~((uint8_t)0x00000002)) == 0);
    REG_ACC_WR(ACC_INTRST_ADDR, (REG_ACC_RD(ACC_INTRST_ADDR) & ~((uint8_t)0x00000002)) | ((uint8_t)clrint2 << 1));
}

__INLINE uint8_t acc_intrst_clr_int1_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_INTRST_ADDR);
    return ((localVal & ((uint8_t)0x00000001)) >> 0);
}

__INLINE void acc_intrst_clr_int1_setf(uint8_t clrint1)
{
    ASSERT_ERR((((uint8_t)clrint1 << 0) & ~((uint8_t)0x00000001)) == 0);
    REG_ACC_WR(ACC_INTRST_ADDR, (REG_ACC_RD(ACC_INTRST_ADDR) & ~((uint8_t)0x00000001)) | ((uint8_t)clrint1 << 0));
}

/**
 * @brief CNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     07                 DFBW   0
 *     06                THOPT   0
 *     05                  ZDA   0
 *     04                  YDA   0
 *     03                  XDA   0
 *  02:01               INTREG   0x0
 *     00               INTPIN   0
 * </pre>
 */
#define ACC_CNTL1_ADDR   0x00000018
#define ACC_CNTL1_OFFSET 0x00000018
#define ACC_CNTL1_INDEX  0x00000018
#define ACC_CNTL1_RESET  0x00000000

__INLINE uint8_t acc_cntl1_get(void)
{
    return REG_ACC_RD(ACC_CNTL1_ADDR);
}

__INLINE void acc_cntl1_set(uint8_t value)
{
    REG_ACC_WR(ACC_CNTL1_ADDR, value);
}

// field definitions
#define ACC_DFBW_BIT      ((uint8_t)0x00000080)
#define ACC_DFBW_POS      7
#define ACC_THOPT_BIT     ((uint8_t)0x00000040)
#define ACC_THOPT_POS     6
#define ACC_ZDA_BIT       ((uint8_t)0x00000020)
#define ACC_ZDA_POS       5
#define ACC_YDA_BIT       ((uint8_t)0x00000010)
#define ACC_YDA_POS       4
#define ACC_XDA_BIT       ((uint8_t)0x00000008)
#define ACC_XDA_POS       3
#define ACC_INTREG_MASK   ((uint8_t)0x00000006)
#define ACC_INTREG_LSB    1
#define ACC_INTREG_WIDTH  ((uint8_t)0x00000002)
#define ACC_INTPIN_BIT    ((uint8_t)0x00000001)
#define ACC_INTPIN_POS    0

#define ACC_DFBW_RST      0x0
#define ACC_THOPT_RST     0x0
#define ACC_ZDA_RST       0x0
#define ACC_YDA_RST       0x0
#define ACC_XDA_RST       0x0
#define ACC_INTREG_RST    0x0
#define ACC_INTPIN_RST    0x0

__INLINE void acc_cntl1_pack(uint8_t dfbw, uint8_t thopt, uint8_t zda, uint8_t yda, uint8_t xda, uint8_t intreg, uint8_t intpin)
{
    ASSERT_ERR((((uint8_t)dfbw << 7) & ~((uint8_t)0x00000080)) == 0);
    ASSERT_ERR((((uint8_t)thopt << 6) & ~((uint8_t)0x00000040)) == 0);
    ASSERT_ERR((((uint8_t)zda << 5) & ~((uint8_t)0x00000020)) == 0);
    ASSERT_ERR((((uint8_t)yda << 4) & ~((uint8_t)0x00000010)) == 0);
    ASSERT_ERR((((uint8_t)xda << 3) & ~((uint8_t)0x00000008)) == 0);
    ASSERT_ERR((((uint8_t)intreg << 1) & ~((uint8_t)0x00000006)) == 0);
    ASSERT_ERR((((uint8_t)intpin << 0) & ~((uint8_t)0x00000001)) == 0);
    REG_ACC_WR(ACC_CNTL1_ADDR,  ((uint8_t)dfbw << 7) | ((uint8_t)thopt << 6) | ((uint8_t)zda << 5) | ((uint8_t)yda << 4) | ((uint8_t)xda << 3) | ((uint8_t)intreg << 1) | ((uint8_t)intpin << 0));
}

__INLINE void acc_cntl1_unpack(uint8_t* dfbw, uint8_t* thopt, uint8_t* zda, uint8_t* yda, uint8_t* xda, uint8_t* intreg, uint8_t* intpin)
{
    uint8_t localVal = REG_ACC_RD(ACC_CNTL1_ADDR);

    *dfbw = (localVal & ((uint8_t)0x00000080)) >> 7;
    *thopt = (localVal & ((uint8_t)0x00000040)) >> 6;
    *zda = (localVal & ((uint8_t)0x00000020)) >> 5;
    *yda = (localVal & ((uint8_t)0x00000010)) >> 4;
    *xda = (localVal & ((uint8_t)0x00000008)) >> 3;
    *intreg = (localVal & ((uint8_t)0x00000006)) >> 1;
    *intpin = (localVal & ((uint8_t)0x00000001)) >> 0;
}

__INLINE uint8_t acc_cntl1_dfbw_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_CNTL1_ADDR);
    return ((localVal & ((uint8_t)0x00000080)) >> 7);
}

__INLINE void acc_cntl1_dfbw_setf(uint8_t dfbw)
{
    ASSERT_ERR((((uint8_t)dfbw << 7) & ~((uint8_t)0x00000080)) == 0);
    REG_ACC_WR(ACC_CNTL1_ADDR, (REG_ACC_RD(ACC_CNTL1_ADDR) & ~((uint8_t)0x00000080)) | ((uint8_t)dfbw << 7));
}

__INLINE uint8_t acc_cntl1_thopt_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_CNTL1_ADDR);
    return ((localVal & ((uint8_t)0x00000040)) >> 6);
}

__INLINE void acc_cntl1_thopt_setf(uint8_t thopt)
{
    ASSERT_ERR((((uint8_t)thopt << 6) & ~((uint8_t)0x00000040)) == 0);
    REG_ACC_WR(ACC_CNTL1_ADDR, (REG_ACC_RD(ACC_CNTL1_ADDR) & ~((uint8_t)0x00000040)) | ((uint8_t)thopt << 6));
}

__INLINE uint8_t acc_cntl1_zda_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_CNTL1_ADDR);
    return ((localVal & ((uint8_t)0x00000020)) >> 5);
}

__INLINE void acc_cntl1_zda_setf(uint8_t zda)
{
    ASSERT_ERR((((uint8_t)zda << 5) & ~((uint8_t)0x00000020)) == 0);
    REG_ACC_WR(ACC_CNTL1_ADDR, (REG_ACC_RD(ACC_CNTL1_ADDR) & ~((uint8_t)0x00000020)) | ((uint8_t)zda << 5));
}

__INLINE uint8_t acc_cntl1_yda_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_CNTL1_ADDR);
    return ((localVal & ((uint8_t)0x00000010)) >> 4);
}

__INLINE void acc_cntl1_yda_setf(uint8_t yda)
{
    ASSERT_ERR((((uint8_t)yda << 4) & ~((uint8_t)0x00000010)) == 0);
    REG_ACC_WR(ACC_CNTL1_ADDR, (REG_ACC_RD(ACC_CNTL1_ADDR) & ~((uint8_t)0x00000010)) | ((uint8_t)yda << 4));
}

__INLINE uint8_t acc_cntl1_xda_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_CNTL1_ADDR);
    return ((localVal & ((uint8_t)0x00000008)) >> 3);
}

__INLINE void acc_cntl1_xda_setf(uint8_t xda)
{
    ASSERT_ERR((((uint8_t)xda << 3) & ~((uint8_t)0x00000008)) == 0);
    REG_ACC_WR(ACC_CNTL1_ADDR, (REG_ACC_RD(ACC_CNTL1_ADDR) & ~((uint8_t)0x00000008)) | ((uint8_t)xda << 3));
}

__INLINE uint8_t acc_cntl1_intreg_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_CNTL1_ADDR);
    return ((localVal & ((uint8_t)0x00000006)) >> 1);
}

__INLINE void acc_cntl1_intreg_setf(uint8_t intreg)
{
    ASSERT_ERR((((uint8_t)intreg << 1) & ~((uint8_t)0x00000006)) == 0);
    REG_ACC_WR(ACC_CNTL1_ADDR, (REG_ACC_RD(ACC_CNTL1_ADDR) & ~((uint8_t)0x00000006)) | ((uint8_t)intreg << 1));
}

__INLINE uint8_t acc_cntl1_intpin_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_CNTL1_ADDR);
    return ((localVal & ((uint8_t)0x00000001)) >> 0);
}

__INLINE void acc_cntl1_intpin_setf(uint8_t intpin)
{
    ASSERT_ERR((((uint8_t)intpin << 0) & ~((uint8_t)0x00000001)) == 0);
    REG_ACC_WR(ACC_CNTL1_ADDR, (REG_ACC_RD(ACC_CNTL1_ADDR) & ~((uint8_t)0x00000001)) | ((uint8_t)intpin << 0));
}

/**
 * @brief CNTL2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     02                 DRVO   0
 *     01                 PDPL   0
 *     00                 LDPL   0
 * </pre>
 */
#define ACC_CNTL2_ADDR   0x00000019
#define ACC_CNTL2_OFFSET 0x00000019
#define ACC_CNTL2_INDEX  0x00000019
#define ACC_CNTL2_RESET  0x00000000

__INLINE uint8_t acc_cntl2_get(void)
{
    return REG_ACC_RD(ACC_CNTL2_ADDR);
}

__INLINE void acc_cntl2_set(uint8_t value)
{
    REG_ACC_WR(ACC_CNTL2_ADDR, value);
}

// field definitions
#define ACC_DRVO_BIT    ((uint8_t)0x00000004)
#define ACC_DRVO_POS    2
#define ACC_PDPL_BIT    ((uint8_t)0x00000002)
#define ACC_PDPL_POS    1
#define ACC_LDPL_BIT    ((uint8_t)0x00000001)
#define ACC_LDPL_POS    0

#define ACC_DRVO_RST    0x0
#define ACC_PDPL_RST    0x0
#define ACC_LDPL_RST    0x0

__INLINE void acc_cntl2_pack(uint8_t drvo, uint8_t pdpl, uint8_t ldpl)
{
    ASSERT_ERR((((uint8_t)drvo << 2) & ~((uint8_t)0x00000004)) == 0);
    ASSERT_ERR((((uint8_t)pdpl << 1) & ~((uint8_t)0x00000002)) == 0);
    ASSERT_ERR((((uint8_t)ldpl << 0) & ~((uint8_t)0x00000001)) == 0);
    REG_ACC_WR(ACC_CNTL2_ADDR,  ((uint8_t)drvo << 2) | ((uint8_t)pdpl << 1) | ((uint8_t)ldpl << 0));
}

__INLINE void acc_cntl2_unpack(uint8_t* drvo, uint8_t* pdpl, uint8_t* ldpl)
{
    uint8_t localVal = REG_ACC_RD(ACC_CNTL2_ADDR);

    *drvo = (localVal & ((uint8_t)0x00000004)) >> 2;
    *pdpl = (localVal & ((uint8_t)0x00000002)) >> 1;
    *ldpl = (localVal & ((uint8_t)0x00000001)) >> 0;
}

__INLINE uint8_t acc_cntl2_drvo_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_CNTL2_ADDR);
    return ((localVal & ((uint8_t)0x00000004)) >> 2);
}

__INLINE void acc_cntl2_drvo_setf(uint8_t drvo)
{
    ASSERT_ERR((((uint8_t)drvo << 2) & ~((uint8_t)0x00000004)) == 0);
    REG_ACC_WR(ACC_CNTL2_ADDR, (REG_ACC_RD(ACC_CNTL2_ADDR) & ~((uint8_t)0x00000004)) | ((uint8_t)drvo << 2));
}

__INLINE uint8_t acc_cntl2_pdpl_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_CNTL2_ADDR);
    return ((localVal & ((uint8_t)0x00000002)) >> 1);
}

__INLINE void acc_cntl2_pdpl_setf(uint8_t pdpl)
{
    ASSERT_ERR((((uint8_t)pdpl << 1) & ~((uint8_t)0x00000002)) == 0);
    REG_ACC_WR(ACC_CNTL2_ADDR, (REG_ACC_RD(ACC_CNTL2_ADDR) & ~((uint8_t)0x00000002)) | ((uint8_t)pdpl << 1));
}

__INLINE uint8_t acc_cntl2_ldpl_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_CNTL2_ADDR);
    return ((localVal & ((uint8_t)0x00000001)) >> 0);
}

__INLINE void acc_cntl2_ldpl_setf(uint8_t ldpl)
{
    ASSERT_ERR((((uint8_t)ldpl << 0) & ~((uint8_t)0x00000001)) == 0);
    REG_ACC_WR(ACC_CNTL2_ADDR, (REG_ACC_RD(ACC_CNTL2_ADDR) & ~((uint8_t)0x00000001)) | ((uint8_t)ldpl << 0));
}

/**
 * @brief LVL_DETECT_TH register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00                 LDTH   0x0
 * </pre>
 */
#define ACC_LVL_DETECT_TH_ADDR   0x0000001A
#define ACC_LVL_DETECT_TH_OFFSET 0x0000001A
#define ACC_LVL_DETECT_TH_INDEX  0x0000001A
#define ACC_LVL_DETECT_TH_RESET  0x00000000

__INLINE uint8_t acc_lvl_detect_th_get(void)
{
    return REG_ACC_RD(ACC_LVL_DETECT_TH_ADDR);
}

__INLINE void acc_lvl_detect_th_set(uint8_t value)
{
    REG_ACC_WR(ACC_LVL_DETECT_TH_ADDR, value);
}

// field definitions
#define ACC_LDTH_MASK   ((uint8_t)0x000000FF)
#define ACC_LDTH_LSB    0
#define ACC_LDTH_WIDTH  ((uint8_t)0x00000008)

#define ACC_LDTH_RST    0x0

__INLINE uint8_t acc_lvl_detect_th_ldth_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_LVL_DETECT_TH_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

__INLINE void acc_lvl_detect_th_ldth_setf(uint8_t ldth)
{
    ASSERT_ERR((((uint8_t)ldth << 0) & ~((uint8_t)0x000000FF)) == 0);
    REG_ACC_WR(ACC_LVL_DETECT_TH_ADDR, (uint8_t)ldth << 0);
}

/**
 * @brief PLS_DETECT_TH register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00                 PDTH   0x0
 * </pre>
 */
#define ACC_PLS_DETECT_TH_ADDR   0x0000001B
#define ACC_PLS_DETECT_TH_OFFSET 0x0000001B
#define ACC_PLS_DETECT_TH_INDEX  0x0000001B
#define ACC_PLS_DETECT_TH_RESET  0x00000000

__INLINE uint8_t acc_pls_detect_th_get(void)
{
    return REG_ACC_RD(ACC_PLS_DETECT_TH_ADDR);
}

__INLINE void acc_pls_detect_th_set(uint8_t value)
{
    REG_ACC_WR(ACC_PLS_DETECT_TH_ADDR, value);
}

// field definitions
#define ACC_PDTH_MASK   ((uint8_t)0x000000FF)
#define ACC_PDTH_LSB    0
#define ACC_PDTH_WIDTH  ((uint8_t)0x00000008)

#define ACC_PDTH_RST    0x0

__INLINE uint8_t acc_pls_detect_th_pdth_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_PLS_DETECT_TH_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

__INLINE void acc_pls_detect_th_pdth_setf(uint8_t pdth)
{
    ASSERT_ERR((((uint8_t)pdth << 0) & ~((uint8_t)0x000000FF)) == 0);
    REG_ACC_WR(ACC_PLS_DETECT_TH_ADDR, (uint8_t)pdth << 0);
}

/**
 * @brief PLS_DUR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00                 PDUR   0x0
 * </pre>
 */
#define ACC_PLS_DUR_ADDR   0x0000001C
#define ACC_PLS_DUR_OFFSET 0x0000001C
#define ACC_PLS_DUR_INDEX  0x0000001C
#define ACC_PLS_DUR_RESET  0x00000000

__INLINE uint8_t acc_pls_dur_get(void)
{
    return REG_ACC_RD(ACC_PLS_DUR_ADDR);
}

__INLINE void acc_pls_dur_set(uint8_t value)
{
    REG_ACC_WR(ACC_PLS_DUR_ADDR, value);
}

// field definitions
#define ACC_PDUR_MASK   ((uint8_t)0x000000FF)
#define ACC_PDUR_LSB    0
#define ACC_PDUR_WIDTH  ((uint8_t)0x00000008)

#define ACC_PDUR_RST    0x0

__INLINE uint8_t acc_pls_dur_pdur_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_PLS_DUR_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

__INLINE void acc_pls_dur_pdur_setf(uint8_t pdur)
{
    ASSERT_ERR((((uint8_t)pdur << 0) & ~((uint8_t)0x000000FF)) == 0);
    REG_ACC_WR(ACC_PLS_DUR_ADDR, (uint8_t)pdur << 0);
}

/**
 * @brief LATENCY register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00                   LT   0x0
 * </pre>
 */
#define ACC_LATENCY_ADDR   0x0000001D
#define ACC_LATENCY_OFFSET 0x0000001D
#define ACC_LATENCY_INDEX  0x0000001D
#define ACC_LATENCY_RESET  0x00000000

__INLINE uint8_t acc_latency_get(void)
{
    return REG_ACC_RD(ACC_LATENCY_ADDR);
}

__INLINE void acc_latency_set(uint8_t value)
{
    REG_ACC_WR(ACC_LATENCY_ADDR, value);
}

// field definitions
#define ACC_LT_MASK   ((uint8_t)0x000000FF)
#define ACC_LT_LSB    0
#define ACC_LT_WIDTH  ((uint8_t)0x00000008)

#define ACC_LT_RST    0x0

__INLINE uint8_t acc_latency_lt_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_LATENCY_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

__INLINE void acc_latency_lt_setf(uint8_t lt)
{
    ASSERT_ERR((((uint8_t)lt << 0) & ~((uint8_t)0x000000FF)) == 0);
    REG_ACC_WR(ACC_LATENCY_ADDR, (uint8_t)lt << 0);
}

/**
 * @brief TIME_WIN_PLS2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00                   TW   0x0
 * </pre>
 */
#define ACC_TIME_WIN_PLS2_ADDR   0x0000001E
#define ACC_TIME_WIN_PLS2_OFFSET 0x0000001E
#define ACC_TIME_WIN_PLS2_INDEX  0x0000001E
#define ACC_TIME_WIN_PLS2_RESET  0x00000000

__INLINE uint8_t acc_time_win_pls2_get(void)
{
    return REG_ACC_RD(ACC_TIME_WIN_PLS2_ADDR);
}

__INLINE void acc_time_win_pls2_set(uint8_t value)
{
    REG_ACC_WR(ACC_TIME_WIN_PLS2_ADDR, value);
}

// field definitions
#define ACC_TW_MASK   ((uint8_t)0x000000FF)
#define ACC_TW_LSB    0
#define ACC_TW_WIDTH  ((uint8_t)0x00000008)

#define ACC_TW_RST    0x0

__INLINE uint8_t acc_time_win_pls2_tw_getf(void)
{
    uint8_t localVal = REG_ACC_RD(ACC_TIME_WIN_PLS2_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

__INLINE void acc_time_win_pls2_tw_setf(uint8_t tw)
{
    ASSERT_ERR((((uint8_t)tw << 0) & ~((uint8_t)0x000000FF)) == 0);
    REG_ACC_WR(ACC_TIME_WIN_PLS2_ADDR, (uint8_t)tw << 0);
}


#endif // _REG_ACC_H_

