/**
 ****************************************************************************************
 *
 * @file prf_utils.c
 *
 * @brief Implementation of Profile Utilities
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup PRF_UTILS
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (HOST_PROFILES)
#if (BLE_SERVER_PRF || BLE_CLIENT_PRF)

#include <stdint.h>
#include <stdbool.h>
#include "ke_task.h"
#include "gatt.h"
#include "prf_utils.h"
#include "gap.h"
#include "gapc.h"

#include "ke_mem.h"
#include "co_utils.h"
#include "co_error.h"
#include "co_endian.h"
#include "co_math.h"

#endif /* (BLE_SERVER_PRF || BLE_CLIENT_PRF) */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

#if (BLE_BATT_SERVER)
void prf_pack_char_pres_fmt(co_buf_t* p_buf, const prf_char_pres_fmt_t* char_pres_fmt)
{
    co_buf_tail(p_buf)[0] =         char_pres_fmt->format;
    co_buf_tail_reserve(p_buf, 1);
    co_buf_tail(p_buf)[0] =         char_pres_fmt->exponent;
    co_buf_tail_reserve(p_buf, 1);
    co_write16p(co_buf_tail(p_buf), co_htobs(char_pres_fmt->unit));
    co_buf_tail_reserve(p_buf, 2);
    co_buf_tail(p_buf)[0] =         char_pres_fmt->name_space;
    co_buf_tail_reserve(p_buf, 1);
    co_write16p(co_buf_tail(p_buf), co_htobs(char_pres_fmt->description));
    co_buf_tail_reserve(p_buf, 2);
}
#endif // (BLE_BATT_SERVER)

#if (BLE_BATT_CLIENT)
void prf_unpack_char_pres_fmt(co_buf_t* p_buf, prf_char_pres_fmt_t* char_pres_fmt)
{
    char_pres_fmt->format       = co_buf_data(p_buf)[0];
    co_buf_head_release(p_buf, 1);
    char_pres_fmt->exponent     = co_buf_data(p_buf)[0];
    co_buf_head_release(p_buf, 1);
    char_pres_fmt->unit         = co_btohs(co_read16p(co_buf_data(p_buf)));
    co_buf_head_release(p_buf, 2);
    char_pres_fmt->name_space   = co_buf_data(p_buf)[0];
    co_buf_head_release(p_buf, 1);
    char_pres_fmt->description  = co_btohs(co_read16p(co_buf_data(p_buf)));
    co_buf_head_release(p_buf, 2);
}
#endif // (BLE_BATT_CLIENT)

#if (BLE_CLIENT_PRF)
uint16_t prf_gatt_write(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint8_t write_type,
                        uint16_t hdl, uint16_t length, const uint8_t* p_data)
{
    co_buf_t* p_buf = NULL;
    uint16_t status;
    if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, length, GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
    {
        co_buf_copy_data_from_mem(p_buf, p_data, length);
        status = gatt_cli_write(conidx, user_lid, dummy, write_type, hdl, 0, p_buf);
        co_buf_release(p_buf);
    }
    else
    {
        status = GAP_ERR_INSUFF_RESOURCES;
    }

    return (status);
}

uint16_t prf_check_svc_char_validity(uint8_t nb_chars, const prf_char_t* p_chars,
                                     const prf_char_def_t* p_chars_req)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    uint8_t i;

    for (i = 0; ((i < nb_chars) && (status == GAP_ERR_NO_ERROR)); i++)
    {
        if (p_chars[i].val_hdl == GATT_INVALID_HDL)
        {
            // If Characteristic is not present, check requirements
            if (GETF(p_chars_req[i].req_bf, PRF_ATT_REQ_PRES) == PRF_ATT_REQ_PRES_MAND)
            {
                status = PRF_ERR_STOP_DISC_CHAR_MISSING;
                break;
            }
        }
        else
        {
            // If Characteristic is present, check properties
            if ((p_chars[i].prop & p_chars_req[i].prop_mand) != p_chars_req[i].prop_mand)
            {
                status = PRF_ERR_STOP_DISC_WRONG_CHAR_PROP;
                break;
            }
        }
    }

    return (status);
}

#if (BLE_PRF_PROPRIETARY_SVC_SUPPORT)
uint16_t prf_check_svc128_char_validity(uint8_t nb_chars, const prf_char_t* p_chars,
                                        const prf_char128_def_t* p_chars_req)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    uint8_t i;

    for (i = 0; ((i < nb_chars) && (status == GAP_ERR_NO_ERROR)); i++)
    {
        if (p_chars[i].val_hdl == GATT_INVALID_HDL)
        {
            // If Characteristic is not present, check requirements
            if (GETF(p_chars_req[i].req_bf, PRF_ATT_REQ_PRES) == PRF_ATT_REQ_PRES_MAND)
            {
                status = PRF_ERR_STOP_DISC_CHAR_MISSING;
                break;
            }
        }
        else
        {
            // If Characteristic is present, check properties
            if ((p_chars[i].prop & p_chars_req[i].prop_mand) != p_chars_req[i].prop_mand)
            {
                status = PRF_ERR_STOP_DISC_WRONG_CHAR_PROP;
                break;
            }
        }
    }

    return (status);
}
#endif // (BLE_PRF_PROPRIETARY_SVC_SUPPORT)

uint16_t prf_check_svc_desc_validity(uint8_t nb_descs, const prf_desc_t* p_descs,
                                     const prf_desc_def_t* p_descs_req, const prf_char_t* p_chars)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    uint8_t i;

    for (i = 0; ((i < nb_descs) && (status == GAP_ERR_NO_ERROR)) ; i++)
    {
        if (p_descs[i].desc_hdl == GATT_INVALID_HDL)
        {
            // If descriptor is missing, check if it is mandatory
            if (GETF(p_descs_req[i].req_bf, PRF_ATT_REQ_PRES) == PRF_ATT_REQ_PRES_MAND)
            {
                // Check if characteristic is present
                if (p_chars[p_descs_req[i].char_code].val_hdl != GATT_INVALID_HDL)
                {
                    // Characteristic is present but descriptor is not, error
                    status = PRF_ERR_STOP_DISC_CHAR_MISSING;
                    break;
                }
            }
        }
    }

    return (status);
}

uint16_t prf_check_svc_validity(uint8_t nb_chars,
                                const prf_char_t* p_chars, const prf_char_def_t* p_chars_req,
                                uint8_t nb_descs,
                                const prf_desc_t* p_descs, const prf_desc_def_t* p_descs_req)
{
    // Status
    uint16_t status = prf_check_svc_char_validity(nb_chars, p_chars, p_chars_req);

    if (status == GAP_ERR_NO_ERROR)
    {
        status = prf_check_svc_desc_validity(nb_descs, p_descs, p_descs_req, p_chars);
    }

    return (status);
}

void prf_extract_svc_info(uint16_t first_hdl, uint8_t nb_att, const gatt_svc_att_t* p_atts,
                          uint8_t nb_chars, const prf_char_def_t* p_chars_req, prf_char_t* p_chars,
                          uint8_t nb_descs, const prf_desc_def_t* p_descs_req, prf_desc_t* p_descs)
{
    // Counter
    uint8_t att_idx;

    for (att_idx = 0; att_idx < nb_att ; att_idx++)
    {
        if(p_atts[att_idx].att_type == GATT_ATT_CHAR)
        {
            uint16_t val_hdl  = p_atts[att_idx].info.charac.val_hdl;
            uint8_t  val_prop = p_atts[att_idx].info.charac.prop;
            // Counter
            uint8_t svc_char;

            //Look over requested characteristics
            for (svc_char = 0; svc_char < nb_chars ; svc_char++)
            {
                // check if attribute is valid
                if(   (p_chars[svc_char].val_hdl == GATT_INVALID_HDL)
                   && gatt_uuid16_comp(p_atts[att_idx].uuid, p_atts[att_idx].uuid_type,
                                       p_chars_req[svc_char].uuid))
                {
                    //Save properties and handles
                    p_chars[svc_char].val_hdl = val_hdl;
                    p_chars[svc_char].prop    = val_prop;

                    // discover descriptors
                    do
                    {
                        att_idx++;

                        // found a descriptor
                        if(p_atts[att_idx].att_type == GATT_ATT_DESC)
                        {
                            // Counter
                            uint8_t svc_desc;

                            //Retrieve characteristic descriptor handle using UUID
                            for(svc_desc = 0; svc_desc<nb_descs; svc_desc++)
                            {
                                // check if it's expected descriptor
                                if (   (p_descs[svc_desc].desc_hdl == GATT_INVALID_HDL)
                                    && (p_descs_req[svc_desc].char_code == svc_char)
                                    && (gatt_uuid16_comp(p_atts[att_idx].uuid, p_atts[att_idx].uuid_type,
                                                         p_descs_req[svc_desc].uuid)))
                                {
                                    p_descs[svc_desc].desc_hdl = first_hdl + att_idx;
                                    // search for next descriptor
                                    break;
                                }
                            }
                        }
                    } while(   (att_idx < (nb_att - 1))
                            && (p_atts[att_idx].att_type != GATT_ATT_CHAR)
                            && (p_atts[att_idx].att_type != GATT_ATT_INCL_SVC));

                    // return to previous valid value
                    att_idx--;

                    // search next characteristic
                    break;
                }
            }
        }
    }
}

#if (BLE_PRF_PROPRIETARY_SVC_SUPPORT)
void prf_extract_svc128_info(uint16_t first_hdl, uint8_t nb_att, const gatt_svc_att_t* p_atts,
                             uint8_t nb_chars, const prf_char128_def_t* p_chars_req, prf_char_t* p_chars,
                             uint8_t nb_descs, const prf_desc_def_t* p_descs_req, prf_desc_t* p_descs)
{
    // Counter
    uint8_t att_idx;

    for (att_idx = 0; att_idx < nb_att ; att_idx++)
    {
        if(p_atts[att_idx].att_type == GATT_ATT_CHAR)
        {
            uint16_t val_hdl  = p_atts[att_idx].info.charac.val_hdl;
            uint8_t  val_prop = p_atts[att_idx].info.charac.prop;
            // Counter
            uint8_t svc_char;

            //Look over requested characteristics
            for (svc_char = 0; svc_char < nb_chars ; svc_char++)
            {
                // check if attribute is valid
                if(   (p_chars[svc_char].val_hdl == GATT_INVALID_HDL)
                   && gatt_uuid_comp(p_atts[att_idx].uuid, p_atts[att_idx].uuid_type, p_chars_req[svc_char].uuid, GATT_UUID_128))
                {
                    //Save properties and handles
                    p_chars[svc_char].val_hdl        = val_hdl;
                    p_chars[svc_char].prop           = val_prop;

                    // find end of characteristic handle and discover descriptors
                    do
                    {
                        att_idx++;

                        // found a descriptor
                        if(p_atts[att_idx].att_type == GATT_ATT_DESC)
                        {
                            // Counter
                            uint8_t svc_desc;

                            //Retrieve characteristic descriptor handle using UUID
                            for(svc_desc = 0; svc_desc<nb_descs; svc_desc++)
                            {
                                // check if it's expected descriptor
                                if (   (p_descs[svc_desc].desc_hdl == GATT_INVALID_HDL)
                                    && (p_descs_req[svc_desc].char_code == svc_char)
                                    && (gatt_uuid16_comp(p_atts[att_idx].uuid, p_atts[att_idx].uuid_type, p_descs_req[svc_desc].uuid)))
                                {
                                    p_descs[svc_desc].desc_hdl = first_hdl + att_idx;
                                    // search for next descriptor
                                    break;
                                }
                            }
                        }
                    } while(   (att_idx < (nb_att - 1))
                            && (p_atts[att_idx].att_type != GATT_ATT_CHAR)
                            && (p_atts[att_idx].att_type != GATT_ATT_INCL_SVC));

                    // return to previous valid value
                    att_idx--;

                    // search next characteristic
                    break;
                }
            }
        }
    }
}
#endif // (BLE_PRF_PROPRIETARY_SVC_SUPPORT)
#endif //(BLE_CLIENT_PRF)

#if ((BLE_SERVER_PRF || BLE_CLIENT_PRF))
void prf_pack_date_time(co_buf_t* p_buf, const prf_date_time_t* p_date_time)
{
    co_write16p(co_buf_tail(p_buf), co_htobs(p_date_time->year));
    co_buf_tail_reserve(p_buf, 2);
    co_buf_tail(p_buf)[0] = p_date_time->month;
    co_buf_tail_reserve(p_buf, 1);
    co_buf_tail(p_buf)[0] = p_date_time->day;
    co_buf_tail_reserve(p_buf, 1);
    co_buf_tail(p_buf)[0] = p_date_time->hour;
    co_buf_tail_reserve(p_buf, 1);
    co_buf_tail(p_buf)[0] = p_date_time->min;
    co_buf_tail_reserve(p_buf, 1);
    co_buf_tail(p_buf)[0] = p_date_time->sec;
    co_buf_tail_reserve(p_buf, 1);
}

void prf_pack_date(co_buf_t* p_buf, const prf_date_t* p_date)
{
    co_write16p(co_buf_tail(p_buf), co_htobs(p_date->year));
    co_buf_tail_reserve(p_buf, 2);
    co_buf_tail(p_buf)[0] = p_date->month;
    co_buf_tail_reserve(p_buf, 1);
    co_buf_tail(p_buf)[0] = p_date->day;
    co_buf_tail_reserve(p_buf, 1);
}

void prf_unpack_date_time(co_buf_t* p_buf, prf_date_time_t* p_date_time)
{
    p_date_time->year   = co_btohs(co_read16p(co_buf_data(p_buf)));
    co_buf_head_release(p_buf, 2);
    p_date_time->month  = co_buf_data(p_buf)[0];
    co_buf_head_release(p_buf, 1);
    p_date_time->day    = co_buf_data(p_buf)[0];
    co_buf_head_release(p_buf, 1);
    p_date_time->hour   = co_buf_data(p_buf)[0];
    co_buf_head_release(p_buf, 1);
    p_date_time->min    = co_buf_data(p_buf)[0];
    co_buf_head_release(p_buf, 1);
    p_date_time->sec    = co_buf_data(p_buf)[0];
    co_buf_head_release(p_buf, 1);
}

void prf_unpack_date(co_buf_t* p_buf, prf_date_t* p_date)
{
    p_date->year   = co_btohs(co_read16p(co_buf_data(p_buf)));
    co_buf_head_release(p_buf, 2);
    p_date->month  = co_buf_data(p_buf)[0];
    co_buf_head_release(p_buf, 1);
    p_date->day    = co_buf_data(p_buf)[0];
    co_buf_head_release(p_buf, 1);
}

#if ((BLE_OT_CLIENT) || (BLE_OT_SERVER))
bool prf_is_object_id_valid(const ot_object_id_t* p_object_id)
{
    // Validity of Object ID
    bool valid = false;

    if (p_object_id != NULL)
    {
        // Check normal Object ID (0x000000000100 to 0xFFFFFFFFFFFF)
        if ((p_object_id->object_id[1] != 0)
                || (p_object_id->object_id[2] != 0)
                || (p_object_id->object_id[3] != 0)
                || (p_object_id->object_id[4] != 0)
                || (p_object_id->object_id[5] != 0))
        {
            valid = true;
        }
        // Check if it's a Directory Listing Object ID (0x000000000000)
        else if ((p_object_id->object_id[0] == 0)
                    && (p_object_id->object_id[1] == 0)
                    && (p_object_id->object_id[2] == 0)
                    && (p_object_id->object_id[3] == 0)
                    && (p_object_id->object_id[4] == 0)
                    && (p_object_id->object_id[5] == 0))
        {
            valid = true;
        }
    }

    return (valid);
}
#endif // ((BLE_OT_CLIENT) || (BLE_OT_SERVER))

#if ((BLE_CGM_SERVER || BLE_CGM_CLIENT))
uint16_t prf_e2e_crc(co_buf_t* p_buf)
{
    uint16_t crc_reg = 0xffff; // initial seed
    uint8_t reg;
    uint16_t crc;
    uint8_t i, j;
    uint8_t* p_data = co_buf_data(p_buf);
    
    for(i=0; i < co_buf_data_len(p_buf); i++)
    {
      reg = *p_data++;
      for(j=0; j<8; j++)
      {
        if ((reg & 1) != ((crc_reg>>15)& 1))
        {
          crc_reg <<= 1;
          crc_reg ^= (uint16_t)(1<<12) | (1<<5) | (1<<0);
        }
        else
        {
          crc_reg <<= 1;
        }
        reg >>= 1;
      }
    }
    
    crc = 0;
    // result is in the order bits:->8 9 10 11 12 13 14 15 0 1 2 3 4 5 6 7
    for(j=0; j<8; j++)
    {
      crc >>= 1;
      crc |= crc_reg & ((1<<15) | (1<<7));
      crc_reg <<= 1;
    }

    return crc;
}
#endif /* ((BLE_CGM_SERVER || BLE_CGM_CLIENT)) */

uint8_t prf_buf_alloc(co_buf_t** pp_buf, uint16_t data_len)
{
    return (co_buf_alloc(pp_buf, GATT_BUFFER_HEADER_LEN, data_len, GATT_BUFFER_TAIL_LEN));
}

uint32_t prf_evt_get_con_bf(uint8_t* p_cli_cfg_bf, uint8_t char_type)
{
    // Bit field indicating connection for which sending of notification is enabled for the indicated characteristic
    uint32_t con_bf = 0;
    // Counter
    uint8_t cnt;

    for (cnt = 0; cnt < HOST_CONNECTION_MAX; cnt++)
    {
        if (*(p_cli_cfg_bf + cnt) & CO_BIT(char_type))
        {
            // Notification can be sent for this connection
            con_bf |= CO_BIT(cnt);
        }
    }

    return (con_bf);
}

#endif /* ((BLE_SERVER_PRF || BLE_CLIENT_PRF)) */

#endif // (HOST_PROFILES)

/// @} PRF_UTILS

