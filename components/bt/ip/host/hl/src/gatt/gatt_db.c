/**
 ****************************************************************************************
 * @file gatt_db.c
 *
 * @brief  GATT Database Manager
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GATT
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"        // IP configuration
#if (BLE_GATT)
#include "gatt.h"               // Native API
#include "gatt_int.h"           // Internals

#include "gapc.h"               // Security level apis
#include "../inc/gap_hl_api.h"  // GAP internals

#include "co_math.h"            // Mathematics
#include "co_endian.h"          // for host to bt number conversion
#include "ke_mem.h"             // Kernel memory

#include "aes.h"                // For Hash computation

#include <string.h>             // for memcmp and memcpy


/*
 * MACROS
 ****************************************************************************************
 */

/// Compute Service End handle
#define GATT_DB_END_HDL_GET(p_svc) ((p_svc)->start_hdl + (p_svc)->nb_att - 1)

/// Compute Service Last handle - from reserved range
#define GATT_DB_LAST_HDL_GET(p_svc) ((p_svc)->start_hdl + (p_svc)->nb_att_rsvd - 1)

/*
 * DEFINES
 ****************************************************************************************
 */
/// Size of characteristic extend property value
#define GATT_CHAR_EXT_PROPERTY_LEN    (2)

/// Security level mask
#define GATT_DB_SEC_LVL_MASK      (0x0003)

/// GATT service type
enum gatt_svc_type
{
    /// Primary service type
    GATT_SVC_PRIMARY   = 0x00,
    /// Secondary service type
    GATT_SVC_SECONDARY = 0x01,
};

///    7    6 5 4 3 2 1 0
/// +------+-+-+-+-+-+-+-+
/// | TYPE |    INFO     |
/// +------+-+-+-+-+-+-+-+
/// GATT Service information Bit Field - continue
enum gatt_svc_info_cont_bf
{
    /// Attribute information
    GATT_SVC_INFO_MASK      = 0x7F,
    GATT_SVC_INFO_LSB       = 0,
    /// Service type (see enum #gatt_svc_type)
    GATT_SVC_TYPE_BIT       = 0x80,
    GATT_SVC_TYPE_POS       = 7,
};


///    15   14   13 12 11 10  9  8  7  6  5  4  3  2  1  0
/// +-----+-----+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
/// | UUID_TYPE |                 INFO                    |
/// +-----+-----+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
///              <----- PERM ----> <------- PROP -------->
/// GATT Attribute information Bit Field - continue
enum gatt_att_info_cont_bf
{
    /// Attribute information
    GATT_ATT_INFO_MASK      = 0x3FFF,
    GATT_ATT_INFO_LSB       = 0,
};


/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Attribute right info
typedef struct gatt_db_right
{
    /// Right mode Mask
    uint16_t mode_mask;
    /// Right Permission LSB
    uint16_t perm_lsb;
    /// Default error status is right disabled
    uint16_t err_status;
} gatt_db_right_t;


/// Buffer meta-data structure for database hash computation
typedef struct gatt_db_hash_buf_meta
{
    /// Callback where database hash is returned
    const gatt_db_hash_cb_t* p_cb;
    /// Dummy parameter provided by upper layer for command execution.
    uint16_t                 dummy;
    /// User Local identifier
    uint8_t                  user_lid;
    /// Requester connection index
    uint8_t                  conidx;
} gatt_db_hash_buf_meta_t;

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Attribute Right info according to access
__STATIC const gatt_db_right_t gatt_db_rights[] =
{
    [GATT_DB_ACCESS_READ]          = { GATT_ATT_RD_BIT,  GATT_ATT_RP_LSB,  ATT_ERR_READ_NOT_PERMITTED    },
    [GATT_DB_ACCESS_WRITE]         = { GATT_ATT_WR_BIT,  GATT_ATT_WP_LSB,  ATT_ERR_WRITE_NOT_PERMITTED   },
    [GATT_DB_ACCESS_WRITE_COMMAND] = { GATT_ATT_WC_BIT,  GATT_ATT_WP_LSB,  ATT_ERR_WRITE_NOT_PERMITTED   },
    [GATT_DB_ACCESS_WRITE_SIGNED]  = { GATT_ATT_WS_BIT,  GATT_ATT_WP_LSB,  ATT_ERR_WRITE_NOT_PERMITTED   },
    [GATT_DB_ACCESS_NOTIFY]        = { GATT_ATT_N_BIT,   GATT_ATT_NIP_LSB, ATT_ERR_REQUEST_NOT_SUPPORTED },
    [GATT_DB_ACCESS_INDICATE]      = { GATT_ATT_I_BIT,   GATT_ATT_NIP_LSB, ATT_ERR_REQUEST_NOT_SUPPORTED },
};

/// Key used to compute database hash
__STATIC const uint8_t gatt_db_hash_key[KEY_LEN] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief The Attribute database has been updated, handle this modification
 *
 * @param[in] start_hdl Service changed start handle
 * @param[in] emd_hdl   Service changed end handle
 ****************************************************************************************
 */
__STATIC void gatt_db_updated(uint16_t start_hdl, uint16_t end_hdl)
{
    // mark database hash invalid
    gatt_env.db_hash_valid = false;

    // Inform that database has been updated
    gapc_svc_db_updated(start_hdl, end_hdl);
}

/**
 ****************************************************************************************
 * @brief Command used to Create a service into local attribute database.
 *
 * @note Service attribute not present in attribute list
 *
 * @param[in]     user_lid     GATT User Local identifier
 * @param[in]     info         Service Information bit field (see Table 18)
 * @param[in]     p_uuid       Pointer to service UUID
 * @param[in]     nb_att_rsvd  Number of attribute(s) reserved for the service (shall be equals or greater nb_att)
 *                             Prevent any services to be inserted between start_hdl and (start_hdl + nb_att_rsvd - 1)
 * @param[in]     nb_att       Number of attribute(s) in service
 * @param[in]     p_atts       Pointer to List of attribute
 * @param[in,out] p_start_hdl  Pointer to Service Start Handle (0 = chosen by GATT module)
 *                             Pointer updated with service start handle associated to
 *                             created service.
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC uint16_t gatt_db_svc_create(uint8_t user_lid, uint8_t info, const uint8_t* p_uuid, uint8_t nb_att_rsvd,
                                     uint8_t nb_att, const gatt_att_desc_t* p_atts, uint16_t uuid_mem_size,
                                     uint16_t* p_start_hdl)
{
    uint16_t status = GAP_ERR_NO_ERROR;

    do
    {
        gatt_db_svc_t* p_svc;
        uint8_t  cursor;
        uint16_t svc_mem_size   = 0;

        // -------------------------------------------------------- //
        // ---   1. Compute Service size                        --- //
        // -------------------------------------------------------- //
        svc_mem_size = sizeof(gatt_db_svc_t) + (nb_att * sizeof(gatt_db_att_t));

        // -------------------------------------------------------- //
        // ---   2. Search where new service can be inserted    --- //
        // -------------------------------------------------------- //

        // Reserve attribute handle range
        status = gatt_db_handle_range_reserve(user_lid, nb_att_rsvd, p_start_hdl);
        if(status != GAP_ERR_NO_ERROR) break;

        // -------------------------------------------------------- //
        // ---   3. Allocate the Service                        --- //
        // -------------------------------------------------------- //
        p_svc  = (gatt_db_svc_t*) ke_malloc_user(svc_mem_size + uuid_mem_size, KE_MEM_PROFILE);
        if(p_svc == NULL)
        {
            status = GAP_ERR_INSUFF_RESOURCES;
            break;
        }

        // -------------------------------------------------------- //
        // ---   4. Fill Service Information                    --- //
        // -------------------------------------------------------- //
        {
            uint8_t att_cursor;
            // pointer where uuid data will be pushed
            uint8_t* p_data = ((uint8_t*) p_svc) + svc_mem_size;
            uint16_t data_offset = svc_mem_size;

            // fill service information
            p_svc->start_hdl   = *p_start_hdl;
            p_svc->nb_att      = nb_att;
            p_svc->nb_att_rsvd = nb_att_rsvd;
            p_svc->perm        = info;
            p_svc->user_lid    = user_lid;

            // Ensure that service is visible
            SETB(p_svc->perm, GATT_SVC_HIDE, 0);

            // Add memory size needed for UUIDs
            switch(GETF(info, GATT_SVC_UUID_TYPE))
            {
                case GATT_UUID_16: { memcpy(&(p_svc->uuid), &(p_uuid[0]), GATT_UUID_16_LEN); } break;
                case GATT_UUID_32:
                {
                    memcpy(p_data, p_uuid,  GATT_UUID_32_LEN);
                    p_svc->uuid    = data_offset; // put data offset where uuid can be found
                    data_offset   += GATT_UUID_32_LEN;
                    p_data        += GATT_UUID_32_LEN;
                } break;
                case GATT_UUID_128:
                {
                    memcpy(p_data, p_uuid, GATT_UUID_128_LEN);
                    p_svc->uuid    = data_offset; // put data offset where uuid can be found
                    data_offset   += GATT_UUID_128_LEN;
                    p_data        += GATT_UUID_128_LEN;
                } break;
                default:            { status = GAP_ERR_INVALID_PARAM;     } break;
            }

            // Verify all attributes
            for(cursor = 0, att_cursor = 0; (cursor < nb_att) ; cursor++)
            {
                const gatt_att_desc_t* p_att_desc = &(p_atts[cursor]);
                uint8_t  uuid_type = GETF(p_att_desc->info, GATT_ATT_UUID_TYPE);
                gatt_db_att_t*         p_att      = &(p_svc->att[att_cursor]);
                p_uuid = &(p_att_desc->uuid[0]);

                p_att->perm     = p_att_desc->info;
                p_att->ext_info = p_att_desc->ext_info;

                // Add memory size needed for UUIDs
                switch(uuid_type)
                {
                    case GATT_UUID_16:
                    {
                        memcpy(&(p_att->uuid), &(p_uuid[0]), GATT_UUID_16_LEN);
                    } break;
                    case GATT_UUID_32:
                    {
                        memcpy(p_data, p_uuid,  GATT_UUID_32_LEN);
                        p_att->uuid    = data_offset; // put data offset where uuid can be found
                        data_offset   += GATT_UUID_32_LEN;
                        p_data        += GATT_UUID_32_LEN;
                    } break;
                    case GATT_UUID_128:
                    {
                        memcpy(p_data, p_uuid, GATT_UUID_128_LEN);
                        p_att->uuid    = data_offset; // put data offset where uuid can be found
                        data_offset   += GATT_UUID_128_LEN;
                        p_data        += GATT_UUID_128_LEN;
                    } break;
                    default: { /* Nothing to do */ } break;
                }

                // for UUID 128 check if it's a 32 or a 128bits UUID
                if(uuid_type == GATT_UUID_16)
                {
                    // value handle is just after characteristic definition
                    if(p_att->uuid == GATT_DECL_CHARACTERISTIC)
                    {
                        // Force permission to read only
                        p_att->perm    = GATT_ATT_RD_BIT;
                        SETF(p_att->perm, GATT_SVC_UUID_TYPE, GATT_UUID_16);
                        p_att->ext_info = 0;
                    }
                    // for following attributes, max_length field contains a value
                    else if((p_att->uuid == GATT_DECL_INCLUDE) || (p_att->uuid == GATT_DESC_CHAR_EXT_PROPERTIES))
                    {
                        // Force permission to read only
                        p_att->perm    = GATT_ATT_RD_BIT;
                        SETF(p_att->perm, GATT_SVC_UUID_TYPE, GATT_UUID_16);
                    }
                }

                att_cursor++;
            }
        }

        // -------------------------------------------------------- //
        // ---   5. Insert Service                              --- //
        // -------------------------------------------------------- //
        {
            gatt_db_svc_t** pp_current_svc = &(gatt_env.p_db);
            bool found = false;

            // browse service list.
            while((*pp_current_svc != NULL) && (!found))
            {
                // new service could be insert before browsed service
                if(p_svc->start_hdl < (*pp_current_svc)->start_hdl)
                {
                    // insert element in list
                    p_svc->p_next = *pp_current_svc;
                    *pp_current_svc = p_svc;
                    found = true;
                }
                // continue with next service
                else
                {
                    pp_current_svc = &((*pp_current_svc)->p_next);
                }
            }

            // no service found, put service at current pointer (end of service list)
            if(!found)
            {
                *pp_current_svc           = p_svc;
                (*pp_current_svc)->p_next = NULL;
            }
        }

        // -------------------------------------------------------- //
        // ---   6. Inform that database has been updated       --- //
        // -------------------------------------------------------- //
        if(!GETB(info, GATT_SVC_HIDE))
        {

            gatt_db_updated(p_svc->start_hdl, GATT_DB_END_HDL_GET(p_svc));
        }

    } while(0);

    return (status);
}


/**
 ****************************************************************************************
 * @brief Find service form attribute handle
 *
 * @param[in] hdl          Attribute handle
 *
 * @return Pointer to the attribute service, NULL if service not found
 ****************************************************************************************
 */
__STATIC gatt_db_svc_t* gatt_db_svc_get(uint16_t hdl)
{
    gatt_db_svc_t* p_svc = NULL;
    gatt_db_svc_t* p_current_svc = gatt_env.p_db;

    while((p_current_svc != NULL))
    {
        // search handle is before current service
        if(hdl < p_current_svc->start_hdl)
        {
            // search is over
            break;
        }
        // search handle is after current service
        else if (hdl > GATT_DB_END_HDL_GET(p_current_svc))
        {
            // check next service
            p_current_svc  = p_current_svc->p_next;
        }
        // Service found
        else
        {
            p_svc = p_current_svc;
            break;
        }
    }

    return (p_svc);
}

/**
 ****************************************************************************************
 * @brief Compare attribute UUID with a given UUID
 *
 * @param[in] p_svc      Pointer to Service of the attribute
 * @param[in] p_att      Pointer to attribute
 * @param[in] p_uuid     Pointer to the UUID value to compare
 * @param[in] uuid_type  Type of UUID (see enum #gatt_uuid_type)
 *
 * @return true if UUIDs matches, false otherwise
 ****************************************************************************************
 */
__STATIC bool gatt_db_uuid_comp(gatt_db_svc_t* p_svc, gatt_db_att_t* p_att, uint8_t* p_uuid, uint8_t uuid_type)
{
    uint8_t* p_att_uuid = (uint8_t*) &(p_att->uuid);

    if((GETF(p_att->perm, GATT_ATT_UUID_TYPE) != GATT_UUID_16))
    {
        p_att_uuid = ((uint8_t*) p_svc) + p_att->uuid;
    }

    return (gatt_uuid_comp(p_att_uuid, GETF(p_att->perm, GATT_ATT_UUID_TYPE), p_uuid, uuid_type));
}
/*
 * INTERNAL FUNCTIONS
 ****************************************************************************************
 */
uint16_t gatt_db_att_get(uint16_t hdl, gatt_db_svc_t** pp_svc, gatt_db_att_t** pp_att)
{
    uint16_t  status = GAP_ERR_NO_ERROR;

    // retrieve Service
    gatt_db_svc_t* p_svc = gatt_db_svc_get(hdl);

    // check if service is available
    if((p_svc == NULL) || (GETB(p_svc->perm, GATT_SVC_HIDE)))
    {
        status = ATT_ERR_INVALID_HANDLE;
    }
    else
    {
        // return found attribute
        *pp_att = &(p_svc->att[hdl - p_svc->start_hdl]);
        *pp_svc = p_svc;
    }

    return (status);
}

uint16_t gatt_db_att_find(uint16_t start_hdl, uint16_t end_hdl, bool by_uuid, uint8_t uuid_type, uint8_t* p_uuid,
                          uint16_t* p_hdl, uint16_t* p_end_grp_hdl, gatt_db_svc_t** pp_svc, gatt_db_att_t** pp_att)
{
    uint16_t status = ATT_ERR_ATTRIBUTE_NOT_FOUND;
    gatt_db_svc_t* p_current_svc = gatt_env.p_db;
    gatt_db_att_t* p_att;
    bool svc_search = false;

    // check if a service discovery is on-going
    if(by_uuid && (uuid_type == GATT_UUID_16))
    {
        uint16_t uuid16 = co_read16p(p_uuid);
        if((uuid16 == GATT_DECL_PRIMARY_SERVICE) || (uuid16 == GATT_DECL_SECONDARY_SERVICE))
        {
            svc_search = true;
        }
    }

    if((start_hdl <= end_hdl) && (start_hdl != GATT_INVALID_HDL))
    {
        while((p_current_svc != NULL))
        {
            uint16_t svc_end_hdl = GATT_DB_END_HDL_GET(p_current_svc);

            // search handle is before current service
            if(end_hdl < p_current_svc->start_hdl)
            {
                // search is over
                break;
            }
            // search handle is after current service or current service is hidden
            else if ((start_hdl > svc_end_hdl) || GETB(p_current_svc->perm, GATT_SVC_HIDE))
            {
                // check next service
                p_current_svc  = p_current_svc->p_next;
            }
            // A possible Service found
            else
            {
                // update searched start handle
                if(start_hdl < p_current_svc->start_hdl)
                {
                    start_hdl = p_current_svc->start_hdl;
                }

                // loop onto the service to find attribute
                while((start_hdl <= svc_end_hdl) && (start_hdl <= end_hdl) && (start_hdl != GATT_INVALID_HDL))
                {
                    p_att = &(p_current_svc->att[start_hdl - p_current_svc->start_hdl]);

                    // check if attribute is found
                    if(!by_uuid || gatt_db_uuid_comp(p_current_svc, p_att, p_uuid, uuid_type))
                    {
                        *p_hdl = start_hdl;

                        status  = GAP_ERR_NO_ERROR;
                        *pp_svc = p_current_svc;
                        *pp_att = p_att;

                        // search end group handle
                        if(start_hdl == p_current_svc->start_hdl)
                        {
                            *p_end_grp_hdl = svc_end_hdl;
                        }
                        else if(   (GETF(p_att->perm, GATT_ATT_UUID_TYPE) != GATT_UUID_16)
                                || (p_att->uuid != GATT_DECL_CHARACTERISTIC))
                        {
                            *p_end_grp_hdl = start_hdl;
                        }
                        else
                        {
                            // for a characteristic find include service, next characteristic or end of service
                            start_hdl++;
                            while((start_hdl <= svc_end_hdl) && (start_hdl != GATT_INVALID_HDL))
                            {
                                p_att = &(p_current_svc->att[start_hdl - p_current_svc->start_hdl]);
                                if(   (GETF(p_att->perm, GATT_ATT_UUID_TYPE) == GATT_UUID_16)
                                   && ((p_att->uuid == GATT_DECL_CHARACTERISTIC) || (p_att->uuid == GATT_DECL_INCLUDE)))
                                {
                                    break;
                                }
                                start_hdl++;
                            }


                            *p_end_grp_hdl = start_hdl - 1;
                        }

                        p_current_svc = NULL; // stop the search
                        break;
                    }


                    // speed-up search
                    if(svc_search)
                    {
                        start_hdl = GATT_DB_LAST_HDL_GET(p_current_svc) + 1;
                    }
                    else
                    {
                        start_hdl++;
                    }
                }

                if(p_current_svc != NULL)
                {
                    // check next service
                    p_current_svc  = p_current_svc->p_next;
                }
            }
        }
    }

    return (status);
}

uint16_t gatt_db_att_access_check(uint8_t conidx, uint8_t access, gatt_db_svc_t* p_svc, gatt_db_att_t* p_att)
{
    uint16_t  status  = GAP_ERR_NO_ERROR;

    do
    {
        uint16_t  sec_lvl = 0;

        // check Native supported attribute types - used for discovery
        if(GETF(p_att->perm, GATT_ATT_UUID_TYPE) == GATT_UUID_16)
        {
            if(   (p_att->uuid == GATT_DECL_PRIMARY_SERVICE)
               || (p_att->uuid == GATT_DECL_SECONDARY_SERVICE)
               || (p_att->uuid == GATT_DECL_CHARACTERISTIC)
               || (p_att->uuid == GATT_DECL_INCLUDE)
               || (p_att->uuid == GATT_DESC_CHAR_EXT_PROPERTIES))
            {
                // Read only without specific permission needed else reject
                if (access != GATT_DB_ACCESS_READ)
                {
                    status = gatt_db_rights[access].err_status;
                }
                break;
            }
        }

        // Check if mode is supported
        if((gatt_db_rights[access].mode_mask & p_att->perm) == 0)
        {
            status = gatt_db_rights[access].err_status;
            break;
        }

        // retrieve security level
        sec_lvl = (p_att->perm >> gatt_db_rights[access].perm_lsb) & GATT_DB_SEC_LVL_MASK;
        // check if security level is not overloaded
        if(sec_lvl < GETF(p_svc->perm, GATT_SVC_AUTH))
        {
            sec_lvl = GETF(p_svc->perm, GATT_SVC_AUTH);
        }

        // When attribute requires pairing
        if(sec_lvl > GATT_SEC_NOT_ENC)
        {
            // NOTE: the checking is done according to CSA3 - by priority
            if(gapc_lk_sec_lvl_get(conidx) == GAP_SEC_NOT_ENC)
            {
                status = ATT_ERR_INSUFF_AUTHEN;
                break;
            }

            // check if link is encrypted and EKS must be in such good level
            // encryption is not for SIGNED WRITE ACCESS
            if (access != GATT_DB_ACCESS_WRITE_SIGNED)
            {
                // encryption must be enabled
                if(!gapc_is_sec_set(conidx, GAPC_LK_ENCRYPTED))
                {
                    // check if LTK exchanged during pairing
                    if(gapc_is_sec_set(conidx, GAPC_LK_BONDED) && gapc_is_sec_set(conidx, GAPC_LK_ENC_KEY_PRESENT))
                    {
                        status = ATT_ERR_INSUFF_ENC;
                    }
                    else
                    {
                        status = ATT_ERR_INSUFF_AUTHEN;
                    }
                    break;
                }

                //check encryption key size if attribute requires maximum
                if (GETB(p_svc->perm, GATT_SVC_EKS) && (gapc_enc_keysize_get(conidx) < GAP_SEC_ENC_KEY_SIZE))
                {
                    status = ATT_ERR_INSUFF_ENC_KEY_SIZE;
                    break;
                }
            }

            // check if connection has enough authentication level
            if(gapc_lk_sec_lvl_get(conidx) < sec_lvl)
            {
                status = ATT_ERR_INSUFF_AUTHEN;
                break;
            }
        }

        // check if service is disabled
        if(GETB(p_svc->perm, GATT_SVC_DIS))
        {
            status = ATT_ERR_INSUFF_AUTHOR;
            break;
        }
    } while(0);

    return (status);
}

uint16_t gatt_db_att_native_val_get(uint16_t hdl, gatt_db_svc_t* p_svc, gatt_db_att_t* p_att,
                                    uint8_t* p_out, uint16_t* p_length)
{
    uint16_t  status  = GAP_ERR_NO_ERROR;

    // check Native supported attribute types - Data is present in database
    if(GETF(p_att->perm, GATT_ATT_UUID_TYPE) == GATT_UUID_16)
    {
        switch(p_att->uuid)
        {
            case GATT_DECL_PRIMARY_SERVICE:
            case GATT_DECL_SECONDARY_SERVICE:
            {
                // extract UUID information
                if(GETF(p_svc->perm, GATT_SVC_UUID_TYPE) == GATT_UUID_16)
                {
                    co_write16p(p_out, p_svc->uuid);
                    *p_length = GATT_UUID_16_LEN;
                }
                else
                {
                    uint8_t*  p_uuid;
                    p_uuid = ((uint8_t*) p_svc) + p_svc->uuid;
                    gatt_uuid128_convert(p_uuid, GETF(p_svc->perm, GATT_SVC_UUID_TYPE), p_out);
                    *p_length = GATT_UUID_128_LEN;
                }
            } break;
            case GATT_DECL_CHARACTERISTIC:
            {
                gatt_db_att_t* p_val_att;
                ASSERT_ERR(hdl < GATT_DB_END_HDL_GET(p_svc));

                p_val_att = &(p_svc->att[hdl + 1 - p_svc->start_hdl]);

                // prop + val_hdl + UUID
                *p_out = GETF(p_val_att->perm, GATT_ATT_PROP);
                p_out += L2CAP_ATT_PROP_LEN;
                co_write16p(p_out, co_htobs(hdl + 1));
                p_out += GATT_HANDLE_LEN;
                *p_length = L2CAP_ATT_PROP_LEN + GATT_HANDLE_LEN;

                // extract UUID information
                if(GETF(p_val_att->perm, GATT_ATT_UUID_TYPE) == GATT_UUID_16)
                {
                    co_write16p(p_out, p_val_att->uuid);
                    *p_length += GATT_UUID_16_LEN;
                }
                else
                {
                    uint8_t*  p_uuid;
                    p_uuid = ((uint8_t*) p_svc) + p_val_att->uuid;
                    gatt_uuid128_convert(p_uuid, GETF(p_val_att->perm, GATT_ATT_UUID_TYPE), p_out);
                    *p_length += GATT_UUID_128_LEN;
                }

            } break;
            case GATT_DECL_INCLUDE:
            {
                gatt_db_svc_t* p_inc_svc = gatt_db_svc_get(p_att->ext_info);

                *p_length = GATT_HANDLE_LEN + GATT_HANDLE_LEN;

                if(p_inc_svc == NULL)
                {
                    // start hdl + end hdl + UUID
                    co_write16p(p_out, co_htobs(0));
                    p_out += GATT_HANDLE_LEN;
                    co_write16p(p_out, co_htobs(0));
                    p_out += GATT_HANDLE_LEN;
                    memset(p_out, 0 , GATT_UUID_128_LEN);
                    *p_length += GATT_UUID_128_LEN;
                }
                else
                {
                    // start hdl + end hdl + UUID
                    co_write16p(p_out, co_htobs(p_inc_svc->start_hdl));
                    p_out += GATT_HANDLE_LEN;
                    co_write16p(p_out, co_htobs(GATT_DB_END_HDL_GET(p_inc_svc)));
                    p_out += GATT_HANDLE_LEN;

                    // extract UUID information
                    if(GETF(p_inc_svc->perm, GATT_SVC_UUID_TYPE) == GATT_UUID_16)
                    {
                        co_write16p(p_out, p_inc_svc->uuid);
                        *p_length += GATT_UUID_16_LEN;
                    }
                    else
                    {
                        uint8_t*  p_uuid;
                        p_uuid = ((uint8_t*) p_inc_svc) + p_inc_svc->uuid;
                        gatt_uuid128_convert(p_uuid, GETF(p_inc_svc->perm, GATT_SVC_UUID_TYPE), p_out);
                        *p_length += GATT_UUID_128_LEN;
                    }
                }
            } break;
            case GATT_DESC_CHAR_EXT_PROPERTIES:
            {
                co_write16p(p_out, p_att->ext_info);
                *p_length = GATT_CHAR_EXT_PROPERTY_LEN;
            } break;
            default: { status = ATT_ERR_INVALID_HANDLE; } break;
        }
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Call back definition of the function that can handle result of Database Hash computation
 *
 * @param[in] aes_status   Execution status
 * @param[in] aes_res      16 bytes block result
 * @param[in] p_buf        Pointer to input buffer used for hash computation
 ****************************************************************************************
 */
__STATIC void gatt_db_hash_res(uint8_t aes_status, const uint8_t* aes_res, co_buf_t* p_buf)
{
    gatt_db_hash_buf_meta_t* p_buf_meta = (gatt_db_hash_buf_meta_t*) co_buf_metadata(p_buf);
    // Copy hash value
    gatt_env.db_hash_valid = true;
    memcpy(gatt_env.db_hash, aes_res, GATT_DB_HASH_LEN);

    // Execute callback
    p_buf_meta->p_cb->cb_db_hash(p_buf_meta->conidx, p_buf_meta->user_lid, p_buf_meta->dummy,
                                 RW_ERR_HCI_TO_HL(aes_status), gatt_env.db_hash);

    // release buffer that handle hash computation
    co_buf_release(p_buf);
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t gatt_db_svc16_add(uint8_t user_lid, uint8_t info, uint16_t uuid16, uint8_t nb_att, const uint8_t* p_att_mask,
                           const gatt_att16_desc_t* p_atts, uint8_t nb_att_rsvd, uint16_t* p_start_hdl)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    gatt_att_desc_t* p_atts_filtered = NULL;

    do
    {
        const gatt_att16_desc_t* p_att_desc;
        uint8_t svc_att_nb = 1;
        uint8_t svc_uuid128[GATT_UUID_128_LEN];
        uint8_t cursor;

        // Check if user is allowed to create service
        if((user_lid >= BLE_GATT_USER_NB) || (gatt_env.users[user_lid].role != GATT_ROLE_SERVER))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // check parameter validity
        if((p_atts == NULL) || (p_start_hdl == NULL) || (nb_att == 0))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // Check service type
        p_att_desc = &(p_atts[0]);

        switch(p_att_desc->uuid16)
        {
            case GATT_DECL_PRIMARY_SERVICE:   { SETB(info, GATT_SVC_TYPE, GATT_SVC_PRIMARY);   } break;
            case GATT_DECL_SECONDARY_SERVICE: { SETB(info, GATT_SVC_TYPE, GATT_SVC_SECONDARY); } break;
            default:                          { status = GAP_ERR_INVALID_PARAM;                } break;
        }
        if(status != GAP_ERR_NO_ERROR) { break; }

        // Get number of attributes in service + Perform some sanity check
        for(cursor = 1 ; cursor < nb_att ; cursor++)
        {
            // check if bit is enabled in mask
            if((p_att_mask == NULL) || ((p_att_mask[cursor >> 3] & CO_BIT(cursor & 0x07)) != 0))
            {
                p_att_desc = &(p_atts[cursor]);

                // Attributes shall not be services (primary or secondary)
                if(   (p_att_desc->uuid16 == GATT_DECL_PRIMARY_SERVICE)
                   || (p_att_desc->uuid16 == GATT_DECL_SECONDARY_SERVICE))
                {
                    status = GAP_ERR_INVALID_PARAM;
                    break;
                }
                // Check permissions of know attributes (CCC, SCC, EXT, CharDecl, ...)
                // Client and Service Configuration
                else if(   (p_att_desc->uuid16 == GATT_DESC_CLIENT_CHAR_CFG)
                        || (p_att_desc->uuid16 == GATT_DESC_SERVER_CHAR_CFG))
                {
                    // Read and Write perm mandatory
                    if(!GETB(p_att_desc->info, GATT_ATT_RD) || !GETB(p_att_desc->info, GATT_ATT_WR))
                    {
                        status = GATT_ERR_INVALID_PERM;
                        break;
                    }
                }

                // increment number of attributes
                svc_att_nb++;
            }
        }
        if(status != GAP_ERR_NO_ERROR) { break; }

        // Allocate service information
        p_atts_filtered = (gatt_att_desc_t*) ke_malloc_user(sizeof(gatt_att_desc_t) * svc_att_nb, KE_MEM_NON_RETENTION);
        if(p_atts_filtered == NULL)
        {
            status = GAP_ERR_INSUFF_RESOURCES;
            break;
        }

        // Prepare attribute array
        svc_att_nb = 0;
        for(cursor = 0 ; cursor < nb_att ; cursor++)
        {
            // check if bit is enabled in mask
            if((p_att_mask == NULL) || ((p_att_mask[cursor >> 3] & CO_BIT(cursor & 0x07)) != 0))
            {
                gatt_att_desc_t* p_att128_new = (&(p_atts_filtered[svc_att_nb]));
                p_att_desc = &(p_atts[cursor]);

                // fill attribute description information
                p_att128_new->info     = p_att_desc->info;
                p_att128_new->ext_info = p_att_desc->ext_info;
                SETF(p_att128_new->info, GATT_ATT_UUID_TYPE, GATT_UUID_16);
                co_write16p(p_att128_new->uuid, p_att_desc->uuid16);

                // ensure that maximum attribute length does not exceed maximum supported length
                switch(p_att_desc->uuid16)
                {
                    case GATT_DESC_CLIENT_CHAR_CFG:
                    case GATT_DESC_SERVER_CHAR_CFG:
                    {
                        // Read and Write perm mandatory
                        SETB(p_att128_new->info, GATT_ATT_RD, true);
                        SETB(p_att128_new->info, GATT_ATT_WR, true);
                        SETF(p_att128_new->ext_info, GATT_ATT_WRITE_MAX_SIZE, sizeof(uint16_t));
                    } break;
                    case GATT_DECL_PRIMARY_SERVICE:
                    case GATT_DECL_SECONDARY_SERVICE:
                    case GATT_DECL_INCLUDE:
                    case GATT_DECL_CHARACTERISTIC:
                    case GATT_DESC_CHAR_EXT_PROPERTIES: { /* Nothing to do */ } break;
                    default:
                    {
                        if(GETF(p_att128_new->ext_info, GATT_ATT_WRITE_MAX_SIZE) > GATT_MAX_VALUE)
                        {
                            SETF(p_att128_new->ext_info, GATT_ATT_WRITE_MAX_SIZE, GATT_MAX_VALUE);
                        }
                    } break;
                }

                svc_att_nb++;
            }
        }

        // Ensure that nb_att_rsvd >= nb_att
        nb_att_rsvd = co_max(svc_att_nb, nb_att_rsvd);
        // Create service
        co_write16p(svc_uuid128, uuid16);
        SETF(info, GATT_SVC_UUID_TYPE, GATT_UUID_16);
        status = gatt_db_svc_create(user_lid, info, svc_uuid128, nb_att_rsvd, svc_att_nb, p_atts_filtered, 0, p_start_hdl);
    } while(0);

    // free attribute array
    if(p_atts_filtered != NULL)
    {
        ke_free(p_atts_filtered);
    }

    return (status);
}

uint16_t gatt_db_svc_add(uint8_t user_lid, uint8_t info, const uint8_t* p_uuid, uint8_t nb_att, const uint8_t* p_att_mask,
                         const gatt_att_desc_t* p_atts, uint8_t nb_att_rsvd, uint16_t* p_start_hdl)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    gatt_att_desc_t* p_atts_filtered = NULL;

    do
    {
        uint8_t svc_att_nb = 1;
        uint8_t svc_uuid128[GATT_UUID_128_LEN];
        uint8_t cursor;
        uint16_t uuid_mem_size = 0;
        const gatt_att_desc_t* p_att_desc;

        // Check if user is allowed to create service
        if((user_lid >= BLE_GATT_USER_NB) || (gatt_env.users[user_lid].role != GATT_ROLE_SERVER))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // check parameter validity
        if((p_atts == NULL) || (p_start_hdl == NULL) || (nb_att == 0))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // Compute size of attribute allocated + number of attributes
        uuid_mem_size = 0;
        memcpy(svc_uuid128, p_uuid, GATT_UUID_128_LEN);

        // Add memory size needed for UUIDs
        switch(GETF(info, GATT_SVC_UUID_TYPE))
        {
            case GATT_UUID_16:  { /* Nothing to do */                  } break;
            case GATT_UUID_32:  { uuid_mem_size += GATT_UUID_32_LEN;   } break;
            case GATT_UUID_128:
            {
                if(gatt_is_uuid16(p_uuid)) // In fact it's a 16-bit UUID
                {
                    SETF(info, GATT_SVC_UUID_TYPE, GATT_UUID_16);
                    memcpy(svc_uuid128, &(p_uuid[12]), GATT_UUID_16_LEN);
                }
                else if(gatt_is_uuid32(p_uuid)) // In fact it's a 32-bit UUID
                {
                    SETF(info, GATT_SVC_UUID_TYPE, GATT_UUID_32);
                    uuid_mem_size += GATT_UUID_32_LEN;
                    memcpy(svc_uuid128, &(p_uuid[12]), GATT_UUID_32_LEN);
                }
                else
                {
                    uuid_mem_size += GATT_UUID_128_LEN;
                }
            } break;
            default:            { status = GAP_ERR_INVALID_PARAM;      } break;
        }

        // Check service type
        p_att_desc = &(p_atts[0]);

        // Primary service
        if(gatt_uuid16_comp(p_att_desc->uuid, GETF(p_att_desc->info, GATT_ATT_UUID_TYPE), GATT_DECL_PRIMARY_SERVICE))
        {
            SETB(info, GATT_SVC_TYPE, GATT_SVC_PRIMARY);
        }
        // Secondary service
        else if(gatt_uuid16_comp(p_att_desc->uuid, GETF(p_att_desc->info, GATT_ATT_UUID_TYPE), GATT_DECL_SECONDARY_SERVICE))
        {
            SETB(info, GATT_SVC_TYPE, GATT_SVC_SECONDARY);
        }
        else
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // Get number of attributes in service
        for(cursor = 1 ; cursor < nb_att ; cursor++)
        {
            // check if bit is enabled in mask
            if((p_att_mask == NULL) || ((p_att_mask[cursor >> 3] & CO_BIT(cursor & 0x07)) != 0))
            {
                // increment number of attributes
                svc_att_nb++;
            }
        }
        if(status != GAP_ERR_NO_ERROR) { break; }

        // Allocate service information
        p_atts_filtered = (gatt_att_desc_t*) ke_malloc_user(sizeof(gatt_att_desc_t) * svc_att_nb, KE_MEM_NON_RETENTION);
        if(p_atts_filtered == NULL)
        {
            status = GAP_ERR_INSUFF_RESOURCES;
            break;
        }

        // Prepare attribute array
        svc_att_nb = 0;
        for(cursor = 0 ; (cursor < nb_att) && (status == GAP_ERR_NO_ERROR) ; cursor++)
        {
            // check if bit is enabled in mask
            if((p_att_mask == NULL) || ((p_att_mask[cursor >> 3] & CO_BIT(cursor & 0x07)) != 0))
            {
                gatt_att_desc_t* p_att_desc_new = (&(p_atts_filtered[svc_att_nb]));
                p_att_desc = &(p_atts[cursor]);

                // fill attribute description information
                p_att_desc_new->ext_info = p_att_desc->ext_info;
                p_att_desc_new->info     = p_att_desc->info;
                memcpy(p_att_desc_new->uuid, p_att_desc->uuid, GATT_UUID_128_LEN);

                // Compute memory size needed for UUIDs - and convert 128-bit UUID to smaller if possible
                switch(GETF(p_att_desc_new->info, GATT_ATT_UUID_TYPE))
                {
                    case GATT_UUID_16:  { /* Nothing to do */                  } break;
                    case GATT_UUID_32:  { uuid_mem_size += GATT_UUID_32_LEN;   } break;
                    case GATT_UUID_128:
                    {
                        if(gatt_is_uuid16(p_uuid)) // In fact it's a 16-bit UUID
                        {
                            SETF(p_att_desc_new->info, GATT_ATT_UUID_TYPE, GATT_UUID_16);
                            memcpy(p_att_desc_new->uuid, &(p_att_desc->uuid[12]), GATT_UUID_16_LEN);
                        }
                        else if(gatt_is_uuid32(p_uuid)) // In fact it's a 32-bit UUID
                        {
                            SETF(p_att_desc_new->info, GATT_ATT_UUID_TYPE, GATT_UUID_32);
                            uuid_mem_size += GATT_UUID_32_LEN;
                            memcpy(p_att_desc_new->uuid, &(p_att_desc->uuid[12]), GATT_UUID_32_LEN);
                        }
                        else
                        {
                            uuid_mem_size += GATT_UUID_128_LEN;
                        }
                    } break;
                    default:            { status = GAP_ERR_INVALID_PARAM;      } break;
                }

                // Perform a sanity check on attribute information
                if(GETF(p_att_desc_new->info, GATT_ATT_UUID_TYPE) == GATT_UUID_16)
                {
                    uint16_t uuid16 = co_read16p(p_att_desc_new->uuid);
                    switch(uuid16)
                    {
                        case GATT_DECL_PRIMARY_SERVICE:
                        case GATT_DECL_SECONDARY_SERVICE:
                        {
                            p_att_desc_new->ext_info = 0;
                            if(cursor != 0) { status = GAP_ERR_INVALID_PARAM; }
                        } break;
                        case GATT_DECL_INCLUDE:
                        case GATT_DECL_CHARACTERISTIC:
                        case GATT_DESC_CHAR_EXT_PROPERTIES: { /* Nothing to do */ } break;
                        case GATT_DESC_CLIENT_CHAR_CFG:
                        case GATT_DESC_SERVER_CHAR_CFG:
                        {
                            // Read and Write perm mandatory
                            SETB(p_att_desc_new->info, GATT_ATT_RD, true);
                            SETB(p_att_desc_new->info, GATT_ATT_WR, true);
                            SETF(p_att_desc_new->ext_info, GATT_ATT_WRITE_MAX_SIZE, sizeof(uint16_t));
                        } break;
                        default:
                        {
                            // ensure that value length does not exceed maximum supported value
                            if(GETF(p_att_desc_new->ext_info, GATT_ATT_WRITE_MAX_SIZE) > GATT_MAX_VALUE)
                            {
                                SETF(p_att_desc_new->ext_info, GATT_ATT_WRITE_MAX_SIZE, GATT_MAX_VALUE);
                            }
                        } break;
                    }
                }
                // ensure that value length does not exceed maximum supported value
                else if(GETF(p_att_desc_new->ext_info, GATT_ATT_WRITE_MAX_SIZE) > GATT_MAX_VALUE)
                {
                    SETF(p_att_desc_new->ext_info, GATT_ATT_WRITE_MAX_SIZE, GATT_MAX_VALUE);
                }

                svc_att_nb++;
            }
        }

        if(status != GAP_ERR_NO_ERROR) break;

        // Ensure that nb_att_rsvd >= nb_att
        nb_att_rsvd = co_max(svc_att_nb, nb_att_rsvd);
        // Create service
        status = gatt_db_svc_create(user_lid, info, svc_uuid128, nb_att_rsvd, svc_att_nb, p_atts_filtered,
                                    uuid_mem_size, p_start_hdl);

    } while(0);

    // free attribute array
    if(p_atts_filtered != NULL)
    {
        ke_free(p_atts_filtered);
    }

    return (status);
}

uint16_t gatt_db_svc_remove(uint8_t user_lid, uint16_t start_hdl)
{
    uint16_t status = ATT_ERR_INVALID_HANDLE;

    gatt_db_svc_t* p_current_svc = gatt_env.p_db;
    gatt_db_svc_t* p_previous_svc = NULL;

    while((p_current_svc != NULL))
    {
        // Search a service with a specific start handle
        if(start_hdl == p_current_svc->start_hdl)
        {
            // check that service can be removed by user
            if(user_lid != p_current_svc->user_lid)
            {
                status = GAP_ERR_COMMAND_DISALLOWED;
                break;
            }

            // remove service from service queue
            if(p_previous_svc == NULL)
            {
                gatt_env.p_db = p_current_svc->p_next;
            }
            else
            {
                p_previous_svc->p_next = p_current_svc->p_next;
            }

            // Inform that database has been updated
            gatt_db_updated(p_current_svc->start_hdl, GATT_DB_END_HDL_GET(p_current_svc));

            // free service information
            ke_free(p_current_svc);
            status = GAP_ERR_NO_ERROR;

            break;
        }
        // else continue to search in database
        else
        {
            p_previous_svc = p_current_svc;
            p_current_svc  = p_current_svc->p_next;
        }
    }

    return (status);
}

void gatt_db_svc_remove_user(uint8_t user_lid)
{
    gatt_db_svc_t* p_current_svc = gatt_env.p_db;
    gatt_db_svc_t* p_previous_svc = NULL;
    gatt_db_svc_t* p_next_svc     = NULL;
    uint8_t start_hdl = GATT_INVALID_HDL;
    uint8_t end_hdl   = GATT_INVALID_HDL;

    while(p_current_svc != NULL)
    {
        p_next_svc = p_current_svc->p_next;

        // Search a service with a specific start handle
        if(user_lid == p_current_svc->user_lid)
        {
            // store service handle range updated
            if(start_hdl == GATT_INVALID_HDL)
            {
                start_hdl = p_current_svc->start_hdl;
            }
            end_hdl = GATT_DB_END_HDL_GET(p_current_svc);

            // remove service from service queue
            if(p_previous_svc == NULL)
            {
                gatt_env.p_db = p_next_svc;
            }
            else
            {
                p_previous_svc->p_next = p_next_svc;
            }

            // free service information
            ke_free(p_current_svc);
        }
        else
        {
            p_previous_svc = p_current_svc;
        }

        p_current_svc  = p_next_svc;
    }

    // Inform that database has been updated
    if(start_hdl != GATT_INVALID_HDL)
    {
        gatt_db_updated(start_hdl, end_hdl);
    }
}

uint16_t gatt_db_handle_range_reserve(uint8_t user_lid, uint8_t nb_att, uint16_t* p_start_hdl)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    bool found = false;
    gatt_db_svc_t** pp_current_svc = &(gatt_env.p_db);

    // Service must start at a specific handle
    if(*p_start_hdl != GATT_INVALID_HDL)
    {
        // search where service shall be put in database since database is a linked list of services sorted by handle. */
        while((!found)&&(*pp_current_svc != NULL))
        {
            // if start handle is less than next services handles, current service is found
            if(*p_start_hdl <= GATT_DB_LAST_HDL_GET(*pp_current_svc))
            {
                found = true;
            }
            // else continue to search in database
            else
            {
                pp_current_svc = &((*pp_current_svc)->p_next);
            }
        }

        if(found)
        {
            // if new database override existing elements, trigger an error.
            if((*p_start_hdl + nb_att) >= (*pp_current_svc)->start_hdl)
            {
                status = ATT_ERR_INVALID_HANDLE;
            }
        }
    }
    else
    {
        // increment start handle to be a valid handle.
        *p_start_hdl += 1;

        // search first available block of handles in database.
        while((!found)&&(*pp_current_svc != NULL))
        {
            // block of free attribute handle is found
            if((*p_start_hdl + nb_att) < (*pp_current_svc)->start_hdl)
            {
                found = true;
            }
            else
            {
                // set new start handle to be after current service
                *p_start_hdl = GATT_DB_LAST_HDL_GET(*pp_current_svc) + 1;
                // update database cursor pointer to continue database search
                pp_current_svc = &((*pp_current_svc)->p_next);
            }
        }
    }

    return (status);
}

uint16_t gatt_db_svc_ctrl(uint8_t user_lid, uint16_t start_hdl, uint8_t enable, uint8_t visible)
{
    uint16_t status = ATT_ERR_INVALID_HANDLE;

    gatt_db_svc_t* p_svc = gatt_db_svc_get(start_hdl);

    // Sanity check on the handle
    if((p_svc != NULL) && (start_hdl == p_svc->start_hdl))
    {
        // check that service can be removed by user
        if(user_lid != p_svc->user_lid)
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
        }
        else
        {
            SETB(p_svc->perm, GATT_SVC_DIS, !enable);

            // check if visibility updated
            if(GETB(p_svc->perm, GATT_SVC_HIDE) == visible)
            {
                // Set service visibility
                SETB(p_svc->perm, GATT_SVC_HIDE, !visible);

                // Mark Database Updated
                gatt_db_updated(p_svc->start_hdl, GATT_DB_END_HDL_GET(p_svc));
            }

            status = GAP_ERR_NO_ERROR;
        }
    }

    return (status);
}

uint16_t gatt_db_hash_get(uint8_t conidx, uint8_t user_lid, uint16_t dummy, const gatt_db_hash_cb_t* p_cb)
{
    uint16_t status = GAP_ERR_NO_ERROR;

    do
    {
        co_buf_t* p_buf = NULL;
        uint8_t* p_hash_in = NULL;
        uint16_t hash_in_len = 0;
        gatt_user_t* p_user = gatt_user_get(user_lid);

        // check user parameters
        if(p_cb == NULL || p_cb->cb_db_hash == NULL)
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }
        else if((p_user == NULL) || (p_user->role != GATT_ROLE_SERVER))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }
        // check if Hash already valid
        if(gatt_env.db_hash_valid)
        {
            // directly return hash value
            p_cb->cb_db_hash(conidx, user_lid, dummy, status, gatt_env.db_hash);
            break;
        }

        while(hash_in_len == 0) // loop twice on service search
        {
            uint8_t  att_cursor;
            uint8_t  nb_att_hdl;
            uint8_t  att_native_val[GATT_DB_ATT_MAX_NATIVE_VAL_LEN];
            gatt_db_att_t *p_att = NULL;
            gatt_db_svc_t* p_svc = gatt_env.p_db;

            // Start service search
            p_svc = gatt_env.p_db;

            // Browse all database to compute hash input
            // The message must be a big number MSB first
            while(p_svc != NULL)
            {
                uint16_t handle = p_svc->start_hdl;
                nb_att_hdl      = p_svc->nb_att;

                // Ignore hidden services
                if(!GETB(p_svc->perm, GATT_SVC_HIDE))
                {

                    // browse all service attributes
                    for(att_cursor = 0 ; att_cursor < nb_att_hdl ; att_cursor++ , handle++)
                    {
                        bool att_val_present = false;
                        p_att = &(p_svc->att[att_cursor]);

                        // only check 16 bits UUID
                        if(GETF(p_att->perm, GATT_ATT_UUID_TYPE) != GATT_UUID_16) continue;

                        // attribute to put in hash input
                        switch(p_att->uuid)
                        {
                            case GATT_DECL_PRIMARY_SERVICE:
                            case GATT_DECL_SECONDARY_SERVICE:
                            case GATT_DECL_INCLUDE:
                            case GATT_DECL_CHARACTERISTIC:
                            case GATT_DESC_CHAR_EXT_PROPERTIES:
                            {
                                att_val_present = true;
                            }
                            // no break
                            case GATT_DESC_CHAR_USER_DESCRIPTION:
                            case GATT_DESC_CLIENT_CHAR_CFG:
                            case GATT_DESC_SERVER_CHAR_CFG:
                            case GATT_DESC_CHAR_PRES_FORMAT:
                            case GATT_DESC_CHAR_AGGREGATE_FORMAT:
                            {

                                if(p_hash_in != NULL)
                                {
                                    p_hash_in -= GATT_HANDLE_LEN;
                                    co_write16p(p_hash_in, co_htons(handle));

                                    p_hash_in -= GATT_UUID_16_LEN;
                                    co_write16p(p_hash_in, co_htons(p_att->uuid));
                                }
                                else
                                {
                                    hash_in_len += GATT_HANDLE_LEN + GATT_UUID_16_LEN;
                                }

                                if(att_val_present)
                                {
                                    uint16_t att_native_val_len;

                                    // Retrieve native attribute value
                                    status = gatt_db_att_native_val_get(handle, p_svc, p_att, att_native_val,
                                                                        &att_native_val_len);
                                    ASSERT_ERR(status == GAP_ERR_NO_ERROR);

                                    if(p_hash_in != NULL)
                                    {
                                        p_hash_in -= att_native_val_len;
                                        co_bswap(p_hash_in, att_native_val, att_native_val_len);
                                    }
                                    else
                                    {
                                        hash_in_len += att_native_val_len;
                                    }
                                }

                            } break;

                            default: { /* Nothing to do */ } break;
                        }
                    }
                }

                // Check next service
                p_svc = p_svc->p_next;
            }

            if(p_hash_in == NULL)
            {
                // Allocate a buffer to prepare Data hash computation
                if(co_buf_alloc(&p_buf, 0, hash_in_len, 0) != CO_BUF_ERR_NO_ERROR)
                {
                    status = ATT_ERR_INSUFF_RESOURCE;
                    break;
                }

                // Provide hash input pointer (which is end of buffer)
                p_hash_in = co_buf_tail(p_buf);

                hash_in_len = 0; // put length to zero to restart loop.
            }
            else
            {
                break;
            }
        }

        if(status == GAP_ERR_NO_ERROR)
        {
            gatt_db_hash_buf_meta_t* p_buf_meta = (gatt_db_hash_buf_meta_t*) co_buf_metadata(p_buf);
            ASSERT_INFO(p_hash_in == co_buf_data(p_buf), p_hash_in, co_buf_data(p_buf));

            p_buf_meta->dummy    = dummy;
            p_buf_meta->p_cb     = p_cb;
            p_buf_meta->user_lid = user_lid;
            p_buf_meta->conidx   = conidx;

            // start computation of database hash
            aes_cmac(gatt_db_hash_key, p_hash_in, co_buf_data_len(p_buf), (aes_func_result_cb) gatt_db_hash_res,
                     (uint32_t) p_buf);
        }
        else if(p_buf != NULL)
        {
            co_buf_release(p_buf);
        }
    } while(0);

    return (status);
}


uint16_t gatt_db_att_info_get(uint8_t user_lid, uint16_t hdl, uint16_t* p_info)
{
    uint16_t status;

    do
    {
        gatt_db_att_t *p_att = NULL;
        gatt_db_svc_t* p_svc = gatt_env.p_db;

        // retrieve attribute
        status = gatt_db_att_get(hdl, &p_svc, &p_att);

        if(status != GAP_ERR_NO_ERROR) break;

        if(p_svc->user_lid != user_lid)
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        *p_info = GETF(p_att->perm, GATT_ATT_INFO);

    } while(0);

    return (status);
}

uint16_t gatt_db_att_info_set(uint8_t user_lid, uint16_t hdl, uint16_t info)
{
    uint16_t status;

    do
    {
#ifdef WIN32
		bool db_updated = false; // peterlee
#else
		bool db_updated;
#endif
        gatt_db_att_t *p_att = NULL;
        gatt_db_svc_t* p_svc = gatt_env.p_db;

        // retrieve attribute
        status = gatt_db_att_get(hdl, &p_svc, &p_att);

        if(status != GAP_ERR_NO_ERROR) break;

        if(p_svc->user_lid != user_lid)
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // only check 16 bits UUID
        if(GETF(p_att->perm, GATT_ATT_UUID_TYPE) == GATT_UUID_16)
        {
            // attribute to put in hash input
            switch(p_att->uuid)
            {
                case GATT_DECL_PRIMARY_SERVICE:
                case GATT_DECL_SECONDARY_SERVICE:
                case GATT_DECL_INCLUDE:
                case GATT_DESC_CHAR_EXT_PROPERTIES:
                case GATT_DECL_CHARACTERISTIC:
                {
                    status = GAP_ERR_INVALID_PARAM;
                } break;
                default: { /* Nothing to do */ } break;
            }

            if(status != GAP_ERR_NO_ERROR) break;
        }

        // Check if current ATT is an attribute value (this means that previous attribute is a characteristic)
        if((hdl - 1) > p_svc->start_hdl)
        {
            // check previous handle
            gatt_db_att_t *p_prev_att = &(p_svc->att[hdl - 1 - p_svc->start_hdl]);

            if(   (GETF(p_prev_att->perm, GATT_ATT_UUID_TYPE) == GATT_UUID_16)
               && (p_att->uuid == GATT_DECL_CHARACTERISTIC))
            {
                // check if database is updated
                if(GETF(p_att->perm, GATT_ATT_PROP) != GETF(info, GATT_ATT_PROP))
                {
                    db_updated = true;
                }
            }
        }

        if(status != GAP_ERR_NO_ERROR) break;

        // update attribute information
        SETF(p_att->perm, GATT_ATT_INFO, GETF(info, GATT_ATT_INFO));

        // Mark database updated
        if(db_updated)
        {
             gatt_db_updated(p_svc->start_hdl, GATT_DB_END_HDL_GET(p_svc));
        }
    } while(0);

    return (status);
}

#if (HOST_MSG_API)
/*
 * MESSAGE HANDLER FUNCTIONS
 ****************************************************************************************
 */
#include "gatt_msg_int.h"

/**
 ****************************************************************************************
 * @brief This function is called when hash value for local attribute database hash has
 *        been computed.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution.
 * @param[in] status        Status of the operation (see enum #hl_err)
 * @param[in] p_hash        Pointer to the 128-bit database hash value
 ****************************************************************************************
 */
__STATIC void gatt_db_hash_result_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status,
                                     const uint8_t* p_hash)
{
    gatt_user_t* p_user = gatt_user_get(user_lid);
    uint16_t dest_task_nbr;
    gatt_db_hash_get_cmp_evt_t* p_cmp_evt;

    dest_task_nbr = p_user->dest_task_nbr;

    // Allocate response message
    p_cmp_evt = KE_MSG_ALLOC(GATT_CMP_EVT, dest_task_nbr ,TASK_GATT, gatt_db_hash_get_cmp_evt);

    if(p_cmp_evt != NULL)
    {
        p_cmp_evt->status = status;
        p_cmp_evt->cmd_code  = GATT_DB_HASH_GET;
        p_cmp_evt->dummy     = dummy;
        p_cmp_evt->user_lid  = user_lid;

        if(status == GAP_ERR_NO_ERROR)
        {
            memcpy(p_cmp_evt->hash, p_hash, GATT_DB_HASH_LEN);
        }

        // send message to host
        ke_msg_send(p_cmp_evt);
    }
}

/// handler of database hash result callback.
__STATIC const gatt_db_hash_cb_t gatt_db_msg_cb =
{
    .cb_db_hash = gatt_db_hash_result_cb,
};


/**
 ****************************************************************************************
 * @brief Handle reception of a Service Add command message
 *
 * @param[in] p_cmd     Pointer to the parameters of the message.
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_db_svc_add_cmd_handler(gatt_db_svc_add_cmd_t* p_cmd, uint16_t src_id)
{
    // allocate response message
    gatt_db_svc_add_cmp_evt_t* p_cmp_evt = KE_MSG_ALLOC(GATT_CMP_EVT, src_id ,TASK_GATT, gatt_db_svc_add_cmp_evt);

    if(p_cmp_evt != NULL)
    {
        p_cmp_evt->status = gatt_db_svc_add(p_cmd->user_lid, p_cmd->info, p_cmd->uuid, p_cmd->nb_att, NULL, p_cmd->atts,
                                            p_cmd->nb_att_rsvd, &(p_cmd->start_hdl));

        p_cmp_evt->cmd_code  = p_cmd->cmd_code;
        p_cmp_evt->dummy     = p_cmd->dummy;
        p_cmp_evt->user_lid  = p_cmd->user_lid;
        p_cmp_evt->start_hdl = p_cmd->start_hdl;

        // send message to host
        ke_msg_send(p_cmp_evt);
    }
}

/**
 ****************************************************************************************
 * @brief Handle reception of a Service Remove command message
 *
 * @param[in] p_cmd     Pointer to the parameters of the message.
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_db_svc_remove_cmd_handler(gatt_db_svc_remove_cmd_t* p_cmd, uint16_t src_id)
{
    uint16_t status = gatt_db_svc_remove(p_cmd->user_lid, p_cmd->start_hdl);
    gatt_msg_send_basic_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, src_id, p_cmd->user_lid, status);
}

/**
 ****************************************************************************************
 * @brief Handle reception of a Service visibility update command message
 *
 * @param[in] p_cmd     Pointer to the parameters of the message.
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_db_svc_ctrl_cmd_handler(gatt_db_svc_ctrl_cmd_t* p_cmd, uint16_t src_id)
{
    uint16_t status = gatt_db_svc_ctrl(p_cmd->user_lid, p_cmd->start_hdl, p_cmd->enable, p_cmd->visible);
    gatt_msg_send_basic_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, src_id, p_cmd->user_lid, status);
}

/**
 ****************************************************************************************
 * @brief Handle reception of a Database Hash compute command message
 *
 * @param[in] p_cmd     Pointer to the parameters of the message.
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_db_hash_get_cmd_handler(gatt_db_hash_get_cmd_t* p_cmd, uint16_t src_id)
{
    uint16_t status;

    status = gatt_db_hash_get(GAP_INVALID_CONIDX, p_cmd->user_lid, p_cmd->dummy, &gatt_db_msg_cb);

    // an error occurs, immediately send back result
    if(status != GAP_ERR_NO_ERROR)
    {
        gatt_db_hash_result_cb(GAP_INVALID_CONIDX, p_cmd->user_lid, p_cmd->dummy, status, (uint8_t*)&src_id);
    }
}


#if (RW_DEBUG)

/**
 ****************************************************************************************
 * @brief Handle request to delete all services in attribute database
 *
 * @param[in] p_cmd     Pointer to the parameters of the message.
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_dbg_db_svc_remove_all_cmd_handler(gatt_dbg_db_svc_remove_all_cmd_t* p_cmd, uint16_t src_id)
{
    uint16_t status = GAP_ERR_NO_ERROR;

    gatt_db_svc_t* p_current_svc = gatt_env.p_db;
    gatt_db_svc_t* p_next_svc = NULL;

    // loop on all services
    while((p_current_svc != NULL))
    {
        p_next_svc = p_current_svc->p_next;

        // free service information
        ke_free(p_current_svc);

        p_current_svc = p_next_svc;
    }

    gatt_env.p_db = NULL;

    // Inform that database has been updated
    gatt_db_updated(GATT_MIN_HDL, GATT_MAX_HDL);

    // send back message with execution status
    gatt_msg_send_basic_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, src_id, GATT_INVALID_USER_LID, status);
}

/**
 ****************************************************************************************
 * @brief Handle request to get list of services present in attribute database
 *
 * @param[in] p_cmd     Pointer to the parameters of the message.
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_dbg_db_svc_list_get_cmd_handler(gatt_dbg_db_svc_list_get_cmd_t* p_cmd, uint16_t src_id)
{
    gatt_dbg_db_svc_list_get_cmp_evt_t *p_cmp_evt;
    gatt_db_svc_t* p_current_svc = gatt_env.p_db;
    uint8_t nb_svc = 0;

    // count number of services
    while((p_current_svc != NULL))
    {
        nb_svc++;
        p_current_svc = p_current_svc->p_next;
    }

    p_cmp_evt = KE_MSG_ALLOC_DYN(GATT_CMP_EVT, src_id ,TASK_GATT, gatt_dbg_db_svc_list_get_cmp_evt,
                                 sizeof(gatt_svc_desc_t) * nb_svc);

    if(p_cmp_evt != NULL)
    {
        uint8_t cursor = 0;
        p_cmp_evt->cmd_code = p_cmd->cmd_code;
        p_cmp_evt->dummy = p_cmd->dummy;
        p_cmp_evt->status = GAP_ERR_NO_ERROR;
        p_cmp_evt->user_lid = GATT_INVALID_USER_LID;
        p_cmp_evt->nb_svc = nb_svc;

        // Fill Service information
        p_current_svc = gatt_env.p_db;
        while((p_current_svc != NULL))
        {
            p_cmp_evt->svcs[cursor].start_hdl = p_current_svc->start_hdl;
            p_cmp_evt->svcs[cursor].end_hdl   = GATT_DB_END_HDL_GET(p_current_svc);
            p_cmp_evt->svcs[cursor].info      = p_current_svc->perm;
            p_cmp_evt->svcs[cursor].user_lid  = p_current_svc->user_lid;

            if(GETF(p_current_svc->perm, GATT_SVC_UUID_TYPE) == GATT_UUID_16)
            {
                memcpy(p_cmp_evt->svcs[cursor].uuid, &(p_current_svc->uuid), GATT_UUID_16_LEN);
            }
            else
            {
                memcpy(p_cmp_evt->svcs[cursor].uuid, ((uint8_t*) p_current_svc) + p_current_svc->uuid, GATT_UUID_128_LEN);
            }

            cursor++;
            p_current_svc = p_current_svc->p_next;
        }

        // Send result
        ke_msg_send(p_cmp_evt);
    }
}

/**
 ****************************************************************************************
 * @brief Handle request to update a service into attribute database
 *
 * @param[in] p_cmd     Pointer to the parameters of the message.
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_dbg_db_svc_info_set_cmd_handler(gatt_dbg_db_svc_info_set_cmd_t* p_cmd, uint16_t src_id)
{
    uint8_t user_lid = GATT_INVALID_USER_LID;
    uint16_t  status = GAP_ERR_NO_ERROR;

    // retrieve Service
    gatt_db_svc_t* p_svc = gatt_db_svc_get(p_cmd->hdl);

    // check if service is available
    if(p_svc == NULL)
    {
        status = ATT_ERR_INVALID_HANDLE;
    }
    else
    {
        user_lid = p_svc->user_lid;

        // update service authentication level
        if(GETF(p_svc->perm, GATT_SVC_AUTH) != GETF(p_cmd->info, GATT_SVC_AUTH))
        {
            SETF(p_svc->perm, GATT_SVC_AUTH, GETF(p_cmd->info, GATT_SVC_AUTH));
        }

        // update key size requirement
        if(GETB(p_svc->perm, GATT_SVC_EKS) != GETB(p_cmd->info, GATT_SVC_EKS))
        {
            SETB(p_svc->perm, GATT_SVC_EKS, GETB(p_cmd->info, GATT_SVC_EKS));
        }

        // update Disable status
        if(GETB(p_svc->perm, GATT_SVC_DIS) != GETB(p_cmd->info, GATT_SVC_DIS))
        {
            SETB(p_svc->perm, GATT_SVC_DIS, GETB(p_cmd->info, GATT_SVC_DIS));
        }
    }

    // send back message with execution status
    gatt_msg_send_basic_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, src_id, user_lid, status);
}

/**
 ****************************************************************************************
 * @brief Handle request to get specific attribute information
 *
 * @param[in] p_cmd     Pointer to the parameters of the message.
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_dbg_db_att_info_get_cmd_handler(gatt_dbg_db_att_info_get_cmd_t* p_cmd, uint16_t src_id)
{
    gatt_dbg_db_att_info_get_cmp_evt_t* p_cmp_evt = KE_MSG_ALLOC(GATT_CMP_EVT, src_id ,TASK_GATT,
                                                                 gatt_dbg_db_att_info_get_cmp_evt);

    if(p_cmp_evt != NULL)
    {
        gatt_db_att_t *p_att = NULL;
        gatt_db_svc_t* p_svc = NULL;

        p_cmp_evt->cmd_code = p_cmd->cmd_code;
        p_cmp_evt->dummy    = p_cmd->dummy;
        p_cmp_evt->user_lid = GATT_INVALID_USER_LID;
        p_cmp_evt->hdl      = p_cmd->hdl;

        p_cmp_evt->status = gatt_db_att_get(p_cmd->hdl, &p_svc, &p_att);

        if( p_cmp_evt->status == GAP_ERR_NO_ERROR)
        {
            p_cmp_evt->att.info     = p_att->perm;
            p_cmp_evt->att.ext_info = p_att->ext_info;

            if(GETF(p_att->perm, GATT_ATT_UUID_TYPE) == GATT_UUID_16)
            {
                memcpy(p_cmp_evt->att.uuid, &(p_att->uuid), GATT_UUID_16_LEN);
            }
            else
            {
                memcpy(p_cmp_evt->att.uuid, ((uint8_t*) p_svc) + p_att->uuid, GATT_UUID_128_LEN);
            }
        }

        // send message to host
        ke_msg_send(p_cmp_evt);
    }
}

/**
 ****************************************************************************************
 * @brief Handle request to update attribute information
 *
 * @param[in] p_cmd     Pointer to the parameters of the message.
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_dbg_db_att_info_set_cmd_handler(gatt_dbg_db_att_info_set_cmd_t* p_cmd, uint16_t src_id)
{
    uint8_t user_lid = GATT_INVALID_USER_LID;
    gatt_db_att_t *p_att = NULL;
    gatt_db_svc_t* p_svc = NULL;

    // retrieve attribute
    uint16_t status = gatt_db_att_get(p_cmd->hdl, &p_svc, &p_att);

    if(status == GAP_ERR_NO_ERROR)
    {
        // retrieve user lid
        user_lid = p_svc->user_lid;
        // update attribute info
        status = gatt_db_att_info_set(user_lid, p_cmd->hdl, p_cmd->info);
    }

    // send back message with execution status
    gatt_msg_send_basic_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, src_id, user_lid, status);
}
#endif // (RW_DEBUG)

#endif // (HOST_MSG_API)

#endif // (BLE_GATT)
/// @} GATT
