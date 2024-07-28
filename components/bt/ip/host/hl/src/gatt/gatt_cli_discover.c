/**
 ****************************************************************************************
 * @file gatt_cli_discover.c
 *
 * @brief  GATT Client Discovery Procedure
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
#include "rwip_config.h"    // IP configuration
#if (BLE_GATT)
#include "gatt.h"           // Native API
#if (BLE_GATT_CLI)
#include "gatt.h"           // Native API
#include "gatt_user.h"      // GATT User API
#include "gatt_proc.h"      // Procedure API
#include "gatt_int.h"       // Internals

#include <string.h>         // for memcmp
#include "co_math.h"        // for co_min
#include "co_endian.h"      // for host to bt number conversion

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/// Put buffer tail length to UUID 128 length
#define GATT_CLI_DISCOVER_BUFFER_TAIL_LEN (GATT_BUFFER_TAIL_LEN + GATT_UUID_128_LEN)

/// Bit field used to retrieve search information
enum gatt_cli_discover_info_bf
{
    /// Perform a discovery by UUID
    GATT_CLI_DISCOVER_BY_UUID_BIT       = 0x01,
    GATT_CLI_DISCOVER_BY_UUID_POS       = 0,
    /// Perform a secondary service search
    GATT_CLI_DISCOVER_SECONDARY_SVC_BIT = 0x02,
    GATT_CLI_DISCOVER_SECONDARY_SVC_POS = 1,
    /// When something is found during discovery, the search is considered as succeed
    GATT_CLI_DISCOVER_SUCCEED_BIT       = 0x04,
    GATT_CLI_DISCOVER_SUCCEED_POS       = 2,
    /// Perform a Full service discovery
    GATT_CLI_DISCOVER_FULL_BIT          = 0x08,
    GATT_CLI_DISCOVER_FULL_POS          = 3,
    /// Discovery UUID type
    GATT_CLI_DISCOVER_UUID_TYPE_MASK    = 0x30,
    GATT_CLI_DISCOVER_UUID_TYPE_LSB     = 4,
    /// Discovery procedure has been canceled
    GATT_CLI_DISCOVER_CANCELED_BIT      = 0x80,
    GATT_CLI_DISCOVER_CANCELED_POS      = 7,
};

/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Full Discovery state information
typedef struct gatt_cli_discover_full
{
    /// Value handle
    uint16_t                    val_hdl;
    /// Next value handle
    uint16_t                    next_val_hdl;
    /// Search Start Handle
    uint16_t                    start_hdl;
    /// Search End Handle
    uint16_t                    end_hdl;
    /// Service Start Handle
    uint16_t                    svc_start_hdl;
    /// Service End Handle
    uint16_t                    svc_end_hdl;
    /// number of attribute in service discovery structure
    uint8_t                     nb_att;
    /// Maximum number of attribute that can be set in service discovery
    uint8_t                     max_nb_att;
    /// Discovery information (see enum #gatt_svc_disc_info)
    uint8_t                     disc_info;
} gatt_cli_discover_full_t;

/// Discovery procedure information
typedef struct gatt_cli_discover_proc
{
    /// Procedure header - required for any Attribute procedure
    gatt_proc_t                 hdr;
    /// Buffer that contains Service information
    co_buf_t*                   p_svc_buf;

    /// Include services handle
    uint16_t                    inc_svc_hdl;
    /// Search Start Handle
    uint16_t                    start_hdl;
    /// Search End Handle
    uint16_t                    end_hdl;
    #if (HOST_MSG_API)
    /// Message command code used.
    uint16_t                    msg_cmd_code;
    #endif // (HOST_MSG_API)
    /// Discover information bit field (see enum #gatt_cli_discover_info_bf);
    uint8_t                     info_bf;
    /// Searched UUID (LSB First)
    uint8_t                     uuid[GATT_UUID_128_LEN];
    /// Pointer to buffer that handle discovery structure
    co_buf_t*                   p_disc_info_buf;
} gatt_cli_discover_proc_t;

/// Buffer meta-data
typedef struct gatt_cli_discover_buf_meta
{
    /// UUID Length present in buffer
    uint8_t uuid_len;
    /// Use to know if buffer reuse has been done
    bool    reuse;
} gatt_cli_discover_buf_meta_t;

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Function called when L2CAP_ATT_FIND_BY_TYPE_RSP attribute PDU is received.
 *
 * @param[in] conidx     Connection index
 * @param[in] p_proc     Pointer to procedure under execution
 * @param[in] p_pdu      Pointer to unpacked PDU
 * @param[in] p_buf      Data of received but not yet extracted
 * @param[in] mtu        MTU size of the bearer
 *
 * @return PDU handling return status that will be provided to the procedure handler.
 ****************************************************************************************
 */
__STATIC uint16_t gatt_cli_l2cap_att_find_by_type_rsp_handler(uint8_t conidx, gatt_cli_discover_proc_t* p_proc,
                                                              l2cap_att_find_by_type_rsp_t* p_pdu,
                                                              co_buf_t* p_buf, uint16_t mtu)
{
    uint16_t status = ATT_ERR_ATTRIBUTE_NOT_FOUND;

    // perform a simple sanity check
    if(co_buf_data_len(p_buf) >= (GATT_HANDLE_LEN*2))
    {
        gatt_cli_discover_buf_meta_t* p_buf_meta =  (gatt_cli_discover_buf_meta_t*) co_buf_metadata(p_buf);
        p_buf_meta->reuse    = false;
        p_buf_meta->uuid_len = 0;

        p_proc->p_svc_buf = p_buf;
        co_buf_acquire(p_buf);

        status = GAP_ERR_NO_ERROR;
    }

    // mark that reception on bearer can continue
    gatt_proc_bearer_rx_continue(conidx, &(p_proc->hdr));

    return (status);
}


/**
 ****************************************************************************************
 * @brief Function called when L2CAP_ATT_RD_BY_GRP_TYPE_RSP attribute PDU is received.
 *
 * @param[in] conidx     Connection index
 * @param[in] p_proc     Pointer to procedure under execution
 * @param[in] p_pdu      Pointer to unpacked PDU
 * @param[in] p_buf      Data of received but not yet extracted
 * @param[in] mtu        MTU size of the bearer
 *
 * @return PDU handling return status that will be provided to the procedure handler.
 ****************************************************************************************
 */
__STATIC uint16_t gatt_cli_l2cap_att_rd_by_grp_type_rsp_handler(uint8_t conidx, gatt_cli_discover_proc_t* p_proc,
                                                                l2cap_att_rd_by_grp_type_rsp_t* p_pdu,
                                                                co_buf_t* p_buf, uint16_t mtu)
{
    uint16_t status = ATT_ERR_ATTRIBUTE_NOT_FOUND;
    uint16_t uuid_len = p_pdu->each_len - (GATT_HANDLE_LEN*2);

    // perform a simple sanity check
    if((uuid_len == GATT_UUID_16_LEN) || (uuid_len == GATT_UUID_128_LEN) || (co_buf_data_len(p_buf) >= p_pdu->each_len))
    {
        gatt_cli_discover_buf_meta_t* p_buf_meta =  (gatt_cli_discover_buf_meta_t*) co_buf_metadata(p_buf);
        p_buf_meta->reuse    = false;
        p_buf_meta->uuid_len = uuid_len;

        p_proc->p_svc_buf = p_buf;
        co_buf_acquire(p_buf);

        status = GAP_ERR_NO_ERROR;
    }

    // mark that reception on bearer can continue
    gatt_proc_bearer_rx_continue(conidx, &(p_proc->hdr));

    return (status);
}

/**
 ****************************************************************************************
 * @brief Function called when L2CAP_ATT_RD_BY_TYPE_RSP attribute PDU is received.
 *
 * @param[in] conidx     Connection index
 * @param[in] p_proc     Pointer to procedure under execution
 * @param[in] p_pdu      Pointer to unpacked PDU
 * @param[in] p_buf      Data of received but not yet extracted
 * @param[in] mtu        MTU size of the bearer
 *
 * @return PDU handling return status that will be provided to the procedure handler.
 ****************************************************************************************
 */
__STATIC uint16_t gatt_cli_discover_l2cap_att_rd_by_type_rsp_handler(uint8_t conidx, gatt_cli_discover_proc_t* p_proc,
                                                                     l2cap_att_rd_by_type_rsp_t* p_pdu,
                                                                     co_buf_t* p_buf, uint16_t mtu)
{
    gatt_user_t* p_user  = gatt_user_get(p_proc->hdr.user_lid);
    uint16_t status = ATT_ERR_ATTRIBUTE_NOT_FOUND;
    uint8_t  proc_id = p_proc->hdr.proc_id;
    uint8_t  exp_info_len = (proc_id == GATT_PROC_DISCOVER_INC_SVC)
                          ? (GATT_HANDLE_LEN * 3)
                          : ((GATT_HANDLE_LEN * 2) + L2CAP_ATT_PROP_LEN);
    uint16_t data_len = co_buf_data_len(p_buf);
    // check if there is some more data to read
    bool     complete = (   (data_len < (mtu - L2CAP_ATT_HEADER_LEN - L2CAP_ATT_EACHLEN_LEN))
            || (p_pdu->each_len < data_len)
            || (p_pdu->each_len > (GATT_UUID_128_LEN + (GATT_HANDLE_LEN * 3)))); // error case

    uint16_t uuid_len = p_pdu->each_len - exp_info_len;

    if(!complete && (proc_id == GATT_PROC_DISCOVER_INC_SVC))
    {
        // store included service handle to read and stop procedure
        p_proc->inc_svc_hdl =  co_btohs(co_read16p(co_buf_data(p_buf)));

        // check if included service handle is in searched range
        if((p_proc->inc_svc_hdl >= p_proc->start_hdl) && (p_proc->inc_svc_hdl <= p_proc->end_hdl))
        {
            status = GAP_ERR_NO_ERROR;
        }
    }
    else
    {
        // ensure that UUID LEN is valid and there is enough data in PDU for a service
        while(   ((uuid_len == GATT_UUID_16_LEN) || (uuid_len == GATT_UUID_128_LEN))
              && co_buf_data_len(p_buf) >= (p_pdu->each_len))
        {
            uint8_t  uuid_type;
            uint8_t  uuid[GATT_UUID_128_LEN];
            uint16_t hdl;
            uint16_t start_hdl = 0;
            uint16_t end_hdl   = 0;
            uint8_t  prop      = 0;

            hdl = co_btohs(co_read16p(co_buf_data(p_buf)));
            co_buf_head_release(p_buf, GATT_HANDLE_LEN);

            if(proc_id == GATT_PROC_DISCOVER_CHAR)
            {
                prop = co_buf_data(p_buf)[0];
                co_buf_head_release(p_buf, L2CAP_ATT_PROP_LEN);
            }

            start_hdl = co_btohs(co_read16p(co_buf_data(p_buf)));
            co_buf_head_release(p_buf, GATT_HANDLE_LEN);

            if(proc_id == GATT_PROC_DISCOVER_INC_SVC)
            {
                end_hdl = co_btohs(co_read16p(co_buf_data(p_buf)));
                co_buf_head_release(p_buf, GATT_HANDLE_LEN);
            }

            // extract UUID of received data
            gatt_uuid_extract(uuid, &uuid_type, co_buf_data(p_buf), uuid_len);
            co_buf_head_release(p_buf, uuid_len);

            // check validity of read handle in searched range
            if((hdl < p_proc->start_hdl) || (hdl > p_proc->end_hdl))
            {
                continue;
            }

            if(proc_id == GATT_PROC_DISCOVER_INC_SVC)
            {
                // perform a simple sanity check
                if(end_hdl >= start_hdl)
                {
                    if(!GETB(p_proc->info_bf, GATT_CLI_DISCOVER_FULL))
                    {
                        // inform user about found service.
                        p_user->p_cb->cli.cb_inc_svc(conidx, p_proc->hdr.user_lid, p_proc->hdr.dummy, /* inc_svc_hdl */hdl,
                                                     start_hdl, end_hdl, uuid_type, uuid);
                        SETB(p_proc->info_bf, GATT_CLI_DISCOVER_SUCCEED, true);
                    }
                    else
                    {
                        gatt_cli_discover_full_t* p_full = (gatt_cli_discover_full_t*) co_buf_metadata(p_proc->p_disc_info_buf);
                        uint16_t att_cursor = hdl - p_full->svc_start_hdl;

                        if(att_cursor < p_full->nb_att)
                        {
                            gatt_svc_att_t* p_att = ((gatt_svc_att_t*) co_buf_data(p_proc->p_disc_info_buf));
                            p_att += att_cursor;

                            p_att->att_type           = GATT_ATT_INCL_SVC;
                            p_att->info.svc.start_hdl = start_hdl;
                            p_att->info.svc.end_hdl   = end_hdl;
                            p_att->uuid_type          = uuid_type;
                            memcpy(p_att->uuid, uuid, GATT_UUID_128_LEN);
                        }
                    }

                    // update start handle used for search
                    p_proc->start_hdl = ((hdl != GATT_MAX_HDL) ? (hdl + 1) : GATT_MAX_HDL);
                    status = GAP_ERR_NO_ERROR;
                }
            }
            else // GATT_PROC_DISCOVER_CHAR
            {
                // accept characteristic for a full discovery or if it's an expected UUID
                if(   !GETB(p_proc->info_bf, GATT_CLI_DISCOVER_BY_UUID)
                   || GETB(p_proc->info_bf, GATT_CLI_DISCOVER_FULL)
                   || gatt_uuid_comp(p_proc->uuid, GETF(p_proc->info_bf, GATT_CLI_DISCOVER_UUID_TYPE), uuid, uuid_type))
                {
                    if(!GETB(p_proc->info_bf, GATT_CLI_DISCOVER_FULL))
                    {
                        // inform user about found characteristic.
                        p_user->p_cb->cli.cb_char(conidx, p_proc->hdr.user_lid, p_proc->hdr.dummy, /* char_hdl */ hdl,
                                                  /* val_hdl */ start_hdl, prop, uuid_type, uuid);
                        SETB(p_proc->info_bf, GATT_CLI_DISCOVER_SUCCEED, true);
                    }
                    else
                    {
                        // fill discovery information
                        gatt_cli_discover_full_t* p_full = (gatt_cli_discover_full_t*) co_buf_metadata(p_proc->p_disc_info_buf);
                        uint16_t att_cursor = hdl - p_full->svc_start_hdl;

                        if(att_cursor < p_full->nb_att)
                        {
                            gatt_svc_att_t* p_att = ((gatt_svc_att_t*) co_buf_data(p_proc->p_disc_info_buf));
                            p_att += att_cursor;

                            p_full->val_hdl       = start_hdl;
                            p_att->att_type            = GATT_ATT_CHAR;
                            p_att->info.charac.val_hdl = start_hdl;
                            p_att->info.charac.prop    = prop;
                            p_att->uuid_type           = uuid_type;
                            memcpy(p_att->uuid, uuid, GATT_UUID_128_LEN);

                            att_cursor = start_hdl - p_full->svc_start_hdl;

                            // Try to update database with value information
                            if(att_cursor < p_full->nb_att)
                            {
                                p_att = ((gatt_svc_att_t*) co_buf_data(p_proc->p_disc_info_buf));
                                p_att += att_cursor;

                                p_att->att_type  = GATT_ATT_VAL;
                                p_att->uuid_type = uuid_type;
                                memcpy(p_att->uuid, uuid, GATT_UUID_128_LEN);
                            }
                            else
                            {
                                // store value handle for next attribute structure
                                p_full->next_val_hdl = start_hdl;
                            }
                        }
                    }
                }

                // update start handle used for search
                p_proc->start_hdl = ((hdl != GATT_MAX_HDL) ? (hdl + 1) : GATT_MAX_HDL);
                status = GAP_ERR_NO_ERROR;
            }
        }
    }

    // mark that reception on bearer can continue
    gatt_proc_bearer_rx_continue(conidx, &(p_proc->hdr));

    return (status);
}


/**
 ****************************************************************************************
 * @brief Function called when L2CAP_ATT_FIND_INFO_RSP attribute PDU is received.
 *
 * @param[in] conidx     Connection index
 * @param[in] p_proc     Pointer to procedure under execution
 * @param[in] p_pdu      Pointer to unpacked PDU
 * @param[in] p_buf      Data of received but not yet extracted
 * @param[in] mtu        MTU size of the bearer
 *
 * @return PDU handling return status that will be provided to the procedure handler.
 ****************************************************************************************
 */
__STATIC uint16_t gatt_cli_l2cap_att_find_info_rsp_handler(uint8_t conidx, gatt_cli_discover_proc_t* p_proc,
                                                            l2cap_att_find_info_rsp_t* p_pdu,
                                                            co_buf_t* p_buf, uint16_t mtu)
{
    gatt_user_t* p_user  = gatt_user_get(p_proc->hdr.user_lid);
    uint16_t status = ATT_ERR_ATTRIBUTE_NOT_FOUND;

    uint8_t  uuid_len = p_pdu->format  == L2CAP_ATT_UUID_16_FORMAT ? GATT_UUID_16_LEN : GATT_UUID_128_LEN;
    uint8_t  each_len = uuid_len + GATT_HANDLE_LEN;


    // ensure that UUID LEN is valid and there is enough data in PDU for a service
    while(   ((uuid_len == GATT_UUID_16_LEN) || (uuid_len == GATT_UUID_128_LEN))
          && co_buf_data_len(p_buf) >= (each_len))
    {
        uint8_t  uuid_type;
        uint8_t  uuid[GATT_UUID_128_LEN];
        uint16_t hdl;

        hdl = co_btohs(co_read16p(co_buf_data(p_buf)));
        co_buf_head_release(p_buf, GATT_HANDLE_LEN);

        // extract UUID of received data
        gatt_uuid_extract(uuid, &uuid_type, co_buf_data(p_buf), uuid_len);
        co_buf_head_release(p_buf, uuid_len);

        // check validity of read handle in searched range
        if((hdl < p_proc->start_hdl) || (hdl > p_proc->end_hdl))
        {
            continue;
        }

        if(!GETB(p_proc->info_bf, GATT_CLI_DISCOVER_FULL))
        {
            // inform user about found descriptor information.
            p_user->p_cb->cli.cb_desc(conidx, p_proc->hdr.user_lid, p_proc->hdr.dummy, /* desc_hdl */ hdl, uuid_type, uuid);
            SETB(p_proc->info_bf, GATT_CLI_DISCOVER_SUCCEED, true);
        }
        else
        {
            // fill discovery information{
            gatt_cli_discover_full_t* p_full = (gatt_cli_discover_full_t*) co_buf_metadata(p_proc->p_disc_info_buf);
            uint16_t att_cursor = hdl - p_full->svc_start_hdl;

            // check if descriptor information not already filled for a value
            if(att_cursor < p_full->nb_att)
            {
                gatt_svc_att_t* p_att = ((gatt_svc_att_t*) co_buf_data(p_proc->p_disc_info_buf));
                p_att += att_cursor;

                if(p_att->att_type == GATT_ATT_NONE)
                {
                    p_att->att_type = (p_full->val_hdl == hdl) ? GATT_ATT_VAL : GATT_ATT_DESC;
                    p_att->uuid_type = uuid_type;
                    memcpy(p_att->uuid, uuid, GATT_UUID_128_LEN);
                }
            }
        }

        // update start handle used for search
        p_proc->start_hdl = ((hdl != GATT_MAX_HDL) ? (hdl + 1) : GATT_MAX_HDL);
        status = GAP_ERR_NO_ERROR;
    }

    // mark that reception on bearer can continue
    gatt_proc_bearer_rx_continue(conidx, &(p_proc->hdr));

    return (status);
}



/**
 ****************************************************************************************
 * @brief Function called when L2CAP_ATT_RD_RSP attribute PDU is received.
 *
 * @param[in] conidx     Connection index
 * @param[in] p_proc     Pointer to procedure under execution
 * @param[in] p_pdu      Pointer to unpacked PDU
 * @param[in] p_buf      Data of received but not yet extracted
 * @param[in] mtu        MTU size of the bearer
 *
 * @return PDU handling return status that will be provided to the procedure handler.
 ****************************************************************************************
 */
__STATIC uint16_t gatt_cli_discover_l2cap_att_rd_rsp_handler(uint8_t conidx, gatt_cli_discover_proc_t* p_proc,
                                                             l2cap_att_rd_rsp_t* p_pdu, co_buf_t* p_buf, uint16_t mtu)
{
    gatt_user_t* p_user  = gatt_user_get(p_proc->hdr.user_lid);
    uint16_t status = ATT_ERR_ATTRIBUTE_NOT_FOUND;

    uint16_t uuid_len = co_buf_data_len(p_buf) - (GATT_HANDLE_LEN * 2);

    // ensure that UUID LEN is valid and there is enough data in PDU for a service
    if((uuid_len == GATT_UUID_16_LEN) || (uuid_len == GATT_UUID_128_LEN))
    {
        uint8_t  uuid_type;
        uint8_t  uuid[GATT_UUID_128_LEN];
        uint16_t start_hdl;
        uint16_t end_hdl;

        start_hdl = co_btohs(co_read16p(co_buf_data(p_buf)));
        co_buf_head_release(p_buf, GATT_HANDLE_LEN);

        end_hdl = co_btohs(co_read16p(co_buf_data(p_buf)));
        co_buf_head_release(p_buf, GATT_HANDLE_LEN);

        // extract UUID of received data
        gatt_uuid_extract(uuid, &uuid_type, co_buf_data(p_buf), uuid_len);
        co_buf_head_release(p_buf, uuid_len);

        // perform a simple sanity check
        if(end_hdl >= start_hdl)
        {

            if(!GETB(p_proc->info_bf, GATT_CLI_DISCOVER_FULL))
            {
                // inform user about found service.
                p_user->p_cb->cli.cb_inc_svc(conidx, p_proc->hdr.user_lid, p_proc->hdr.dummy, p_proc->inc_svc_hdl,
                                             start_hdl, end_hdl, uuid_type, uuid);
                SETB(p_proc->info_bf, GATT_CLI_DISCOVER_SUCCEED, true);
            }
            else
            {
                gatt_cli_discover_full_t* p_full = (gatt_cli_discover_full_t*) co_buf_metadata(p_proc->p_disc_info_buf);
                uint16_t att_cursor = p_proc->inc_svc_hdl - p_full->svc_start_hdl;

                if(att_cursor < p_full->nb_att)
                {
                    gatt_svc_att_t* p_att = ((gatt_svc_att_t*) co_buf_data(p_proc->p_disc_info_buf));
                    p_att += att_cursor;

                    p_att->att_type           = GATT_ATT_INCL_SVC;
                    p_att->info.svc.start_hdl = start_hdl;
                    p_att->info.svc.start_hdl = end_hdl;
                    p_att->uuid_type          = uuid_type;
                    memcpy(p_att->uuid, uuid, GATT_UUID_128_LEN);
                }
            }

            // update start handle used for search
            p_proc->start_hdl = ((p_proc->inc_svc_hdl != GATT_MAX_HDL) ? (p_proc->inc_svc_hdl + 1) : GATT_MAX_HDL);
            // ensure that we don't use read request to continue included service discovery
            p_proc->inc_svc_hdl = GATT_INVALID_HDL;
            status = GAP_ERR_NO_ERROR;
        }
    }

    // mark that reception on bearer can continue
    gatt_proc_bearer_rx_continue(conidx, &(p_proc->hdr));

    return (status);
}

/**
 ****************************************************************************************
 * @brief Function called to extract service information from service buffer
 *
 * @param[in] conidx     Connection index
 * @param[in] p_proc     Pointer to procedure under execution
 * @param[in] p_user     Pointer to GATT user information
 *
 * @return PDU handling return status that will be provided to the procedure handler.
 ****************************************************************************************
 */
__STATIC uint16_t gatt_cli_discover_svc_extract(uint8_t conidx, gatt_cli_discover_proc_t* p_proc, gatt_user_t* p_user)
{
    uint16_t status = ATT_ERR_ATTRIBUTE_NOT_FOUND;
    co_buf_t* p_buf = p_proc->p_svc_buf;
    gatt_cli_discover_buf_meta_t* p_buf_meta =  (gatt_cli_discover_buf_meta_t*) co_buf_metadata(p_buf);
    uint8_t each_len;
    uint8_t uuid_type;
    uint8_t uuid[GATT_UUID_128_LEN];

    each_len = p_buf_meta->uuid_len + (GATT_HANDLE_LEN*2);

    // if UUID isn't present, use searched one
    if(p_buf_meta->uuid_len == 0)
    {
        uuid_type = GETF(p_proc->info_bf, GATT_CLI_DISCOVER_UUID_TYPE);
        memcpy(uuid, p_proc->uuid, GATT_UUID_128_LEN);
    }

    // ensure  there is enough data in PDU for a service
    while(co_buf_data_len(p_buf) >= each_len)
    {
        uint16_t start_hdl;
        uint16_t end_hdl;

        start_hdl = co_btohs(co_read16p(co_buf_data(p_buf)));
        co_buf_head_release(p_buf, GATT_HANDLE_LEN);
        end_hdl = co_btohs(co_read16p(co_buf_data(p_buf)));
        co_buf_head_release(p_buf, GATT_HANDLE_LEN);

        if(p_buf_meta->uuid_len > 0)
        {
            // extract UUID of received data
            gatt_uuid_extract(uuid, &uuid_type, co_buf_data(p_buf), p_buf_meta->uuid_len);
            co_buf_head_release(p_buf, p_buf_meta->uuid_len);
        }

        // perform a simple sanity check
        if((start_hdl >= p_proc->start_hdl) && (end_hdl >= start_hdl))
        {
            // update start handle used for search
            p_proc->start_hdl = ((end_hdl != GATT_MAX_HDL) ? (end_hdl + 1) : GATT_MAX_HDL);

            status = GAP_ERR_NO_ERROR;

            if(!GETB(p_proc->info_bf, GATT_CLI_DISCOVER_FULL))
            {
                // inform user about found service.
                p_user->p_cb->cli.cb_svc_info(conidx, p_proc->hdr.user_lid, p_proc->hdr.dummy, start_hdl, end_hdl,
                                              uuid_type, uuid);

                SETB(p_proc->info_bf, GATT_CLI_DISCOVER_SUCCEED, true);
            }
            else
            {
                uint8_t cursor;
                gatt_svc_att_t* p_atts = (gatt_svc_att_t*) co_buf_data(p_proc->p_disc_info_buf);
                uint8_t nb_att = end_hdl - start_hdl + 1;
                gatt_cli_discover_full_t* p_full = (gatt_cli_discover_full_t*) co_buf_metadata(p_proc->p_disc_info_buf);

                // split structure if number of attribute is too big
                if(nb_att > p_full->max_nb_att)
                {
                    nb_att = p_full->max_nb_att;
                    p_full->disc_info = GATT_SVC_START;
                }
                else
                {
                    p_full->disc_info = GATT_SVC_CMPLT;
                }

                // fill service information
                p_atts[0].att_type  = GETB(p_proc->info_bf, GATT_CLI_DISCOVER_SECONDARY_SVC)
                                    ? GATT_ATT_SECONDARY_SVC
                                    : GATT_ATT_PRIMARY_SVC;
                p_atts[0].uuid_type =  uuid_type;
                memcpy(p_atts[0].uuid, uuid, GATT_UUID_128_LEN);
                p_atts[0].info.svc.start_hdl = start_hdl;
                p_atts[0].info.svc.end_hdl   = end_hdl;

                // initialize structure
                for(cursor = 1 ; cursor < nb_att ; cursor++)
                {
                    p_atts[cursor].att_type = GATT_ATT_NONE;
                    memset(p_atts[cursor].uuid, 0, GATT_UUID_128_LEN);
                }

                // Store state of service discovery
                p_full->start_hdl     = p_proc->start_hdl;
                p_full->end_hdl       = p_proc->end_hdl;
                p_full->svc_start_hdl = start_hdl;
                p_full->svc_end_hdl   = end_hdl;
                p_full->nb_att        = nb_att;
                p_full->next_val_hdl  = GATT_INVALID_HDL;
                p_full->val_hdl       = GATT_INVALID_HDL;

                // update procedure state to search for included services
                p_proc->hdr.proc_id        = GATT_PROC_DISCOVER_INC_SVC;
                p_proc->start_hdl          = p_full->svc_start_hdl+1;
                p_proc->end_hdl            = p_full->svc_end_hdl;

                break;
            }
        }
    }

    // check if buffer usage is over
    if(co_buf_data_len(p_buf) < each_len)
    {
        co_buf_release(p_buf);
        p_proc->p_svc_buf = NULL;
    }
    else if(!p_buf_meta->reuse)
    {
        uint8_t uuid_len = p_buf_meta->uuid_len;

        // force buffer to be reused
        co_buf_reuse(p_buf);
        p_buf_meta =  (gatt_cli_discover_buf_meta_t*) co_buf_metadata(p_buf);

        // reset environment since it can be present in a different memory pointer
        p_buf_meta->reuse    = true;
        p_buf_meta->uuid_len = uuid_len;
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Function called to update internal step of FULL service discovery
 *
 * @param[in]  conidx     Connection index
 * @param[in]  p_proc     Pointer to procedure under execution
 * @param[in]  p_user     Pointer to GATT user information
 * @param[out] p_finished True to finish the procedure, else procedure is not over.
 *
 * @return function execution status (@see hl_err)
 ****************************************************************************************
 */
__STATIC uint16_t gatt_cli_discover_full_state_update(uint8_t conidx, gatt_cli_discover_proc_t* p_proc,
                                                      gatt_user_t* p_user, bool* p_finished)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    uint8_t  proc_id = p_proc->hdr.proc_id;
    gatt_cli_discover_full_t* p_full = (gatt_cli_discover_full_t*) co_buf_metadata(p_proc->p_disc_info_buf);


    // discovery is over
    if(proc_id == GATT_PROC_DISCOVER_SVC)
    {
        *p_finished = true;
    }
    // all phase of one service discovery is over
    else if((proc_id == GATT_PROC_DISCOVER_DESC) || (p_full->svc_start_hdl == p_full->svc_end_hdl))
    {
        gatt_svc_att_t* p_atts = (gatt_svc_att_t*) co_buf_data(p_proc->p_disc_info_buf);

        // inform user of service information discovered
        p_user->p_cb->cli.cb_svc(conidx, p_proc->hdr.user_lid, p_proc->hdr.dummy,
                                 p_full->svc_start_hdl, p_full->disc_info,
                                 p_full->nb_att, p_atts);

        // update service discovery cursor
        p_full->svc_start_hdl += p_full->nb_att;
        if(p_full->svc_start_hdl == GATT_INVALID_HDL)
        {
            p_full->svc_start_hdl = GATT_MAX_HDL;
        }

        // check if current service discovery is over
        if(p_full->svc_start_hdl < p_full->svc_end_hdl)
        {
            uint8_t cursor;
            uint8_t nb_att = p_full->svc_end_hdl - p_full->svc_start_hdl + 1;

            // split structure if number of attribute is too big
            if(nb_att > p_full->max_nb_att)
            {
                nb_att = p_full->max_nb_att;
                p_full->disc_info = GATT_SVC_CONT;
            }
            else
            {
                p_full->disc_info = GATT_SVC_END;
            }

            // initialize structure
            for(cursor = 0 ; cursor < nb_att ; cursor++)
            {
                p_atts[cursor].att_type = GATT_ATT_NONE;
                memset(p_atts[cursor].uuid, 0, GATT_UUID_128_LEN);
            }

            // Store state of service discovery
            p_full->nb_att        = nb_att;
            p_full->val_hdl       = p_full->next_val_hdl;
            p_full->next_val_hdl  = GATT_INVALID_HDL;

            p_proc->hdr.proc_id        = GATT_PROC_DISCOVER_INC_SVC;
            p_proc->start_hdl          = p_full->svc_start_hdl;
            p_proc->end_hdl            = p_full->svc_end_hdl;
        }
        else
        {
            // Mark discovery succeed
            SETB(p_proc->info_bf, GATT_CLI_DISCOVER_SUCCEED, true);

            // continue service discovery
            p_proc->hdr.proc_id        = GATT_PROC_DISCOVER_SVC;
            p_proc->start_hdl          = p_full->start_hdl;
            p_proc->end_hdl            = p_full->end_hdl;

            // more data to extract from service
            if(p_proc->p_svc_buf != NULL)
            {
                // extract service information that remains
                if(gatt_cli_discover_svc_extract(conidx, p_proc, p_user) == GAP_ERR_INSUFF_RESOURCES)
                {
                    status = GAP_ERR_INSUFF_RESOURCES;
                    *p_finished = true;
                }
            }

            // check if service discovery is over
            if((p_proc->start_hdl > p_proc->end_hdl) || (p_proc->start_hdl == GATT_MAX_HDL))
            {
                *p_finished = true;
            }
            // else, just continue discovery
        }
    }
    // include service discovery is over
    else if (proc_id == GATT_PROC_DISCOVER_INC_SVC)
    {
        // search for characteristic
        p_proc->hdr.proc_id = GATT_PROC_DISCOVER_CHAR;
        p_proc->start_hdl   = p_full->svc_start_hdl;
        p_proc->end_hdl     = p_full->svc_end_hdl;
    }
    //  characteristic discovery is over
    else //  (proc_id == GATT_PROC_DISCOVER_CHAR)
    {
        // search for descriptors
        p_proc->hdr.proc_id = GATT_PROC_DISCOVER_DESC;
        p_proc->start_hdl   = p_full->svc_start_hdl;
        p_proc->end_hdl     = p_full->svc_end_hdl;
    }

    return (status);
}


/**
 ****************************************************************************************
 * @brief function called when operation state is updated
 *
 * @param[in] conidx     Connection index
 * @param[in] p_proc     Pointer to procedure to continue
 * @param[in] proc_state Operation transition state (see enum #gatt_proc_state)
 * @param[in] status     Execution status
 ****************************************************************************************
 */
__STATIC void gatt_cli_discover_proc_continue(uint8_t conidx, gatt_cli_discover_proc_t* p_proc, uint8_t proc_state, uint16_t status)
{
    gatt_user_t* p_user  = gatt_user_get(p_proc->hdr.user_lid);
    bool finished = false;

    switch(proc_state)
    {
        case GATT_PROC_PDU_RX:
        {
            // Update Status code to cancel
            if(GETB(p_proc->info_bf, GATT_CLI_DISCOVER_CANCELED))
            {
                status = GAP_ERR_CANCELED;
                finished = true;
                break;
            }
            // check if there is something to extract from a service discovery
            else if((p_proc->hdr.proc_id == GATT_PROC_DISCOVER_SVC) && (p_proc->p_svc_buf != NULL))
            {
                // extract service information
                status = gatt_cli_discover_svc_extract(conidx, p_proc, p_user);
            }

            // Check if discovery is over
            if(   (status != GAP_ERR_NO_ERROR) || GETB(p_proc->info_bf, GATT_CLI_DISCOVER_CANCELED)
               || (p_proc->start_hdl > p_proc->end_hdl) || (p_proc->start_hdl == GATT_MAX_HDL))
            {

                // check if for full discovery search procedure can continue
                if(GETB(p_proc->info_bf, GATT_CLI_DISCOVER_FULL))
                {
                    // update state og the full discovery procedure
                    status = gatt_cli_discover_full_state_update(conidx, p_proc, p_user, &finished);
                }
                else
                {
                    finished = true;
                }

                // stop if procedure is over
                if(finished) break;
            }
        }
        // no break;
        case GATT_PROC_START:
        {
            gatt_proc_pdu_handler_cb cb_rsp_handler = NULL;
            l2cap_att_pdu_t pdu;
            co_buf_t* p_buf = NULL;
            bool by_uuid = GETB(p_proc->info_bf, GATT_CLI_DISCOVER_BY_UUID);

            // allocate buffer used for attribute transmission
            if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, 0, GATT_CLI_DISCOVER_BUFFER_TAIL_LEN) != CO_BUF_ERR_NO_ERROR)
            {
                status = GAP_ERR_INSUFF_RESOURCES;
                break;
            }

            // set search handle range (valid for all searched PDUs)
            pdu.rd_by_type_req.shdl = p_proc->start_hdl;
            pdu.rd_by_type_req.ehdl = p_proc->end_hdl;

            switch(p_proc->hdr.proc_id)
            {
                case GATT_PROC_DISCOVER_SVC:
                {
                    uint16_t searched_uuid16 = GETB(p_proc->info_bf, GATT_CLI_DISCOVER_SECONDARY_SVC)
                                             ? GATT_DECL_SECONDARY_SERVICE
                                             : GATT_DECL_PRIMARY_SERVICE;

                    // Set Searched UUID
                    co_buf_tail_reserve(p_buf, GATT_UUID_16_LEN);
                    co_write16p(co_buf_data(p_buf), searched_uuid16);

                    if(by_uuid)
                    {
                        cb_rsp_handler = (gatt_proc_pdu_handler_cb) gatt_cli_l2cap_att_find_by_type_rsp_handler;
                        pdu.code       = L2CAP_ATT_FIND_BY_TYPE_REQ_OPCODE;

                        // Copy UUID expected
                        if(GETF(p_proc->info_bf, GATT_CLI_DISCOVER_UUID_TYPE) == GATT_UUID_16)
                        {
                            memcpy(co_buf_tail(p_buf), p_proc->uuid, GATT_UUID_16_LEN);
                            co_buf_tail_reserve(p_buf, GATT_UUID_16_LEN);
                        }
                        else
                        {
                            // convert UUID to 128-bit UUID
                            gatt_uuid128_convert(p_proc->uuid, GETF(p_proc->info_bf, GATT_CLI_DISCOVER_UUID_TYPE),
                                                 co_buf_tail(p_buf));
                            co_buf_tail_reserve(p_buf, GATT_UUID_128_LEN);
                        }
                    }
                    else
                    {
                        cb_rsp_handler = (gatt_proc_pdu_handler_cb) gatt_cli_l2cap_att_rd_by_grp_type_rsp_handler;
                        pdu.code       = L2CAP_ATT_RD_BY_GRP_TYPE_REQ_OPCODE;
                    }
                } break;

                case GATT_PROC_DISCOVER_INC_SVC:
                {
                    if(p_proc->inc_svc_hdl == GATT_INVALID_HDL)
                    {
                        cb_rsp_handler = (gatt_proc_pdu_handler_cb) gatt_cli_discover_l2cap_att_rd_by_type_rsp_handler;
                        pdu.code       = L2CAP_ATT_RD_BY_TYPE_REQ_OPCODE;
                        co_buf_tail_reserve(p_buf, GATT_UUID_16_LEN);
                        co_write16p(co_buf_data(p_buf), GATT_DECL_INCLUDE);
                    }
                    else
                    {
                        cb_rsp_handler    = (gatt_proc_pdu_handler_cb) gatt_cli_discover_l2cap_att_rd_rsp_handler;
                        pdu.code          = L2CAP_ATT_RD_REQ_OPCODE;
                        pdu.rd_req.handle = p_proc->inc_svc_hdl;
                    }
                } break;

                case GATT_PROC_DISCOVER_CHAR:
                {
                    cb_rsp_handler = (gatt_proc_pdu_handler_cb) gatt_cli_discover_l2cap_att_rd_by_type_rsp_handler;
                    pdu.code       = L2CAP_ATT_RD_BY_TYPE_REQ_OPCODE;
                    co_buf_tail_reserve(p_buf, GATT_UUID_16_LEN);
                    co_write16p(co_buf_data(p_buf), GATT_DECL_CHARACTERISTIC);
                } break;

                case GATT_PROC_DISCOVER_DESC:
                {
                    cb_rsp_handler = (gatt_proc_pdu_handler_cb) gatt_cli_l2cap_att_find_info_rsp_handler;
                    pdu.code       = L2CAP_ATT_FIND_INFO_REQ_OPCODE;
                } break;

                default: { ASSERT_ERR(0); } break;
            }

            // Ask for PDU transmission
            status = gatt_proc_pdu_send(conidx, &(p_proc->hdr), &pdu, p_buf, cb_rsp_handler);

            // release buffer
            co_buf_release(p_buf);
        } break;
        case GATT_PROC_ERROR:
        {
            finished = true;
        } break;

        default: { /*  Nothing to do */  } break;
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        finished = true;
    }

    if(finished)
    {
        // check if service discovery information must be released
        if(GETB(p_proc->info_bf, GATT_CLI_DISCOVER_FULL))
        {
            gatt_env.total_mem_size -= co_buf_size(p_proc->p_disc_info_buf);
            co_buf_release(p_proc->p_disc_info_buf);
        }

        // check if buffer must be released
        if(p_proc->p_svc_buf != NULL)
        {
            co_buf_release(p_proc->p_svc_buf);
        }

        // update status according to discover result
        if ((status == ATT_ERR_ATTRIBUTE_NOT_FOUND) ||(status == GAP_ERR_NO_ERROR))
        {
            status = GETB(p_proc->info_bf, GATT_CLI_DISCOVER_SUCCEED) ? GAP_ERR_NO_ERROR : ATT_ERR_ATTRIBUTE_NOT_FOUND;
        }

        // Inform user that procedure is over
        #if (HOST_MSG_API)
        gatt_proc_cur_set((gatt_proc_t*) p_proc);
        #endif // (HOST_MSG_API)
        p_user->p_cb->cli.cb_discover_cmp(conidx, p_proc->hdr.user_lid, p_proc->hdr.dummy, status);

        // Pop Procedure
        gatt_proc_pop(conidx, &(p_proc->hdr), true);
    }
}


/**
 ****************************************************************************************
 * @brief Create the internal discovery procedure
 *
 * @param[in]  conidx        Connection index
 * @param[in]  user_lid      GATT User Local identifier
 * @param[in]  dummy         Dummy parameter whose meaning is upper layer dependent and
 *                           which is returned in command complete.
 * @param[in]  proc_id       Procedure Identifier (see enum #gatt_proc_id)
 * @param[in]  secondary_svc Secondary Service discovery (false: primary service discovery)
 * @param[in]  start_hdl     Search start handle
 * @param[in]  end_hdl       Search end handle
 * @param[in]  by_uuid       True: Perform a search for a specific UUID
 * @param[in]  uuid_type     UUID Type (see enum #gatt_uuid_type)
 * @param[in]  p_uuid        Pointer to searched attribute UUID (LSB First)
 *
 * @return Status of the function execution (see enum #hl_err)
 *         Consider status only if an error occurs; else wait for execution completion
 ****************************************************************************************
 */
__STATIC uint16_t gatt_cli_discover_proc_create(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint8_t proc_id,
                                                bool secondary_svc, uint16_t start_hdl, uint16_t end_hdl,
                                                bool by_uuid, uint8_t uuid_type, const uint8_t* p_uuid)
{
    uint16_t     status = GAP_ERR_NO_ERROR;
    gatt_user_t* p_user  = gatt_user_get(user_lid);
    bool         full_svc_discovery = false;
    co_buf_t*    p_disc_info_buf = NULL;

    // Ensure that user can handle the procedure
    if((p_user == NULL) || (p_user->p_cb->cli.cb_discover_cmp == NULL))
    {
        status = GAP_ERR_NOT_SUPPORTED;
    }
    else
    {
        void* p_cb = NULL;

        switch(proc_id)
        {
            case GATT_PROC_DISCOVER_SVC:     { p_cb = (void*) p_user->p_cb->cli.cb_svc_info; } break;
            case GATT_PROC_DISCOVER_INC_SVC: { p_cb = (void*) p_user->p_cb->cli.cb_inc_svc;  } break;
            case GATT_PROC_DISCOVER_CHAR:    { p_cb = (void*) p_user->p_cb->cli.cb_char;     } break;
            case GATT_PROC_DISCOVER_DESC:    { p_cb = (void*) p_user->p_cb->cli.cb_desc;     } break;
            case GATT_PROC_DISCOVER_FULL:    { p_cb = (void*) p_user->p_cb->cli.cb_svc;      } break;
            default:                         { ASSERT_ERR(0);                                } break;
        }

        // check that a callback exist for requested discovery
        if(p_cb == NULL)
        {
            status = GAP_ERR_NOT_SUPPORTED;
        }

        // for a full discovery, change the procedure identifier.
        if(proc_id == GATT_PROC_DISCOVER_FULL)
        {
            full_svc_discovery = true;
            proc_id            = GATT_PROC_DISCOVER_SVC;

            // allocate a buffer that will contain discovered services (and that will be kept during discovery)
            if(   (gatt_env.total_mem_size > GATT_MEM_LIMIT)
               || (co_buf_alloc(&p_disc_info_buf, 0, sizeof(gatt_svc_att_t) * GATT_DISCOVER_SVC_ATT_MAX, 0) != CO_BUF_ERR_NO_ERROR))
            {
                status = GAP_ERR_INSUFF_RESOURCES;
            }
            else
            {
                gatt_env.total_mem_size += co_buf_size(p_disc_info_buf);
                DBG_MEM_INIT(co_buf_data(p_disc_info_buf), gatt_env.total_mem_size);
            }
        }
    }

    if(status == GAP_ERR_NO_ERROR)
    {
        gatt_cli_discover_proc_t* p_proc;

        // Create procedure
        status = gatt_proc_create(conidx, user_lid, dummy, proc_id, L2CAP_LE_MTU_MIN, sizeof(gatt_cli_discover_proc_t),
                                  (gatt_proc_cb) gatt_cli_discover_proc_continue, (gatt_proc_t**) &p_proc);

        if(status == GAP_ERR_NO_ERROR)
        {
            // fill discovery parameters
            p_proc->start_hdl   = start_hdl;
            p_proc->end_hdl     = end_hdl;
            p_proc->info_bf     = 0;
            p_proc->inc_svc_hdl = GATT_INVALID_HDL;
            p_proc->p_svc_buf   = NULL;

            SETB(p_proc->info_bf, GATT_CLI_DISCOVER_SECONDARY_SVC, secondary_svc);
            SETB(p_proc->info_bf, GATT_CLI_DISCOVER_FULL, full_svc_discovery);

            if(by_uuid)
            {
                SETB(p_proc->info_bf, GATT_CLI_DISCOVER_BY_UUID,   true);
                SETF(p_proc->info_bf, GATT_CLI_DISCOVER_UUID_TYPE, uuid_type);
                memcpy(p_proc->uuid, p_uuid, GATT_UUID_128_LEN);
            }

            if(full_svc_discovery)
            {
                gatt_cli_discover_full_t* p_full = (gatt_cli_discover_full_t*) co_buf_metadata(p_disc_info_buf);

                p_proc->p_disc_info_buf = p_disc_info_buf;
                // compute number of attribute that can be put in buffer
                p_full->max_nb_att = (co_buf_data_len(p_disc_info_buf) + co_buf_tail_len(p_disc_info_buf))
                                   / sizeof(gatt_svc_att_t);
            }

            #if (HOST_MSG_API)
            gatt_proc_cur_set((gatt_proc_t*) p_proc);
            #endif // (HOST_MSG_API)

            // ask procedure to be granted - push it in wait list
            gatt_proc_push(conidx, &(p_proc->hdr));
        }
        // in case of error remove service discovery buffer
        else if (p_disc_info_buf != NULL)
        {
            gatt_env.total_mem_size -= co_buf_size(p_disc_info_buf);
            co_buf_release(p_disc_info_buf);
        }

    }

    return (status);
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */


uint16_t gatt_cli_discover_svc(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint8_t disc_type, bool full,
                               uint16_t start_hdl, uint16_t end_hdl, uint8_t uuid_type, const uint8_t* p_uuid)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    // Check parameters
    if((uuid_type > GATT_UUID_128) || (start_hdl > end_hdl) || (disc_type > GATT_DISCOVER_SVC_SECONDARY_BY_UUID))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else
    {
        bool secondary_svc = ((disc_type & GATT_DISCOVER_SVC_SECONDARY_ALL)   != 0);
        bool by_uuid       = ((disc_type & GATT_DISCOVER_SVC_PRIMARY_BY_UUID) != 0);

        // Create procedure
        status = gatt_cli_discover_proc_create(conidx, user_lid, dummy,
                                               (full ? GATT_PROC_DISCOVER_FULL : GATT_PROC_DISCOVER_SVC),
                                               secondary_svc, start_hdl, end_hdl, by_uuid, uuid_type, p_uuid);
    }

    return (status);
}

uint16_t gatt_cli_discover_inc_svc(uint8_t conidx, uint8_t user_lid, uint16_t dummy,
                                   uint16_t start_hdl, uint16_t end_hdl)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    // Check parameters
    if((start_hdl > end_hdl))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else
    {
        // Create procedure
        status = gatt_cli_discover_proc_create(conidx, user_lid, dummy, GATT_PROC_DISCOVER_INC_SVC, false,
                                               start_hdl, end_hdl, false, 0, NULL);
    }

    return (status);
}

uint16_t gatt_cli_discover_char(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint8_t disc_type,
                                uint16_t start_hdl, uint16_t end_hdl, uint8_t uuid_type, const uint8_t* p_uuid)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    // Check parameters
    if((uuid_type > GATT_UUID_128) || (start_hdl > end_hdl) || (disc_type > GATT_DISCOVER_SVC_SECONDARY_BY_UUID))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else
    {
        // Create procedure
        status = gatt_cli_discover_proc_create(conidx, user_lid, dummy, GATT_PROC_DISCOVER_CHAR, false, start_hdl, end_hdl,
                                               (disc_type == GATT_DISCOVER_CHAR_BY_UUID), uuid_type, p_uuid);
    }

    return (status);
}

uint16_t gatt_cli_discover_desc(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t start_hdl, uint16_t end_hdl)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    // Check parameters
    if((start_hdl > end_hdl))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else
    {
        // Create procedure
        status = gatt_cli_discover_proc_create(conidx, user_lid, dummy, GATT_PROC_DISCOVER_DESC, false,
                                               start_hdl, end_hdl, false, 0, NULL);
    }

    return (status);
}

uint16_t gatt_cli_discover_cancel(uint8_t conidx, uint8_t user_lid, uint16_t dummy)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    gatt_cli_discover_proc_t* p_proc = (gatt_cli_discover_proc_t*) gatt_proc_find(conidx, dummy);

    if((p_proc != NULL) && (p_proc->hdr.user_lid == user_lid))
    {
        switch(p_proc->hdr.proc_id)
        {
            case GATT_PROC_DISCOVER_SVC:
            case GATT_PROC_DISCOVER_INC_SVC:
            case GATT_PROC_DISCOVER_CHAR:
            case GATT_PROC_DISCOVER_DESC:
            case GATT_PROC_DISCOVER_FULL:
            {
                status = GAP_ERR_NO_ERROR;

                SETB(p_proc->info_bf, GATT_CLI_DISCOVER_CANCELED, true);

                // if procedure not yet started
                if(GETF(p_proc->hdr.info_bf, GATT_PROC_STATE) == GATT_PROC_WAIT_GRANT)
                {
                    // Request an immediate procedure abort
                    gatt_proc_continue(conidx, &(p_proc->hdr), GATT_PROC_PDU_RX, GAP_ERR_CANCELED);
                }
            } break;
            default:                         { /* Nothing to do */  } break;
        }
    }

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
 * @brief Handle Service discovery command from GATT client user
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_cli_discover_svc_cmd_handler(gatt_cli_discover_svc_cmd_t* p_cmd, uint16_t src_id)
{
    // start discovery
    uint16_t status = gatt_cli_discover_svc(p_cmd->conidx, p_cmd->user_lid, p_cmd->dummy, p_cmd->disc_type, p_cmd->full,
                                            p_cmd->start_hdl, p_cmd->end_hdl, p_cmd->uuid_type, p_cmd->uuid);

    if(status != GAP_ERR_NO_ERROR)
    {
        // send command completion - in error
        gatt_msg_send_proc_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, p_cmd->conidx, src_id, p_cmd->user_lid, status);
    }
    else
    {
        // store command code
        gatt_cli_discover_proc_t* p_proc = (gatt_cli_discover_proc_t*) gatt_proc_cur_get();
        p_proc->msg_cmd_code = p_cmd->cmd_code;
    }
}

/**
 ****************************************************************************************
 * @brief Handle Include Service discovery command from GATT client user
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_cli_discover_inc_svc_cmd_handler(gatt_cli_discover_inc_svc_cmd_t* p_cmd, uint16_t src_id)
{
    // start discovery
    uint16_t status = gatt_cli_discover_inc_svc(p_cmd->conidx, p_cmd->user_lid, p_cmd->dummy,
                                                p_cmd->start_hdl, p_cmd->end_hdl);

    if(status != GAP_ERR_NO_ERROR)
    {
        // send command completion - in error
        gatt_msg_send_proc_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, p_cmd->conidx, src_id, p_cmd->user_lid, status);
    }
    else
    {
        // store command code
        gatt_cli_discover_proc_t* p_proc = (gatt_cli_discover_proc_t*) gatt_proc_cur_get();
        p_proc->msg_cmd_code = p_cmd->cmd_code;
    }
}

/**
 ****************************************************************************************
 * @brief Handle Characteristic discovery command from GATT client user
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_cli_discover_char_cmd_handler(gatt_cli_discover_char_cmd_t* p_cmd, uint16_t src_id)
{
    // start discovery
    uint16_t status = gatt_cli_discover_char(p_cmd->conidx, p_cmd->user_lid, p_cmd->dummy, p_cmd->disc_type,
                                             p_cmd->start_hdl, p_cmd->end_hdl, p_cmd->uuid_type, p_cmd->uuid);

    if(status != GAP_ERR_NO_ERROR)
    {
        // send command completion - in error
        gatt_msg_send_proc_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, p_cmd->conidx, src_id, p_cmd->user_lid, status);
    }
    else
    {
        // store command code
        gatt_cli_discover_proc_t* p_proc = (gatt_cli_discover_proc_t*) gatt_proc_cur_get();
        p_proc->msg_cmd_code = p_cmd->cmd_code;
    }
}

/**
 ****************************************************************************************
 * @brief Handle Attribute Descriptor discovery command from GATT client user
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_cli_discover_desc_cmd_handler(gatt_cli_discover_desc_cmd_t* p_cmd, uint16_t src_id)
{
    // start discovery
    uint16_t status = gatt_cli_discover_desc(p_cmd->conidx, p_cmd->user_lid, p_cmd->dummy,
                                             p_cmd->start_hdl, p_cmd->end_hdl);

    if(status != GAP_ERR_NO_ERROR)
    {
        // send command completion - in error
        gatt_msg_send_proc_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, p_cmd->conidx, src_id, p_cmd->user_lid, status);
    }
    else
    {
        // store command code
        gatt_cli_discover_proc_t* p_proc = (gatt_cli_discover_proc_t*) gatt_proc_cur_get();
        p_proc->msg_cmd_code = p_cmd->cmd_code;
    }
}


/**
 ****************************************************************************************
 * @brief Handle Cancellation of discovery command from GATT client user
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_cli_discover_cancel_cmd_handler(gatt_cli_discover_cancel_cmd_t* p_cmd, uint16_t src_id)
{
    // cancel discovery
    uint16_t status = gatt_cli_discover_cancel(p_cmd->conidx, p_cmd->user_lid, p_cmd->dummy);
    // send command completion
    gatt_msg_send_proc_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, p_cmd->conidx, src_id, p_cmd->user_lid, status);
}

/**
 ****************************************************************************************
 * @brief This function is called when GATT client user discovery procedure is over.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] status        Status of the procedure (see enum #hl_err)
 ****************************************************************************************
 */
void gatt_cli_discover_cmp_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status)
{
    gatt_user_t* p_user = gatt_user_get(user_lid);

    if(p_user != NULL)
    {
        gatt_cli_discover_proc_t* p_proc = (gatt_cli_discover_proc_t*) gatt_proc_cur_get();

        // send back command complete event
        gatt_msg_send_proc_cmp_evt(p_proc->msg_cmd_code, dummy, conidx, p_user->dest_task_nbr, user_lid, status);
    }
}

/**
 ****************************************************************************************
 * @brief This function is called when a full service has been found during a discovery procedure.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] hdl           First handle value of following list
 * @param[in] disc_info     Discovery information (see enum #gatt_svc_disc_info)
 * @param[in] nb_att        Number of attributes
 * @param[in] p_atts        Pointer to attribute information present in a service
 ****************************************************************************************
 */
void gatt_cli_svc_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t hdl, uint8_t disc_info,
                     uint8_t nb_att, const gatt_svc_att_t* p_atts)
{
    gatt_user_t* p_user = gatt_user_get(user_lid);

    if(p_user != NULL)
    {
        gatt_cli_svc_ind_t* p_ind = KE_MSG_ALLOC_DYN(GATT_IND, p_user->dest_task_nbr, TASK_GATT, gatt_cli_svc_ind,
                                                     sizeof(gatt_svc_att_t) * nb_att);

        // prepare indication to send
        if(p_ind != NULL)
        {
            p_ind->ind_code  = GATT_CLI_SVC;
            p_ind->dummy     = dummy;
            p_ind->user_lid  = user_lid;
            p_ind->conidx    = conidx;

            p_ind->hdl       = hdl;
            p_ind->disc_info = disc_info;
            p_ind->nb_att    = nb_att;

            memcpy(p_ind->atts, p_atts, sizeof(gatt_svc_att_t) * nb_att);

            // send message to host
            ke_msg_send(p_ind);
        }
    }
}

/**
 ****************************************************************************************
 * @brief This function is called when a service has been found during a discovery procedure.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] start_hdl     Service start handle
 * @param[in] end_hdl       Service end handle
 * @param[in] uuid_type     UUID Type (see enum #gatt_uuid_type)
 * @param[in] p_uuid        Pointer to service UUID (LSB first)
 ****************************************************************************************
 */
void gatt_cli_svc_info_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t start_hdl,
                          uint16_t end_hdl, uint8_t uuid_type, const uint8_t* p_uuid)
{
    gatt_user_t* p_user = gatt_user_get(user_lid);

    if(p_user != NULL)
    {
        gatt_cli_svc_info_ind_t* p_ind = KE_MSG_ALLOC(GATT_IND, p_user->dest_task_nbr, TASK_GATT, gatt_cli_svc_info_ind);

        // prepare indication to send
        if(p_ind != NULL)
        {
            p_ind->ind_code  = GATT_CLI_SVC_INFO;
            p_ind->dummy     = dummy;
            p_ind->user_lid  = user_lid;
            p_ind->conidx    = conidx;
            p_ind->start_hdl = start_hdl;
            p_ind->end_hdl   = end_hdl;
            p_ind->uuid_type = uuid_type;
            memcpy(p_ind->uuid, p_uuid, GATT_UUID_128_LEN);

            // send message to host
            ke_msg_send(p_ind);
        }
    }
}

/**
 ****************************************************************************************
 * @brief This function is called when an include service has been found during a discovery procedure.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] inc_svc_hdl   Include service attribute handle
 * @param[in] start_hdl     Service start handle
 * @param[in] end_hdl       Service end handle
 * @param[in] uuid_type     UUID Type (see enum #gatt_uuid_type)
 * @param[in] p_uuid        Pointer to service UUID (LSB first)
 ****************************************************************************************
 */
void gatt_cli_inc_svc_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t inc_svc_hdl,
                         uint16_t start_hdl, uint16_t end_hdl, uint8_t uuid_type, const uint8_t* p_uuid)
{
    gatt_user_t* p_user = gatt_user_get(user_lid);

    if(p_user != NULL)
    {
        gatt_cli_inc_svc_ind_t* p_ind = KE_MSG_ALLOC(GATT_IND, p_user->dest_task_nbr, TASK_GATT, gatt_cli_inc_svc_ind);

        // prepare indication to send
        if(p_ind != NULL)
        {
            p_ind->ind_code    = GATT_CLI_INC_SVC;
            p_ind->dummy       = dummy;
            p_ind->user_lid    = user_lid;
            p_ind->conidx      = conidx;
            p_ind->inc_svc_hdl = inc_svc_hdl;
            p_ind->start_hdl   = start_hdl;
            p_ind->end_hdl     = end_hdl;
            p_ind->uuid_type   = uuid_type;
            memcpy(p_ind->uuid, p_uuid, GATT_UUID_128_LEN);

            // send message to host
            ke_msg_send(p_ind);
        }
    }
}
/**
 ****************************************************************************************
 * @brief This function is called when a characteristic has been found during a discovery procedure.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] char_hdl      Characteristic attribute handle
 * @param[in] val_hdl       Value handle
 * @param[in] prop          Characteristic properties (see enum #gatt_att_info_bf - bits [0-7])
 * @param[in] uuid_type     UUID Type (see enum #gatt_uuid_type)
 * @param[in] p_uuid        Pointer to characteristic value UUID (LSB first)
 ****************************************************************************************
 */
void gatt_cli_char_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t char_hdl, uint16_t val_hdl,
                      uint8_t prop, uint8_t uuid_type, const uint8_t* p_uuid)
{
    gatt_user_t* p_user = gatt_user_get(user_lid);

    if(p_user != NULL)
    {
        gatt_cli_char_ind_t* p_ind = KE_MSG_ALLOC(GATT_IND, p_user->dest_task_nbr, TASK_GATT, gatt_cli_char_ind);

        // prepare indication to send
        if(p_ind != NULL)
        {
            p_ind->ind_code    = GATT_CLI_CHAR;
            p_ind->dummy       = dummy;
            p_ind->user_lid    = user_lid;
            p_ind->conidx      = conidx;
            p_ind->char_hdl    = char_hdl;
            p_ind->val_hdl     = val_hdl;
            p_ind->prop        = prop;
            p_ind->uuid_type   = uuid_type;
            memcpy(p_ind->uuid, p_uuid, GATT_UUID_128_LEN);

            // send message to host
            ke_msg_send(p_ind);
        }
    }
}
/**
 ****************************************************************************************
 * @brief This function is called when a descriptor has been found during a discovery procedure.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] desc_hdl      Characteristic descriptor attribute handle
 * @param[in] uuid_type     UUID Type (see enum #gatt_uuid_type)
 * @param[in] p_uuid        Pointer to attribute UUID (LSB first)
 ****************************************************************************************
 */
void gatt_cli_desc_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t desc_hdl,
                      uint8_t uuid_type, const uint8_t* p_uuid)
{
    gatt_user_t* p_user = gatt_user_get(user_lid);

    if(p_user != NULL)
    {
        gatt_cli_desc_ind_t* p_ind = KE_MSG_ALLOC(GATT_IND, p_user->dest_task_nbr, TASK_GATT, gatt_cli_desc_ind);

        // prepare indication to send
        if(p_ind != NULL)
        {
            p_ind->ind_code    = GATT_CLI_DESC;
            p_ind->dummy       = dummy;
            p_ind->user_lid    = user_lid;
            p_ind->conidx      = conidx;
            p_ind->desc_hdl    = desc_hdl;
            p_ind->uuid_type   = uuid_type;
            memcpy(p_ind->uuid, p_uuid, GATT_UUID_128_LEN);

            // send message to host
            ke_msg_send(p_ind);
        }
    }
}

#endif // (HOST_MSG_API)


#else  // !(BLE_GATT_CLI)

uint16_t gatt_cli_discover_svc(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint8_t disc_type, bool full,
                               uint16_t start_hdl, uint16_t end_hdl, uint8_t uuid_type, const uint8_t* p_uuid)
{
    return (GAP_ERR_NOT_SUPPORTED);
}

uint16_t gatt_cli_discover_inc_svc(uint8_t conidx, uint8_t user_lid, uint16_t dummy,
                                   uint16_t start_hdl, uint16_t end_hdl)
{
    return (GAP_ERR_NOT_SUPPORTED);
}

uint16_t gatt_cli_discover_char(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint8_t disc_type,
                                uint16_t start_hdl, uint16_t end_hdl, uint8_t uuid_type, const uint8_t* p_uuid)
{
    return (GAP_ERR_NOT_SUPPORTED);
}

uint16_t gatt_cli_discover_desc(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t start_hdl, uint16_t end_hdl)
{
    return (GAP_ERR_NOT_SUPPORTED);
}

uint16_t gatt_cli_discover_cancel(uint8_t conidx, uint8_t user_lid, uint16_t dummy)
{
    return (GAP_ERR_NOT_SUPPORTED);
}


#endif // (BLE_GATT_CLI)
#endif // (BLE_GATT)
/// @} GATT

