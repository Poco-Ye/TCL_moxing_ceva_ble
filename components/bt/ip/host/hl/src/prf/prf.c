/**
 ****************************************************************************************
 *
 * @file prf.c
 *
 * @brief Entry point of profile source file.
 *
 * Used to manage life cycle of profiles
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup PRF
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (HOST_PROFILES)
#include "rwip.h"
#include "prf.h"
#include "prf_utils.h"
#include "../inc/prf_hl_api.h"
#include "gatt.h"

#include <string.h>

#if (BLE_HT_THERMOM)
extern const prf_task_cbs_t* htpt_prf_itf_get(void);
#endif // (BLE_HT_THERMOM)

#if (BLE_HT_COLLECTOR)
extern const prf_task_cbs_t* htpc_prf_itf_get(void);
#endif // (BLE_HT_COLLECTOR)

#if (BLE_DIS_SERVER)
extern const prf_task_cbs_t* diss_prf_itf_get(void);
#endif // (BLE_HT_THERMOM)

#if (BLE_DIS_CLIENT)
extern const prf_task_cbs_t* disc_prf_itf_get(void);
#endif // (BLE_DIS_CLIENT)

#if (BLE_BP_SENSOR)
extern const prf_task_cbs_t* blps_prf_itf_get(void);
#endif // (BLE_BP_SENSOR)

#if (BLE_BP_COLLECTOR)
extern const prf_task_cbs_t* blpc_prf_itf_get(void);
#endif // (BLE_BP_COLLECTOR)

#if (BLE_TIP_SERVER)
extern const prf_task_cbs_t* tips_prf_itf_get(void);
#endif // (BLE_TIP_SERVER)

#if (BLE_TIP_CLIENT)
extern const prf_task_cbs_t* tipc_prf_itf_get(void);
#endif // (BLE_TIP_CLIENT)

#if (BLE_HR_SENSOR)
extern const prf_task_cbs_t* hrps_prf_itf_get(void);
#endif // (BLE_HR_SENSOR)

#if (BLE_HR_COLLECTOR)
extern const prf_task_cbs_t* hrpc_prf_itf_get(void);
#endif // (BLE_HR_COLLECTOR)

#if (BLE_FINDME_LOCATOR)
extern const prf_task_cbs_t* findl_prf_itf_get(void);
#endif // (BLE_FINDME_LOCATOR)

#if (BLE_FINDME_TARGET)
extern const prf_task_cbs_t* findt_prf_itf_get(void);
#endif // (BLE_FINDME_TARGET)

#if (BLE_PROX_MONITOR)
extern const prf_task_cbs_t* proxm_prf_itf_get(void);
#endif // (BLE_PROX_MONITOR)

#if (BLE_PROX_REPORTER)
extern const prf_task_cbs_t* proxr_prf_itf_get(void);
#endif // (BLE_PROX_REPORTER)

#if (BLE_SP_CLIENT)
extern const prf_task_cbs_t* scppc_prf_itf_get(void);
#endif // (BLE_SP_CLENT)

#if (BLE_SP_SERVER)
extern const prf_task_cbs_t* scpps_prf_itf_get(void);
#endif // (BLE_SP_SERVER)

#if (BLE_BATT_CLIENT)
extern const prf_task_cbs_t* basc_prf_itf_get(void);
#endif // (BLE_BATT_CLIENT)

#if (BLE_BATT_SERVER)
extern const prf_task_cbs_t* bass_prf_itf_get(void);
#endif // (BLE_BATT_SERVER)

#if (BLE_HID_DEVICE)
extern const prf_task_cbs_t* hogpd_prf_itf_get(void);
#endif // (BLE_HID_DEVICE)

#if (BLE_GATV_SERVER)
extern const prf_task_cbs_t *gatvs_prf_itf_get(void);
#endif

#if (BLE_HID_BOOT_HOST)
extern const prf_task_cbs_t* hogpbh_prf_itf_get(void);
#endif // (BLE_HID_BOOT_HOST)

#if (BLE_HID_REPORT_HOST)
extern const prf_task_cbs_t* hogprh_prf_itf_get(void);
#endif // (BLE_HID_REPORT_HOST)

#if (BLE_GL_COLLECTOR)
extern const prf_task_cbs_t* glpc_prf_itf_get(void);
#endif // (BLE_GL_COLLECTOR)

#if (BLE_GL_SENSOR)
extern const prf_task_cbs_t* glps_prf_itf_get(void);
#endif // (BLE_GL_SENSOR)

#if (BLE_RSC_COLLECTOR)
extern const prf_task_cbs_t* rscpc_prf_itf_get(void);
#endif // (BLE_RSC_COLLECTOR)

#if (BLE_RSC_SENSOR)
extern const prf_task_cbs_t* rscps_prf_itf_get(void);
#endif // (BLE_RSC_COLLECTOR)

#if (BLE_CSC_COLLECTOR)
extern const prf_task_cbs_t* cscpc_prf_itf_get(void);
#endif // (BLE_CSC_COLLECTOR)

#if (BLE_CSC_SENSOR)
extern const prf_task_cbs_t* cscps_prf_itf_get(void);
#endif // (BLE_CSC_COLLECTOR)

#if (BLE_AN_CLIENT)
extern const prf_task_cbs_t* anpc_prf_itf_get(void);
#endif // (BLE_AN_CLIENT)

#if (BLE_AN_SERVER)
extern const prf_task_cbs_t* anps_prf_itf_get(void);
#endif // (BLE_AN_SERVER)

#if (BLE_PAS_CLIENT)
extern const prf_task_cbs_t* paspc_prf_itf_get(void);
#endif // (BLE_PAS_CLIENT)

#if (BLE_PAS_SERVER)
extern const prf_task_cbs_t* pasps_prf_itf_get(void);
#endif // (BLE_PAS_SERVER)

#if (BLE_CP_COLLECTOR)
extern const prf_task_cbs_t* cppc_prf_itf_get(void);
#endif //(BLE_CP_COLLECTOR)

#if (BLE_CP_SENSOR)
extern const prf_task_cbs_t* cpps_prf_itf_get(void);
#endif //(BLE_CP_SENSOR)

#if (BLE_LN_COLLECTOR)
extern const prf_task_cbs_t* lanc_prf_itf_get(void);
#endif //(BLE_CP_COLLECTOR)

#if (BLE_LN_SENSOR)
extern const prf_task_cbs_t* lans_prf_itf_get(void);
#endif //(BLE_CP_SENSOR)

#if (BLE_IPS_SERVER)
extern const prf_task_cbs_t* ipss_prf_itf_get(void);
#endif //(BLE_IPS_SERVER)

#if (BLE_IPS_CLIENT)
extern const prf_task_cbs_t* ipsc_prf_itf_get(void);
#endif //(BLE_IPS_CLIENT)

#if (BLE_ENV_SERVER)
extern const prf_task_cbs_t* envs_prf_itf_get(void);
#endif //(BLE_ENV_SERVER)

#if (BLE_ENV_CLIENT)
extern const prf_task_cbs_t* envc_prf_itf_get(void);
#endif //(BLE_ENV_CLIENT

#if (BLE_WSC_SERVER)
extern const prf_task_cbs_t* wscs_prf_itf_get(void);
#endif //(BLE_WSC_SERVER)

#if (BLE_WSC_CLIENT)
extern const prf_task_cbs_t* wscc_prf_itf_get(void);
#endif //(BLE_WSC_CLIENT

#if (BLE_BCS_SERVER)
extern const prf_task_cbs_t* bcss_prf_itf_get(void);
#endif //(BLE_BCS_SERVER)

#if (BLE_BCS_CLIENT)
extern const prf_task_cbs_t* bcsc_prf_itf_get(void);
#endif //(BLE_BCS_CLIENT)

#if (BLE_WPT_SERVER)
extern const prf_task_cbs_t* wpts_prf_itf_get(void);
#endif //(BLE_WPT_SERVER)

#if (BLE_WPT_CLIENT)
extern const prf_task_cbs_t* wptc_prf_itf_get(void);
#endif //(BLE_WPT_CLIENT

#if (BLE_PLX_SERVER)
extern const prf_task_cbs_t* plxs_prf_itf_get(void);
#endif //(BLE_PLX_SERVER)

#if (BLE_PLX_CLIENT)
extern const prf_task_cbs_t* plxc_prf_itf_get(void);
#endif //(BLE_PLX_CLIENT

#if (BLE_CGM_SERVER)
extern const prf_task_cbs_t* cgms_prf_itf_get(void);
#endif //(BLE_CGM_SERVER)

#if (BLE_CGM_CLIENT)
extern const prf_task_cbs_t* cgmc_prf_itf_get(void);
#endif //(BLE_CGM_CLIENT

#if (BLE_ISO_MODE_0_PROFILE)
extern const prf_task_cbs_t* am0_prf_itf_get(void);
#endif // (BLE_ISO_MODE_0_PROFILE)

#if (BLE_UDS_SERVER)
extern const prf_task_cbs_t* udss_prf_itf_get(void);
#endif //(BLE_UDS_SERVER)

#if (BLE_UDS_CLIENT)
extern const prf_task_cbs_t* udsc_prf_itf_get(void);
#endif //(BLE_UDS_SERVER)

#if (BLE_CSIS_MEMBER)
extern const prf_task_cbs_t* csism_prf_itf_get(void);
#endif //(BLE_CSIS_MEMBER)

#if (BLE_CSIS_COORD)
extern const prf_task_cbs_t* csisc_prf_itf_get(void);
#endif //(BLE_CSIS_COORD)

#if (BLE_OT_SERVER)
extern const prf_task_cbs_t* ots_prf_itf_get(void);
#endif //(BLE_OT_SERVER)

#if (BLE_OT_CLIENT)
extern const prf_task_cbs_t* otc_prf_itf_get(void);
#endif //(BLE_OT_CLIENT)

#if (BLE_DBG_THPP)
extern const prf_task_cbs_t* thpp_prf_itf_get(void);
#endif //(BLE_DBG_THPP)

#if (BLE_MESH)
extern const prf_task_cbs_t* mal_prf_itf_get(void);
#endif // (BLE_MESH)

#if (BT_A2DP_SINK)
extern const prf_task_cbs_t* a2dp_sink_prf_itf_get(void);
#endif //(BT_A2DP_SINK)

#if (BT_A2DP_SOURCE)
extern const prf_task_cbs_t* a2dp_source_prf_itf_get(void);
#endif //(BT_A2DP_SOURCE)

#if (BT_AVRCP)
extern const prf_task_cbs_t* avrcp_prf_itf_get(void);
#endif //(BT_AVRCP)

#if (BT_HFP_AG)
extern const prf_task_cbs_t* hfp_ag_prf_itf_get(void);
#endif //(BT_HFP_AG)

#if (BT_HFP_HF)
extern const prf_task_cbs_t* hfp_hf_prf_itf_get(void);
#endif //(BT_HFP_HF)

#if (BT_SPP)
extern const prf_task_cbs_t* spp_prf_itf_get(void);
#endif //(BT_SPP)

#if (BT_PBAP_CLIENT)
extern const prf_task_cbs_t* pbap_client_prf_itf_get(void);
#endif //(BT_PBAP_CLIENT)

#if (APPLE_IAP)
extern const prf_task_cbs_t* apple_iap_prf_itf_get(void);
#endif //(APPLE_IAP)

#if (GFP_PROVIDER)
extern const prf_task_cbs_t* gfp_provider_prf_itf_get(void);
#endif // (GFP_PROVIDER)
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Profile interface structure
typedef const prf_task_cbs_t* (*prf_task_itf) (void);

/*
 * DEFINES
 ****************************************************************************************
 */
/// Maximum number of instance per profile must be 1
#define PRF_TASK_IDX_MAX     (1)

/// Invalid profile Index
#define PRF_IDX_INVALID      (0xFF)

/*
 * MACROS
 ****************************************************************************************
 */
#define PRF_ITF(name, itf) [(PRF_ID_##name)] = (itf)

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Profile Manager environment structure
typedef struct prf_env_
{
    /// Array of profile tasks that can be managed by Profile manager.
    prf_data_t prf[HOST_NB_PROFILES];
} prf_env_t;

prf_env_t prf_env;

/// Profile interface structure
__STATIC const prf_task_itf prf_itf[PRF_ID_MAX] =
{
        #if (BLE_HT_THERMOM)
        PRF_ITF(HTPT, htpt_prf_itf_get),
        #endif // (BLE_HT_THERMOM)

        #if (BLE_HT_COLLECTOR)
        PRF_ITF(HTPC, htpc_prf_itf_get),
        #endif // (BLE_HT_COLLECTOR)

        #if (BLE_DIS_SERVER)
        PRF_ITF(DISS, diss_prf_itf_get),
        #endif // (BLE_DIS_SERVER)

        #if (BLE_DIS_CLIENT)
        PRF_ITF(DISC, disc_prf_itf_get),
        #endif // (BLE_DIS_CLIENT)

        #if (BLE_BP_SENSOR)
        PRF_ITF(BLPS, blps_prf_itf_get),
        #endif // (BLE_BP_SENSOR)

        #if (BLE_BP_COLLECTOR)
        PRF_ITF(BLPC, blpc_prf_itf_get),
        #endif // (BLE_BP_COLLECTOR)

        #if (BLE_TIP_SERVER)
        PRF_ITF(TIPS, tips_prf_itf_get),
        #endif // (BLE_TIP_SERVER)

        #if (BLE_TIP_CLIENT)
        PRF_ITF(TIPC, tipc_prf_itf_get),
        #endif // (BLE_TIP_CLIENT)

        #if (BLE_HR_SENSOR)
        PRF_ITF(HRPS, hrps_prf_itf_get),
        #endif // (BLE_HR_SENSOR)

        #if (BLE_HR_COLLECTOR)
        PRF_ITF(HRPC, hrpc_prf_itf_get),
        #endif // (BLE_HR_COLLECTOR)

        #if (BLE_FINDME_LOCATOR)
        PRF_ITF(FINDL, findl_prf_itf_get),
        #endif // (BLE_FINDME_LOCATOR)

        #if (BLE_FINDME_TARGET)
        PRF_ITF(FINDT, findt_prf_itf_get),
        #endif // (BLE_FINDME_TARGET)

        #if (BLE_PROX_MONITOR)
        PRF_ITF(PROXM, proxm_prf_itf_get),
        #endif // (BLE_PROX_MONITOR)

        #if (BLE_PROX_REPORTER)
        PRF_ITF(PROXR, proxr_prf_itf_get),
        #endif // (BLE_PROX_REPORTER)

        #if (BLE_SP_SERVER)
        PRF_ITF(SCPPS, scpps_prf_itf_get),
        #endif // (BLE_SP_SERVER)

        #if (BLE_SP_CLIENT)
        PRF_ITF(SCPPC, scppc_prf_itf_get),
        #endif // (BLE_SP_CLIENT)

        #if (BLE_BATT_SERVER)
        PRF_ITF(BASS, bass_prf_itf_get),
        #endif // (BLE_BATT_SERVER)

        #if (BLE_BATT_CLIENT)
        PRF_ITF(BASC, basc_prf_itf_get),
        #endif // (BLE_BATT_CLIENT)

        #if (BLE_HID_DEVICE)
        PRF_ITF(HOGPD, hogpd_prf_itf_get),
        #endif // (BLE_HID_DEVICE)

        #if (BLE_HID_BOOT_HOST)
        PRF_ITF(HOGPBH, hogpbh_prf_itf_get),
        #endif // (BLE_HID_BOOT_HOST)

        #if (BLE_HID_REPORT_HOST)
        PRF_ITF(HOGPRH, hogprh_prf_itf_get),
        #endif // (BLE_HID_REPORT_HOST)

        #if (BLE_GL_COLLECTOR)
        PRF_ITF(GLPC, glpc_prf_itf_get),
        #endif // (BLE_GL_COLLECTOR)

        #if (BLE_GL_SENSOR)
        PRF_ITF(GLPS, glps_prf_itf_get),
        #endif // (BLE_GL_SENSOR)

        #if (BLE_RSC_COLLECTOR)
        PRF_ITF(RSCPC, rscpc_prf_itf_get),
        #endif // (BLE_RSC_COLLECTOR)

        #if (BLE_RSC_SENSOR)
        PRF_ITF(RSCPS, rscps_prf_itf_get),
        #endif // (BLE_RSC_SENSOR)

        #if (BLE_CSC_COLLECTOR)
        PRF_ITF(CSCPC, cscpc_prf_itf_get),
        #endif // (BLE_CSC_COLLECTOR)

        #if (BLE_CSC_SENSOR)
        PRF_ITF(CSCPS, cscps_prf_itf_get),
        #endif // (BLE_CSC_SENSOR)

        #if (BLE_CP_COLLECTOR)
        PRF_ITF(CPPC, cppc_prf_itf_get),
        #endif // (BLE_CP_COLLECTOR)

        #if (BLE_CP_SENSOR)
        PRF_ITF(CPPS, cpps_prf_itf_get),
        #endif // (BLE_CP_SENSOR)

        #if (BLE_LN_COLLECTOR)
        PRF_ITF(LANC, lanc_prf_itf_get),
        #endif // (BLE_LN_COLLECTOR)

        #if (BLE_LN_SENSOR)
        PRF_ITF(LANS, lans_prf_itf_get),
        #endif // (BLE_LN_SENSOR)

        #if (BLE_AN_CLIENT)
        PRF_ITF(ANPC, anpc_prf_itf_get),
        #endif // (BLE_AN_CLIENT)

        #if (BLE_AN_SERVER)
        PRF_ITF(ANPS, anps_prf_itf_get),
        #endif // (BLE_AN_SERVER)

        #if (BLE_PAS_CLIENT)
        PRF_ITF(PASPC, paspc_prf_itf_get),
        #endif // (BLE_PAS_CLIENT)

        #if (BLE_PAS_SERVER)
        PRF_ITF(PASPS, pasps_prf_itf_get),
        #endif // (BLE_PAS_SERVER)

        #if (BLE_ISO_MODE_0_PROFILE)
        PRF_ITF(AM0, am0_prf_itf_get),
        #endif // (BLE_ISO_MODE_0_PROFILE)

        #if (BLE_IPS_SERVER)
        PRF_ITF(IPSS, ipss_prf_itf_get),
        #endif //(BLE_IPS_SERVER)

        #if (BLE_IPS_CLIENT)
        PRF_ITF(IPSC, ipsc_prf_itf_get),
        #endif //(BLE_IPS_CLIENT)

        #if (BLE_ENV_SERVER)
        PRF_ITF(ENVS, envs_prf_itf_get),
        #endif //(BLE_ENV_SERVER)

        #if (BLE_ENV_CLIENT)
        PRF_ITF(ENVC, envc_prf_itf_get),
        #endif //(BLE_ENV_CLIENT)

        #if (BLE_WSC_SERVER)
        PRF_ITF(WSCS, wscs_prf_itf_get),
        #endif //(BLE_WSC_SERVER)

        #if (BLE_WSC_CLIENT)
        PRF_ITF(WSCC, wscc_prf_itf_get),
        #endif //(BLE_WSC_CLIENT)

        #if (BLE_BCS_SERVER)
        PRF_ITF(BCSS, bcss_prf_itf_get),
        #endif //(BLE_BCS_SERVER)

        #if (BLE_BCS_CLIENT)
        PRF_ITF(BCSC, bcsc_prf_itf_get),
        #endif //(BLE_BCS_CLIENT)

        #if (BLE_UDS_SERVER)
        PRF_ITF(UDSS, udss_prf_itf_get),
        #endif //(BLE_UDS_SERVER)

        #if (BLE_UDS_CLIENT)
        PRF_ITF(UDSC, udsc_prf_itf_get),
        #endif //(BLE_UDS_CLIENT)

        #if (BLE_WPT_SERVER)
        PRF_ITF(WPTS, wpts_prf_itf_get),
        #endif //(BLE_WPT_SERVER)

        #if (BLE_WPT_CLIENT)
        PRF_ITF(WPTC, wptc_prf_itf_get),
        #endif //(BLE_WPT_CLIENT)

        #if (BLE_PLX_SERVER)
        PRF_ITF(PLXS, plxs_prf_itf_get),
        #endif //(BLE_PLX_SERVER)

        #if (BLE_PLX_CLIENT)
        PRF_ITF(PLXC, plxc_prf_itf_get),
        #endif //(BLE_PLX_CLIENT)

        #if (BLE_CGM_SERVER)
        PRF_ITF(CGMS, cgms_prf_itf_get),
        #endif //(BLE_CGM_SERVER)

        #if (BLE_CGM_CLIENT)
        PRF_ITF(CGMC, cgmc_prf_itf_get),
        #endif //(BLE_CGM_CLIENT)

        #if (BLE_CSIS_MEMBER)
        PRF_ITF(CSISM, csism_prf_itf_get),
        #endif //(BLE_CSIS_MEMBER)

        #if (BLE_CSIS_COORD)
        PRF_ITF(CSISC, csisc_prf_itf_get),
        #endif //(BLE_CSIS_COORD)

        #if (BLE_OT_SERVER)
        PRF_ITF(OTS, ots_prf_itf_get),
        #endif //(BLE_OT_SERVER)

        #if (BLE_OT_CLIENT)
        PRF_ITF(OTC, otc_prf_itf_get),
        #endif //(BLE_OT_CLIENT)

        #if (BLE_DBG_THPP)
        PRF_ITF(THPP, thpp_prf_itf_get),
        #endif //(BLE_DBG_THPP)

        #if (BLE_MESH)
        PRF_ITF(MESH, mal_prf_itf_get),
        #endif //(BLE_MESH)

        // ----------------------- BT Classic Profiles -----------------------
        #if (BT_A2DP_SINK)
        PRF_ITF(A2DP_SINK, a2dp_sink_prf_itf_get),
        #endif //(BT_A2DP_SINK)

        #if (BT_A2DP_SOURCE)
        PRF_ITF(A2DP_SOURCE, a2dp_source_prf_itf_get),
        #endif //(BT_A2DP_SOURCE)

        #if (BT_AVRCP)
        PRF_ITF(AVRCP, avrcp_prf_itf_get),
        #endif //(BT_AVRCP)

        #if (BT_HFP_AG)
        PRF_ITF(HFP_AG, hfp_ag_prf_itf_get),
        #endif //(BT_HFP_AG)

        #if (BT_HFP_HF)
        PRF_ITF(HFP_HF, hfp_hf_prf_itf_get),
        #endif //(BT_HFP_HF)

        #if (BT_SPP)
        PRF_ITF(SPP, spp_prf_itf_get),
        #endif //(BT_SPP)

        #if (BT_PBAP_CLIENT)
        PRF_ITF(PBAP_CLIENT, pbap_client_prf_itf_get),
        #endif //(BT_PBAP_CLIENT)

        #if (APPLE_IAP)
        PRF_ITF(APPLE_IAP, apple_iap_prf_itf_get),
        #endif //(APPLE_IAP)

        // ----------------------- BT Dual Mode Profiles -----------------------
        #if (GFP_PROVIDER)
        PRF_ITF(GFP_PROVIDER, gfp_provider_prf_itf_get),
        #endif // (GFP_PROVIDER)
		#if (BLE_GATV_SERVER)
		PRF_ITF(GATVS, gatvs_prf_itf_get),
        #endif
};

/// Mapping between profile ID and profile index
__STATIC uint8_t prf_id_to_idx[PRF_ID_MAX];

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Convert API identifier to profile identifier
 ****************************************************************************************
 */
__STATIC uint8_t prf_id_get(uint8_t api_id)
{
    uint8_t prf_id = PRF_ID_INVALID;

    switch(TASK_TYPE_GET(api_id))
    {
        #if (BLE_HT_THERMOM)
        case TASK_ID_HTPT: { prf_id = PRF_ID_HTPT; } break;
        #endif // (BLE_HT_THERMOM)

        #if (BLE_HT_COLLECTOR)
        case TASK_ID_HTPC: { prf_id = PRF_ID_HTPC; } break;
        #endif // (BLE_HT_COLLECTOR)

        #if (BLE_DIS_SERVER)
        case TASK_ID_DISS: { prf_id = PRF_ID_DISS; } break;
        #endif // (BLE_DIS_SERVER)

        #if (BLE_DIS_CLIENT)
        case TASK_ID_DISC: { prf_id = PRF_ID_DISC; } break;
        #endif // (BLE_DIS_CLIENT)

        #if (BLE_BP_SENSOR)
        case TASK_ID_BLPS: { prf_id = PRF_ID_BLPS; } break;
        #endif // (BLE_BP_SENSOR)

        #if (BLE_BP_COLLECTOR)
        case TASK_ID_BLPC: { prf_id = PRF_ID_BLPC; } break;
        #endif // (BLE_BP_COLLECTOR)

        #if (BLE_TIP_SERVER)
        case TASK_ID_TIPS: { prf_id = PRF_ID_TIPS; } break;
        #endif // (BLE_TIP_SERVER)

        #if (BLE_TIP_CLIENT)
        case TASK_ID_TIPC: { prf_id = PRF_ID_TIPC; } break;
        #endif // (BLE_TIP_CLIENT)

        #if (BLE_HR_SENSOR)
        case TASK_ID_HRPS: { prf_id = PRF_ID_HRPS; } break;
        #endif // (BLE_HR_SENSOR)

        #if (BLE_HR_COLLECTOR)
        case TASK_ID_HRPC: { prf_id = PRF_ID_HRPC; } break;
        #endif // (BLE_HR_COLLECTOR)

        #if (BLE_FINDME_LOCATOR)
        case TASK_ID_FINDL: { prf_id = PRF_ID_FINDL; } break;
        #endif // (BLE_FINDME_LOCATOR)

        #if (BLE_FINDME_TARGET)
        case TASK_ID_FINDT: { prf_id = PRF_ID_FINDT; } break;
        #endif // (BLE_FINDME_TARGET)

        #if (BLE_PROX_MONITOR)
        case TASK_ID_PROXM: { prf_id = PRF_ID_PROXM; } break;
        #endif // (BLE_PROX_MONITOR)

        #if (BLE_PROX_REPORTER)
        case TASK_ID_PROXR: { prf_id = PRF_ID_PROXR; } break;
        #endif // (BLE_PROX_REPORTER)

        #if (BLE_SP_SERVER)
        case TASK_ID_SCPPS: { prf_id = PRF_ID_SCPPS; } break;
        #endif // (BLE_SP_SERVER)

        #if (BLE_SP_CLIENT)
        case TASK_ID_SCPPC: { prf_id = PRF_ID_SCPPC; } break;
        #endif // (BLE_SP_CLIENT)

        #if (BLE_BATT_SERVER)
        case TASK_ID_BASS: { prf_id = PRF_ID_BASS; } break;
        #endif // (BLE_BATT_SERVER)

        #if (BLE_BATT_CLIENT)
        case TASK_ID_BASC: { prf_id = PRF_ID_BASC; } break;
        #endif // (BLE_BATT_CLIENT)

        #if (BLE_HID_DEVICE)
        case TASK_ID_HOGPD: { prf_id = PRF_ID_HOGPD; } break;
        #endif // (BLE_HID_DEVICE)
		  
		#if (BLE_GATV_SERVER)
		  case TASK_ID_GATVS: { prf_id = PRF_ID_GATVS; } break;
        #endif

        #if (BLE_HID_BOOT_HOST)
        case TASK_ID_HOGPBH: { prf_id = PRF_ID_HOGPBH; } break;
        #endif // (BLE_HID_BOOT_HOST)

        #if (BLE_HID_REPORT_HOST)
        case TASK_ID_HOGPRH: { prf_id = PRF_ID_HOGPRH; } break;
        #endif // (BLE_HID_REPORT_HOST)

        #if (BLE_GL_COLLECTOR)
        case TASK_ID_GLPC: { prf_id = PRF_ID_GLPC; } break;
        #endif // (BLE_GL_COLLECTOR)

        #if (BLE_GL_SENSOR)
        case TASK_ID_GLPS: { prf_id = PRF_ID_GLPS; } break;
        #endif // (BLE_GL_SENSOR)

        #if (BLE_RSC_COLLECTOR)
        case TASK_ID_RSCPC: { prf_id = PRF_ID_RSCPC; } break;
        #endif // (BLE_RSC_COLLECTOR)

        #if (BLE_RSC_SENSOR)
        case TASK_ID_RSCPS: { prf_id = PRF_ID_RSCPS; } break;
        #endif // (BLE_RSC_SENSOR)

        #if (BLE_CSC_COLLECTOR)
        case TASK_ID_CSCPC: { prf_id = PRF_ID_CSCPC; } break;
        #endif // (BLE_CSC_COLLECTOR)

        #if (BLE_CSC_SENSOR)
        case TASK_ID_CSCPS: { prf_id = PRF_ID_CSCPS; } break;
        #endif // (BLE_CSC_SENSOR)

        #if (BLE_CP_COLLECTOR)
        case TASK_ID_CPPC: { prf_id = PRF_ID_CPPC; } break;
        #endif // (BLE_CP_COLLECTOR)

        #if (BLE_CP_SENSOR)
        case TASK_ID_CPPS: { prf_id = PRF_ID_CPPS; } break;
        #endif // (BLE_CP_SENSOR)

        #if (BLE_LN_COLLECTOR)
        case TASK_ID_LANC: { prf_id = PRF_ID_LANC; } break;
        #endif // (BLE_LN_COLLECTOR)

        #if (BLE_LN_SENSOR)
        case TASK_ID_LANS: { prf_id = PRF_ID_LANS; } break;
        #endif // (BLE_LN_SENSOR)

        #if (BLE_AN_CLIENT)
        case TASK_ID_ANPC: { prf_id = PRF_ID_ANPC; } break;
        #endif // (BLE_AN_CLIENT)

        #if (BLE_AN_SERVER)
        case TASK_ID_ANPS: { prf_id = PRF_ID_ANPS; } break;
        #endif // (BLE_AN_SERVER)

        #if (BLE_PAS_CLIENT)
        case TASK_ID_PASPC: { prf_id = PRF_ID_PASPC; } break;
        #endif // (BLE_PAS_CLIENT)

        #if (BLE_PAS_SERVER)
        case TASK_ID_PASPS: { prf_id = PRF_ID_PASPS; } break;
        #endif // (BLE_PAS_SERVER)

        #if (BLE_ISO_MODE_0_PROFILE)
        case TASK_ID_AM0:  { prf_id = PRF_ID_AM0;  } break;
        #endif // (BLE_ISO_MODE_0_PROFILE)

        #if (BLE_IPS_SERVER)
        case TASK_ID_IPSS: { prf_id = PRF_ID_IPSS; } break;
        #endif //(BLE_IPS_SERVER)

        #if (BLE_IPS_CLIENT)
        case TASK_ID_IPSC: { prf_id = PRF_ID_IPSC; } break;
        #endif //(BLE_IPS_CLIENT)

        #if (BLE_ENV_SERVER)
        case TASK_ID_ENVS: { prf_id = PRF_ID_ENVS; } break;
        #endif //(BLE_ENV_SERVER)

        #if (BLE_ENV_CLIENT)
        case TASK_ID_ENVC: { prf_id = PRF_ID_ENVC; } break;
        #endif //(BLE_ENV_CLIENT)

        #if (BLE_WSC_SERVER)
        case TASK_ID_WSCS: { prf_id = PRF_ID_WSCS; } break;
        #endif //(BLE_WSC_SERVER)

        #if (BLE_WSC_CLIENT)
        case TASK_ID_WSCC: { prf_id = PRF_ID_WSCC; } break;
        #endif //(BLE_WSC_CLIENT)

        #if (BLE_BCS_SERVER)
        case TASK_ID_BCSS: { prf_id = PRF_ID_BCSS; } break;
        #endif //(BLE_BCS_SERVER)

        #if (BLE_BCS_CLIENT)
        case TASK_ID_BCSC: { prf_id = PRF_ID_BCSC; } break;
        #endif //(BLE_BCS_CLIENT)

        #if (BLE_UDS_SERVER)
        case TASK_ID_UDSS: { prf_id = PRF_ID_UDSS; } break;
        #endif //(BLE_UDS_SERVER)

        #if (BLE_UDS_CLIENT)
        case TASK_ID_UDSC: { prf_id = PRF_ID_UDSC; } break;
        #endif //(BLE_UDS_CLIENT)

        #if (BLE_WPT_SERVER)
        case TASK_ID_WPTS: { prf_id = PRF_ID_WPTS; } break;
        #endif //(BLE_WPT_SERVER)

        #if (BLE_WPT_CLIENT)
        case TASK_ID_WPTC: { prf_id = PRF_ID_WPTC; } break;
        #endif //(BLE_WPT_CLIENT)

        #if (BLE_PLX_SERVER)
        case TASK_ID_PLXS: { prf_id = PRF_ID_PLXS; } break;
        #endif //(BLE_PLX_SERVER)

        #if (BLE_PLX_CLIENT)
        case TASK_ID_PLXC: { prf_id = PRF_ID_PLXC; } break;
        #endif //(BLE_PLX_CLIENT)

        #if (BLE_CGM_SERVER)
        case TASK_ID_CGMS: { prf_id = PRF_ID_CGMS; } break;
        #endif //(BLE_CGM_SERVER)

        #if (BLE_CGM_CLIENT)
        case TASK_ID_CGMC: { prf_id = PRF_ID_CGMC; } break;
        #endif //(BLE_CGM_CLIENT)

        #if (BLE_CSIS_MEMBER)
        case TASK_ID_CSISM: { prf_id = PRF_ID_CSISM; } break;
        #endif //(BLE_CSIS_MEMBER)

        #if (BLE_CSIS_COORD)
        case TASK_ID_CSISC: { prf_id = PRF_ID_CSISC; } break;
        #endif //(BLE_CSIS_COORD)

        #if (BLE_OT_SERVER)
        case TASK_ID_OTS: { prf_id = PRF_ID_OTS; } break;
        #endif //(BLE_OT_SERVER)

        #if (BLE_OT_CLIENT)
        case TASK_ID_OTC: { prf_id = PRF_ID_OTC; } break;
        #endif //(BLE_OT_CLIENT)

        #if (BLE_DBG_THPP)
        case TASK_ID_THPP: { prf_id = PRF_ID_THPP; } break;
        #endif //(BLE_DBG_THPP)

        #if (BLE_MESH)
        case TASK_ID_MESH: { prf_id = PRF_ID_MESH; } break;
        #endif //(BLE_MESH)

        #if (BT_A2DP_SINK)
        case TASK_ID_A2DP_SINK:   { prf_id = PRF_ID_A2DP_SINK; } break;
        #endif //(BT_A2DP_SINK)

        #if (BT_A2DP_SOURCE)
        case TASK_ID_A2DP_SOURCE: { prf_id = PRF_ID_A2DP_SOURCE; } break;
        #endif //(BT_A2DP_SOURCE)

        #if (BT_AVRCP)
        case TASK_ID_AVRCP: { prf_id = PRF_ID_AVRCP; } break;
        #endif //(BT_AVRCP)

        #if (BT_HFP_AG)
        case TASK_ID_HFP_AG: { prf_id = PRF_ID_HFP_AG; } break;
        #endif //(BT_HFP_AG)

        #if (BT_HFP_HF)
        case TASK_ID_HFP_HF: { prf_id = PRF_ID_HFP_HF; } break;
        #endif //(BT_HFP_HF)

        #if (BT_SPP)
        case TASK_ID_SPP: { prf_id = PRF_ID_SPP; } break;
        #endif //(BT_SPP)

        #if (BT_PBAP_CLIENT)
        case TASK_ID_PBAP_CLIENT: { prf_id = PRF_ID_PBAP_CLIENT; } break;
        #endif //(BT_PBAP_CLIENT)

        #if (APPLE_IAP)
        case TASK_ID_APPLE_IAP: { prf_id = PRF_ID_APPLE_IAP; } break;
        #endif //(APPLE_IAP)

        #if (GFP_PROVIDER)
        case TASK_ID_GFP_PROVIDER: { prf_id = PRF_ID_GFP_PROVIDER; } break;
        #endif // (GFP_PROVIDER)
        default: /* Nothing to do */ break;
    }

    return prf_id;
}


/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */
void prf_initialize(uint8_t init_type)
{
    uint8_t prf_idx;

    if(init_type == RWIP_INIT)
    {

        #if (HOST_MSG_API)
        for(prf_idx = 0; prf_idx < HOST_NB_PROFILES ; prf_idx++)
        {
            prf_data_t* p_prf = &(prf_env.prf[prf_idx]);
            // FW boot profile initialization
            p_prf->prf_task = TASK_PRF_MIN + prf_idx;
            ke_task_create(p_prf->prf_task, &(p_prf->desc));
        }
        #endif // (HOST_MSG_API)
    }
    else
    {
        for(prf_idx = 0; prf_idx < HOST_NB_PROFILES ; prf_idx++)
        {
            prf_data_t* p_prf = &(prf_env.prf[prf_idx]);

            switch (init_type)
            {
                case RWIP_RST:
                {
                    // FW reset profile destruction
                    if(p_prf->prf_id != PRF_ID_INVALID)
                    {
                        // Get Profile API
                        const prf_task_cbs_t* p_cbs = prf_itf[p_prf->prf_id]();
                        // request to destroy profile
                        p_cbs->cb_destroy(p_prf, PRF_DESTROY_RESET);
                    }
                }
                // No break
                case RWIP_1ST_RST:
                {
                    // FW boot profile destruction
                    p_prf->p_env                = NULL;
                    // unregister profile
                    p_prf->api_id               = TASK_ID_INVALID;
                    p_prf->prf_id               = PRF_ID_INVALID;
                    #if (HOST_MSG_API)
                    p_prf->desc.msg_handler_tab = NULL;
                    p_prf->desc.state           = NULL;
                    p_prf->desc.idx_max         = 0;
                    p_prf->desc.msg_cnt         = 0;
                    #endif // (HOST_MSG_API)
                } break;

                default: { /* Do nothing */  } break;
            }
        }

        memset(prf_id_to_idx, PRF_IDX_INVALID, PRF_ID_MAX);
    }
}

uint16_t prf_add_profile(uint8_t api_id, uint8_t sec_lvl, uint8_t user_prio, const void* p_params, const void* p_cb,
                         uint16_t* p_start_hdl)
{
    uint8_t prf_idx;
    uint16_t status = GAP_ERR_NO_ERROR;

    // retrieve profile callback
    const prf_task_cbs_t* p_cbs = NULL;
    uint8_t prf_id = prf_id_get(api_id);
    prf_data_t* p_prf = NULL;

    if(prf_id != PRF_ID_INVALID)
    {
        p_cbs = prf_itf[prf_id]();
    }

    if(p_cbs == NULL)
    {
        // profile API not available
        status = GAP_ERR_INVALID_PARAM;
    }

    // check if profile not already present in task list
    if(status == GAP_ERR_NO_ERROR)
    {

        for(prf_idx = 0; prf_idx < HOST_NB_PROFILES ; prf_idx++)
        {
            p_prf = &(prf_env.prf[prf_idx]);
            if(p_prf->prf_id == prf_id)
            {
                status = GAP_ERR_COMMAND_DISALLOWED;
                break;
            }
        }
    }

    if(status == GAP_ERR_NO_ERROR)
    {
        // find first available task
        for(prf_idx = 0; prf_idx < HOST_NB_PROFILES ; prf_idx++)
        {
            p_prf = &(prf_env.prf[prf_idx]);

            // available task found
            if(p_prf->prf_id == PRF_ID_INVALID)
            {
                // initialize profile
                status = p_cbs->cb_init(p_prf, p_start_hdl, sec_lvl, user_prio, p_params, p_cb);

                // initialization succeed
                if(status == GAP_ERR_NO_ERROR)
                {
                    prf_id_to_idx[prf_id] = prf_idx;
                    // register profile
                    p_prf->prf_id = prf_id;
                    p_prf->api_id = api_id;

                    #if (HOST_MSG_API)
                    p_prf->desc.state      = NULL;  // Shall not be used
                    p_prf->desc.idx_max    = PRF_TASK_IDX_MAX;
                    #endif // (HOST_MSG_API)

                    #if ((RW_DEBUG) && (HOST_MSG_API))
                    {
                        // Sanity check of ID ascending order in the message table
                        int16_t i;
                        // Pointer to previous handler in the table
                        const struct ke_msg_handler* p_prev_handler = &((p_prf->desc).msg_handler_tab[0]);
                        // Pointer to current handler in the table
                        const struct ke_msg_handler* p_cur_handler = &((p_prf->desc).msg_handler_tab[1]);

                        for (i = (p_prf->desc).msg_cnt - 2; i >= 0; i--)
                        {
                            // ID of previous handler should small than ID of current handler
                            // as the table is sorted in ascending order
                            ASSERT_ERR(p_prev_handler->id < p_cur_handler->id);

                            p_prev_handler++;
                            p_cur_handler++;
                        }
                    }
                    #endif // ((RW_DEBUG) && (HOST_MSG_API))

                }
                break;
            }
        }

        if(prf_idx == HOST_NB_PROFILES)
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    return (status);
}



void prf_con_create(uint8_t conidx, bool is_le_con)
{
    uint8_t prf_idx;
    /* simple connection creation handler, nothing to do. */

    // execute create function of each profiles
    for(prf_idx = 0; prf_idx < HOST_NB_PROFILES ; prf_idx++)
    {
        prf_data_t* p_prf = &(prf_env.prf[prf_idx]);

        // FW reset profile destruction
        if(p_prf->prf_id != PRF_ID_INVALID)
        {
            // Get Profile API
            const prf_task_cbs_t* p_cbs = prf_itf[p_prf->prf_id]();
            // call create callback
            p_cbs->cb_con_create(p_prf, conidx, is_le_con);
        }
    }
}

void prf_con_cleanup(uint8_t conidx, uint8_t reason)
{
    uint8_t prf_idx;
    /* simple connection creation handler, nothing to do. */

    // execute create function of each profiles
    for(prf_idx = 0; prf_idx < HOST_NB_PROFILES ; prf_idx++)
    {
        prf_data_t* p_prf = &(prf_env.prf[prf_idx]);

        // FW reset profile destruction
        if(p_prf->prf_id != PRF_ID_INVALID)
        {
            // Get Profile API
            const prf_task_cbs_t* p_cbs = prf_itf[p_prf->prf_id]();
            // call cleanup callback
            p_cbs->cb_con_cleanup(p_prf, conidx, reason);
        }
    }
}


prf_hdr_t* prf_env_get(uint8_t prf_id)
{
    prf_hdr_t* p_env = NULL;

    if(prf_id < PRF_ID_MAX)
    {
        uint8_t prf_idx = prf_id_to_idx[prf_id];
        if(prf_idx != PRF_IDX_INVALID)
        {
            p_env = prf_env.prf[prf_idx].p_env;
        }
    }

    return p_env;
}

#if (HOST_MSG_API)

ke_msg_id_t prf_msg_api_init(uint8_t api_id, ke_msg_id_t app_task)
{
    uint8_t prf_idx;
    ke_msg_id_t prf_task = TASK_NONE;

    // find if profile present in profile tasks
    for(prf_idx = 0; prf_idx < HOST_NB_PROFILES ; prf_idx++)
    {
        prf_data_t* p_prf = &(prf_env.prf[prf_idx]);
        // check if application identifier is present
        if(p_prf->api_id == api_id)
        {
            prf_task = p_prf->prf_task;
            p_prf->app_task = KE_TYPE_GET(app_task);
            break;
        }
    }

    return prf_task;
}

ke_task_id_t prf_dst_task_get(uint8_t prf_id)
{
    ke_task_id_t app_task = TASK_NONE;
    uint8_t prf_idx = prf_id_to_idx[prf_id];
    if(prf_idx != PRF_IDX_INVALID)
    {
        app_task = prf_env.prf[prf_idx].app_task;
    }

    return app_task;
}

ke_task_id_t prf_src_task_get(uint8_t prf_id)
{
    ke_task_id_t prf_task = TASK_NONE;
    uint8_t prf_idx = prf_id_to_idx[prf_id];
    if(prf_idx != PRF_IDX_INVALID)
    {
        prf_task = prf_env.prf[prf_idx].prf_task;
    }

    return prf_task;
}

uint8_t prf_get_api_id_from_task_num(uint8_t task)
{
    uint8_t api_id = TASK_ID_INVALID;
    uint8_t prf_idx;

    // find if profile present in profile tasks
    for(prf_idx = 0; prf_idx < HOST_NB_PROFILES ; prf_idx++)
    {
        prf_data_t* p_prf = &(prf_env.prf[prf_idx]);
        // check if profile identifier is known
        if(p_prf->prf_task == task)
        {
            api_id = p_prf->api_id;
            break;
        }
    }

    return api_id;
}

uint8_t prf_get_task_num_from_api_id(uint8_t api_id)
{
    uint8_t task_num = TASK_NONE;
    uint8_t prf_idx;

    // find if profile present in profile tasks
    for(prf_idx = 0; prf_idx < HOST_NB_PROFILES ; prf_idx++)
    {
        prf_data_t* p_prf = &(prf_env.prf[prf_idx]);

        // check if profile identifier is known
        if(p_prf->api_id == api_id)
        {
            task_num = p_prf->prf_task;
            break;
        }
    }

    return task_num;
}

#endif //(HOST_MSG_API)

#endif // (HOST_PROFILES)

/// @} PRF
