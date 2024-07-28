/**
 ****************************************************************************************
 *
 * @file security_dbg_test.c
 *
 * @brief Module use to test security
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#if (RW_DEBUG)
#include "co_bt.h"
#include "aes.h"
#if (ECC_P256_SUPPORT)
#include "ecc_p256.h"
#endif // (ECC_P256_SUPPORT)

#if defined(CFG_SHA_256_SUPPORT)
#include "sha.h"
#endif // defined(CFG_SHA_256_SUPPORT)

#if defined(CFG_AES_CTR_SUPPORT)
#include "co_buf.h"
#endif // defined(CFG_AES_CTR_SUPPORT)

#include  "co_utils.h"
#include <string.h>


/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/// List of supported tests
enum security_test_type
{
    /// ECC P256 Private Key Generation
    SECURITY_TEST_ECC_PRIVATE_KEY_GEN,
    /// ECC P256 Public Key Generation
    SECURITY_TEST_ECC_PUBLIC_KEY_GEN,
    /// ECC P256 DH Key Generation
    SECURITY_TEST_ECC_DH_KEY_GEN,
    /// SHA-256 Hash Compute
    SECURITY_TEST_SHA_256_HASH,
    /// HMAC-SHA-256 Compute
    SECURITY_TEST_SHA_256_HMAC,
    /// AES-CTR
    SECURITY_TEST_AES_CTR,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
/// Callback executed to return result
typedef void (*security_test_result_cb)(uint16_t result_size, const void* p_result);


/// Test type
typedef struct security_test_info
{
    /// see enum #security_test_type
    uint8_t test_type;
} security_test_info_t;

#if (ECC_P256_SUPPORT)

/// ECC P256 Private Key Generate Result
typedef struct security_test_ecc_private_key_gen_result
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Private Key
    uint8_t private_key[ECC_256_KEY_SIZE];
} security_test_ecc_private_key_gen_result_t;

/// ECC P256 Public Key Generate Parameters
typedef struct security_test_ecc_public_key_gen_param
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Private Key
    uint8_t private_key[ECC_256_KEY_SIZE];
} security_test_ecc_public_key_gen_param_t;

/// ECC P256 Public Key Generate Result
typedef struct security_test_ecc_public_key_gen_result
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Public Key X
    uint8_t public_key_x[ECC_256_KEY_SIZE];
    /// Public Key Y
    uint8_t public_key_y[ECC_256_KEY_SIZE];
} security_test_ecc_public_key_gen_result_t;

/// ECC P256 DH Key Generate Parameters
typedef struct security_test_ecc_dh_key_gen_param
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Private Key
    uint8_t private_key[ECC_256_KEY_SIZE];
    /// Public Key X
    uint8_t public_key_x[ECC_256_KEY_SIZE];
    /// Public Key Y
    uint8_t public_key_y[ECC_256_KEY_SIZE];
} security_test_ecc_dh_key_gen_param_t;

/// ECC P256 Public Key Generate Result
typedef struct security_test_ecc_dh_key_gen_result
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// DH-Key
    uint8_t dh_key[ECC_256_KEY_SIZE];
} security_test_ecc_dh_key_gen_result_t;

#endif // (ECC_P256_SUPPORT)


#if defined(CFG_SHA_256_SUPPORT)
/// SHA 256 hash Parameters
typedef struct security_test_sha_256_hash_param
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Message Length
    uint16_t message_len;
    /// Message
    uint8_t message[__ARRAY_EMPTY];
} security_test_sha_256_hash_param_t;

/// HMAC-SHA 256 Parameters
typedef struct security_test_sha_256_hmac_param
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Key Length
    uint8_t key_len;
    /// Message Length
    uint16_t message_len;
    /// key + Message
    uint8_t key_message[__ARRAY_EMPTY];
} security_test_sha_256_hmac_param_t;

/// SHA 256 hash / HMAC result
typedef struct security_test_sha_256_hash_result
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Sha 256 hash
    uint8_t hash[SHA_256_HASH_BYTE_SIZE];
} security_test_sha_256_hash_result_t;
#endif // defined(CFG_SHA_256_SUPPORT)



#if defined(CFG_AES_CTR_SUPPORT)
/// AES CTR Parameters
typedef struct security_test_aes_ctr_param
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Secret key
    uint8_t  secret_key[AES_BLOCK_SIZE];
    /// Nonce value
    uint8_t  nonce[AES_BLOCK_SIZE];
    /// Message Length
    uint16_t message_len;
    /// Message (raw or encrypted data)
    uint8_t  message[__ARRAY_EMPTY];
} security_test_aes_ctr_param_t;

/// AES CTR result
typedef struct security_test_aes_ctr_result
{
    /// see enum #security_test_type
    uint8_t test_type;
    /// Message Length
    uint16_t message_len;
    /// Message (encrypted or raw data)
    uint8_t  message[__ARRAY_EMPTY];
} security_test_aes_ctr_result_t;

/// AES CTR buffer metadata
typedef struct security_test_aes_ctr_buf_metadata
{
    co_buf_t* p_buf_in;
    security_test_result_cb cb_result;
} security_test_aes_ctr_buf_metadata_t;

#endif // defined(CFG_AES_CTR_SUPPORT)

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

#if (ECC_P256_SUPPORT)
__STATIC void security_test_ecc_public_key_gen_result_cb(security_test_result_cb cb_result, const ecc_result_t* p_res)
{
    security_test_ecc_public_key_gen_result_t result;
    result.test_type = SECURITY_TEST_ECC_PUBLIC_KEY_GEN;
    memcpy(result.public_key_x, p_res->key_res_x, ECC_256_KEY_SIZE);
    memcpy(result.public_key_y, p_res->key_res_y, ECC_256_KEY_SIZE);
    cb_result(sizeof(result), &result);
}


__STATIC void security_test_ecc_dh_key_gen_result_cb(security_test_result_cb cb_result, const ecc_result_t* p_res)
{
    security_test_ecc_dh_key_gen_result_t result;
    result.test_type = SECURITY_TEST_ECC_DH_KEY_GEN;
    memcpy(result.dh_key, p_res->key_res_x, ECC_256_KEY_SIZE);
    cb_result(sizeof(result), &result);
}
#endif // (ECC_P256_SUPPORT)


#if defined(CFG_AES_CTR_SUPPORT)
__STATIC void security_test_aes_ctr_result_cb(co_buf_t* p_buf_out)
{
    security_test_aes_ctr_buf_metadata_t* p_meta  = (security_test_aes_ctr_buf_metadata_t*) co_buf_metadata(p_buf_out);
    uint16_t data_len = co_buf_data_len(p_buf_out);
    uint8_t* p_data;
    co_buf_head_reserve(p_buf_out, 4); // Put header data
    p_data = co_buf_data(p_buf_out);
    *(p_data+1) = 0; // padding byte
    *p_data = SECURITY_TEST_AES_CTR;
    co_write16p(p_data+2, data_len);

    p_meta->cb_result(co_buf_data_len(p_buf_out), p_data);
    co_buf_release(p_meta->p_buf_in);
    co_buf_release(p_buf_out);
}
#endif // defined(CFG_AES_CTR_SUPPORT)
/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

bool security_dbg_test_execute(const security_test_info_t* p_info, security_test_result_cb cb_result)
{
    bool is_test_executed = true;

    switch(p_info->test_type)
    {
        #if (ECC_P256_SUPPORT)
        case SECURITY_TEST_ECC_PRIVATE_KEY_GEN:
        {
            security_test_ecc_private_key_gen_result_t result;
            result.test_type = SECURITY_TEST_ECC_PRIVATE_KEY_GEN;
            ecc_gen_new_secret_key(result.private_key);
            cb_result(sizeof(result), &result);
        } break;
        case SECURITY_TEST_ECC_PUBLIC_KEY_GEN:
        {
            security_test_ecc_public_key_gen_param_t* p_param = (security_test_ecc_public_key_gen_param_t*) p_info;
            ecc_gen_new_public_key(p_param->private_key, (uint32_t)cb_result, (ecc_result_cb)security_test_ecc_public_key_gen_result_cb);

        }break;
        case SECURITY_TEST_ECC_DH_KEY_GEN:
        {
            security_test_ecc_dh_key_gen_param_t* p_param = (security_test_ecc_dh_key_gen_param_t*) p_info;
            is_test_executed = (ecc_gen_dh_key(p_param->private_key, p_param->public_key_x, p_param->public_key_y,
                                               (uint32_t)cb_result, (ecc_result_cb) security_test_ecc_dh_key_gen_result_cb) == CO_ERROR_NO_ERROR);
        }break;
        #endif // (ECC_P256_SUPPORT)
        #if defined(CFG_SHA_256_SUPPORT)
        case SECURITY_TEST_SHA_256_HASH:
        {
            security_test_sha_256_hash_param_t* p_param = (security_test_sha_256_hash_param_t*) p_info;
            security_test_sha_256_hash_result_t result;
            sha_256_hash_t hash;
            result.test_type = SECURITY_TEST_SHA_256_HASH;
            sha_256(p_param->message, p_param->message_len, &hash);
            memcpy(result.hash, &hash, SHA_256_HASH_BYTE_SIZE);
            cb_result(sizeof(result), &result);
        } break;

        case SECURITY_TEST_SHA_256_HMAC:
        {
            security_test_sha_256_hmac_param_t* p_param = (security_test_sha_256_hmac_param_t*) p_info;
            security_test_sha_256_hash_result_t result;
            sha_256_hash_t hash;
            result.test_type = SECURITY_TEST_SHA_256_HMAC;
            sha_256_hmac(&(p_param->key_message[0]), p_param->key_len,
                         &(p_param->key_message[p_param->key_len]), p_param->message_len, &hash);
            memcpy(result.hash, &hash, SHA_256_HASH_BYTE_SIZE);
            cb_result(sizeof(result), &result);
        } break;
        #endif // #if defined(CFG_SHA_256_SUPPORT)

        #if defined(CFG_AES_CTR_SUPPORT)
        case SECURITY_TEST_AES_CTR:
        {
            co_buf_t* p_buf_in;
            co_buf_t* p_buf_out;

            security_test_aes_ctr_param_t* p_param = (security_test_aes_ctr_param_t*) p_info;
            security_test_aes_ctr_buf_metadata_t* p_meta;

            if(co_buf_alloc(&p_buf_in, 0, p_param->message_len, 0) != CO_BUF_ERR_NO_ERROR)
            {
                is_test_executed = false;
                break;
            }

            memcpy(co_buf_data(p_buf_in), p_param->message, p_param->message_len);

            if(co_buf_alloc(&p_buf_out, 4, p_param->message_len, 0) != CO_BUF_ERR_NO_ERROR)
            {
                co_buf_release(p_buf_in);
                is_test_executed = false;
                break;
            }
            p_meta  = (security_test_aes_ctr_buf_metadata_t*) co_buf_metadata(p_buf_out);
            p_meta->p_buf_in  = p_buf_in;
            p_meta->cb_result = cb_result;
            aes_ctr(p_param->secret_key, p_param->nonce, p_param->message_len,
                    co_buf_data(p_buf_in), co_buf_data(p_buf_out),
                    (aes_ctr_func_result_cb)security_test_aes_ctr_result_cb, (uint32_t)p_buf_out);
        } break;
        #endif // defined(CFG_AES_CTR_SUPPORT)

        default: { is_test_executed = false; } break;
    }

    return (is_test_executed);
}

#endif // (RW_DEBUG)
