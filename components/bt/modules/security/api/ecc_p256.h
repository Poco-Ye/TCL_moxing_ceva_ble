/**
 ****************************************************************************************
 *
 * @file ecc_p256.h
 *
 * @brief  ECC functions for P256
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

#ifndef ECC_P256_H_
#define ECC_P256_H_


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#include <stdint.h>
#include <stdbool.h>
#include "ke_task.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#define ECC_256_KEY_SIZE         (32)


/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

/// Elliptic Curve computation result structure
typedef struct ecc_result
{
    uint8_t key_res_x[ECC_256_KEY_SIZE];
    uint8_t key_res_y[ECC_256_KEY_SIZE];
} ecc_result_t;

/**
 * Callback executed when Elliptic Curve algorithm completes
 *
 * @param[in] metadata      Metadata information provided by requester returned with result
 * @param[in] p_res         Pointer to Computed result
 */
typedef void (*ecc_result_cb)(uint32_t metadata, const ecc_result_t* p_res);


/*
 * VARIABLE DECLARATION
 ****************************************************************************************
 */

/// Debug Private Key
extern const uint8_t DebugE256SecretKey[32];

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize Elliptic Curve algorithm
 *
 * @param[in] init_type  Type of initialization (@see enum rwip_init_type)
 ****************************************************************************************
 */
void ecc_init(uint8_t init_type);

/**
 ****************************************************************************************
 * @brief Generate a Secret Key compliant with ECC P256 algorithm
 *
 * If key is forced, just check its validity
 *
 * @param[out] secret_key Private key - MSB First
 ****************************************************************************************
 */
void ecc_gen_new_secret_key(uint8_t* secret_key);

/**
 ****************************************************************************************
 * @brief Generate a new Public key pair using ECC P256 algorithm
 *
 * @param[in] secret_key Private key - MSB First
 * @param[in] metadata   Metadata information to return with result in order to retrieve execution context
 * @param[in] cb_result  Callback function to execute once algorithm completes
 *
 * @return status   0 if key generation is started, > 0 otherwise
 ****************************************************************************************
 */
uint8_t ecc_gen_new_public_key(uint8_t* secret_key256, uint32_t metadata, ecc_result_cb cb_result);

/**
 ****************************************************************************************
 * @brief Generate a new DHKey using ECC P256 algorithm
 *
 * @param[in] secret_key        Private key                  - MSB First
 * @param[in] public_key_x      Peer public key x coordinate - LSB First
 * @param[in] public_key_y      Peer public key y coordinate - LSB First
 * @param[in] metadata          Metadata information to return with result in order to retrieve execution context
 * @param[in] cb_result         Callback function to execute once algorithm completes
 *
 * @return status   0 if key generation is started, > 0 otherwise
 ****************************************************************************************
 */
uint8_t ecc_gen_dh_key(const uint8_t* secret_key, const uint8_t* public_key_x, const uint8_t* public_key_y,
                       uint32_t metadata, ecc_result_cb cb_result);

/**
 ****************************************************************************************
 * @brief Abort a current DHKey generation procedure
 *
 * @param[in] metadata  Metadata information used when starting algorithm
 ****************************************************************************************
 */
void ecc_abort_key256_generation(uint16_t metadata);


#endif /* ECC_P256_H_ */
