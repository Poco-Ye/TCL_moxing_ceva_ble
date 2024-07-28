/*
 * trng.h
 *
 *  Created on: 2021年12月24日
 *      Author: haijun.mai
 */

#ifndef TRNG_H_
#define TRNG_H_

#include "ms1008.h"



extern uint32_t trng_init(void);
extern uint32_t trng_get_data(void);
extern uint32_t trng_deinit(void);



#endif /* TRNG_H_ */
