/*
 * wahchdog.h
 *
 *  Created on: 2021年12月24日
 *      Author: haijun.mai
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include "ms1008.h"

extern int32_t watchdog_init(void);
extern int32_t watchdog_feed(void);
extern int32_t watchdog_deinit(void);
extern int32_t watchdog_close(void);


#endif /* WATCHDOG_H_ */

