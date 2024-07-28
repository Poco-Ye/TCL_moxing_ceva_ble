#ifndef _MS_H_
#define _MS_H_

#ifdef __cplusplus
    extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
typedef enum{ RESET, SET = !RESET} FlagStatus, ITstatus;
typedef enum{ DISABLE = 0, ENABLE = !DISABLE } State;



#ifdef __cplusplus
}
#endif

#endif //_MS_H_
