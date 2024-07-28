#include "rwip_config.h"    // stack configuration

#include <string.h>

#include "arch.h"
#include "co_list.h"
#include "co_math.h"

#include "rwip.h"
#include "dbg.h"

#include "reg_em_et.h"       // EM Exchange Table

#include "reg_ipcore.h"      // DM core registers
#if BLE_EMB_PRESENT
#include "reg_em_ble_cs.h"   // EM Control Structure
#include "reg_blecore.h"     // for ble_leschcntl_pack
#endif // BLE_EMB_PRESENT

#if BT_EMB_PRESENT
#include "reg_em_bt_cs.h"   // EM Control Structure
#include "reg_btcore.h"     // for bt_leschcntl_pack
#endif // BT_EMB_PRESENT

#include "sch_prog.h"        // link layer driver Scheduling Programmer
#include "ms_section.h"


BOOTROM_FUNCTION int _rand (void)
{
    uint32_t clock = rwip_time_get().hs;
	int rand_value = 0x5A5A1234;
    
	if((clock != 0xFFFFFFFF) && (clock != 0x00))
	{
		rand_value = (int)clock * clock + 1;
	}
	return rand_value;
}