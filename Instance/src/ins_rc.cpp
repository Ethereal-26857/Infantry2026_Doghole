#include "ins_rc.hpp"
#include "iwdg.h"

static void RefreshIwdg(void)
{
    HAL_IWDG_Refresh(&hiwdg1);
}

bool is_rc_inited = false;

hw_rc::DT7 unique_rc = hw_rc::DT7();

hw_rc::DT7* GetRcIns()
{
    if (!is_rc_inited)
    {
        unique_rc.registerUpdateCallback(RefreshIwdg);
        is_rc_inited = true;
    }
    return &unique_rc;
};


