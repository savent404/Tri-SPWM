#include "littleGUI.h"
#include "cpu_utils.h"

GUI_Status_t ParaShow_Stage(uint8_t init_flag)
{

    
    if (init_flag)
    {
        GUI_CLEAR();
    }
    //HAL_ResumeTick();
    SETFONT(12);
    SSHOWSTRING(0, 0, "%d%%", osGetCPUUsage());
    HAL_SuspendTick();
    if (KEY_FLAG)
    {
        GUI_Status_t buf;
        KEY_FLAG = 0;
        buf = Main_Response();
        if (buf != NOCHANGE) return buf;
    }

    return Showing;
}
