#include "littleGUI.h"
#include "cpu_utils.h"
#include "PWM-Tri.h"

extern PWM_Tri_t PWM_TRI;


GUI_Set_t Param[Para_NUM] = {
    {"Vfix", 1000, 1000, 2000, 1, 1e-3f},
};

// 0-输入到Param
// 1-输出到实际参数
// 2-从flash输入参数
// 3-输出到flash
static void para_fresh(uint8_t flag)
{
    if (flag == 0) {
        Param[0].Val = (int)(PWM_TRI.fAmpFix / Param[0].gain);
    }
    else if (flag == 1)
    {
        PWM_TRI.fAmpFix = Param[0].Val * Param[0].gain;
    }
    else if (flag == 2)
    {

    }
    else if (flag == 3)
    {
        
    }
}


GUI_Status_t Start_Stage(void)
{
    return Showing_Fresh;
}



GUI_Status_t ParaSet_Stage(uint8_t init_flag)
{
    if (init_flag) {
        para_fresh(0);
        GUI_CLEAR();
        Param[Para_pos]._Val = Param[Para_pos].Val;
        CLEAR_CNT();
        SHOWSTRING(0, 0, Param[Para_pos].Header);
    }

    Param[Para_pos]._Val += GET_CNT();
    CLEAR_CNT();
    if (Param[Para_pos]._Val > Param[Para_pos].max) Param[Para_pos]._Val = Param[Para_pos].max;
    if (Param[Para_pos]._Val < Param[Para_pos].min) Param[Para_pos]._Val = Param[Para_pos].min;

    ShowPara_LL(&Param[Para_pos]);

    if (KEY_STATUS & KEY1_V)
    {
        while (KEY_STATUS & KEY1_V) { osDelay(5);}
        Para_pos += 1;
        Para_pos %= Para_NUM;
        SETFONT(20);
        SHOWSTRING(0, 50, "Next!");
        KEY_FLAG = 0;
        return Setting_Fresh;
    }

    if (KEY_STATUS & KEY2_V)
    {
        while (KEY_STATUS & KEY2_V) { osDelay(5);}
        SETFONT(20);
        SHOWSTRING(0, 50, "exit!");
        osDelay(200);
        Para_pos = 0;
        KEY_FLAG = 0;
        return Showing_Fresh;
    }

    if (KEY_STATUS & KEY0_V)
    {
        while (KEY_STATUS & KEY0_V) { osDelay(5); }
        Param[Para_pos].Val = LIMIT(Param[Para_pos]._Val, Param[Para_pos].max, Param[Para_pos].min);

        para_fresh(1);

        SETFONT(20);
        SHOWSTRING(0, 50, "Set OK!");
        KEY_FLAG = 0;
        return Setting_Fresh;
    }
    if (KEY_FLAG)
    {
        GUI_Status_t buf;
        KEY_FLAG = 0;
        buf = Main_Response();
        if (buf != NOCHANGE) return buf;
    }
    return Setting;
}

