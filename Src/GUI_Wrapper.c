#include "littleGUI.h"
#include "cpu_utils.h"
#include "PWM-Tri.h"

extern PWM_Tri_t PWM_TRI;

#define CODE_ERAE_ADDR  (0x800C000)
// Sector 3
const uint8_t code_erea[16*1024] @ CODE_ERAE_ADDR;

int16_t * const Para_pt[] = {
  (int16_t*)CODE_ERAE_ADDR + 0,
  (int16_t*)CODE_ERAE_ADDR + 1,
};

GUI_Set_t Param[Para_NUM] = {
    {"Vfix", 1000, 1000, 1500, 500, 1e-3f},
    {"Pfix", 0, 0, 1000, -1000, 1e-3f},
};

// 0-从内存加载实际参数到Param(Val)
// 1-从Param输出到实际参数(_Val)
// 2-从flash输入参数到Para(Val)
// 3-从Para(Val)输出到flash
void para_fresh(uint8_t flag)
{
    if (flag == 0)
    {
        Param[0].Val = (int)(PWM_TRI.fAmpFix / Param[0].gain);
        Param[1].Val = (int)(PWM_TRI.fPhaseOffset / Param[1].gain);
    }
    else if (flag == 1)
    {
        PWM_TRI.fAmpFix = Param[0].Val * Param[0].gain;
        PWM_TRI.fPhaseOffset = Param[1].Val * Param[1].gain;
    }
    else if (flag == 2)
    {
        Param[0].Val = *Para_pt[0];
        Param[1].Val = *Para_pt[1];
    }
    else if (flag == 3)
    {
        extern void FLASH_PageErase(uint32_t PageAddress);
        HAL_FLASH_Unlock();
        FLASH_PageErase(CODE_ERAE_ADDR);
        uint32_t status = FLASH_WaitForLastOperation((uint32_t) FLASH_TIMEOUT_VALUE);
        CLEAR_BIT(FLASH->CR, FLASH_CR_PER);

        if (status == HAL_OK)
        {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)Para_pt[0], Param[0].Val);
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)Para_pt[1], Param[1].Val);
        }

        HAL_FLASH_Lock();
    }
    
    Param[0]._Val = Param[0].Val;
    Param[1]._Val = Param[1].Val;
}


GUI_Status_t Start_Stage(void)
{
    para_fresh(2); // read from flash
    para_fresh(1); // write to ram

    LCD_240128_Init();
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    return Showing_Fresh;
}



GUI_Status_t ParaSet_Stage(uint8_t init_flag)
{
    if (init_flag) {

        para_fresh(2); // read from flash
        GUI_CLEAR();
        Param[Para_pos]._Val = Param[Para_pos].Val;
        CLEAR_CNT();
        SHOWSTRING(0, 0, Param[Para_pos].Header);
    }

    Param[Para_pos]._Val += GET_CNT();
    CLEAR_CNT();
    if (Param[Para_pos]._Val > Param[Para_pos].max) Param[Para_pos]._Val = Param[Para_pos].max;
    if (Param[Para_pos]._Val < Param[Para_pos].min) Param[Para_pos]._Val = Param[Para_pos].min;
    Param[Para_pos].Val = Param[Para_pos]._Val;
    
    para_fresh(1);

    ShowPara_LL(&Param[Para_pos]);

    if (KEY_STATUS & KEY1_V)
    {
        // while (KEY_STATUS & KEY1_V) { osDelay(5);}
        osDelay(100);
        Para_pos += 1;
        Para_pos %= Para_NUM;
        SETFONT(20);
        SHOWSTRING(0, 50, "Next!");
        KEY_FLAG = 0;
        return Setting_Fresh;
    }

    if (KEY_STATUS & KEY2_V)
    {
        // while (KEY_STATUS & KEY2_V) { osDelay(5);}
        osDelay(100);
        SETFONT(20);
        
        para_fresh(2); // Read form flash
        para_fresh(1); // write to ram
        
        SHOWSTRING(0, 50, "exit!");
        osDelay(200);
        Para_pos = 0;
        KEY_FLAG = 0;
        return Showing_Fresh;
    }

    if (KEY_STATUS & KEY0_V)
    {
        // while (KEY_STATUS & KEY0_V) { osDelay(5); }
        osDelay(100);
        Param[Para_pos].Val = LIMIT(Param[Para_pos]._Val, Param[Para_pos].max, Param[Para_pos].min);

        para_fresh(3); // write to flash

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

