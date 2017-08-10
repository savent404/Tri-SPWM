#include "wave.h"
#include "adc.h"

#define N 800 //20Khz / 50 = 400
float T1_Vrms = 0;
uint32_t VA_CNT = 0;

uint16_t VA_M[2] = {0, 0}; // 最大、最小值
uint16_t _VA_M[2] = {0, 4096};
void Arry_IN(void)
{
    uint16_t buf = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);

    if (_VA_M[0] < buf) _VA_M[0] = buf;
    if (_VA_M[1] > buf) _VA_M[1] = buf;

    if (++VA_CNT >= N)
    {
        VA_CNT = 0;
        
        T1_Vrms = GetVrms() * 24.0f;
        
        printf("%.4f\t%d\t%d\r\n", T1_Vrms, VA_M[0], VA_M[1]);
        
        _VA_M[0] = 0;
        _VA_M[1] = 4096;
    }
}

float GetVrms(void)
{
    VA_M[0] += _VA_M[0];
    VA_M[1] /= 2;
    VA_M[1] += _VA_M[1];
    VA_M[1] /= 2;
    int sub = VA_M[0] - VA_M[1];

    return sub * 3.3f / 4096 * 24.0f / 2 / sqrtf(2);
}