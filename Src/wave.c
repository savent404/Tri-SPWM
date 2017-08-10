#include "wave.h"
#include "adc.h"

#define N 400 //20Khz / 50 = 400
float T1_Vrms = 0;
uint32_t VA_CNT = 0;

#ifdef WAVE_ARRY
uint32_t VA[N];
#else // 不适用Arry，直接在一周期上累加
uint64_t VA_VAL = 0;
#endif

void Arry_IN(void)
{
    int32_t buf = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    buf -= 2048;
    
#ifdef WAVE_ARRY
    VA[VA_CNT] = buf * buf;
#else
    VA_VAL += buf * buf;
#endif
    
    if (++VA_CNT == N)
    {
        VA_CNT = 0;
        
        T1_Vrms = GetVrms();

    }
}

float GetVrms(void)
{
#ifdef WAVE_ARRY
    for (uint32_t i = 0; i < N / 2; i++)
    {
        VA[2*i] += VA[2*i+1];
    }

    for (uint32_t i = 0; i < N / 4; i++)
    {
        VA[4*i] += VA[4*i+2];
    }

    uint64_t VA_VAL = 0;
    for (uint32_t i = 0; i < N / 8; i++)
    {
        VA_VAL += VA[8*i] + VA[8*i+4];
    }

    VA_VAL /= N;
    return sqrtf(VA_VAL);
#else
    VA_VAL /= N;
    return sqrtf(VA_VAL);
#endif
}