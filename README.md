# AD-DC

- 使用TIM1产生频率40KHz的SPWM波驱动逆变器1
- TIM1同时通过Update中断产生5K的SPWM更改频率用于产生交流信号
- 使用TIM6产生ADC采样触发(20KHz)
- 使用ADC1采集逆变器1三相电压
- 使用ADC2采集逆变器1，2三相电流

# 中断分析
- TIM1
        更新电压闭环设定值产生AD输出
- TIM6
        采集ADC输入数组