# AD-DC

- 使用TIM1产生频率40KHz的SPWM波驱动逆变器1
- TIM1同时通过Update中断产生5K的SPWM更改频率用于产生交流信号
- 使用TIM6产生ADC采样触发(20KHz)
- 使用ADC1采集逆变器1三相电压
- 使用ADC2采集逆变器1，2三相电流

# 中断分析
- TIM1
        更新电压闭环设定值产生AD输出

# 电压闭环
- 200Hz更新频率

# 日志

## 8-11 2:38
能进行闭环调节输出电压了，PID参数需要调整。

## 8-11 8:21
中断频率10K使输出的50Hz减少了0.3Hz左右，只有降低中断频率到5K