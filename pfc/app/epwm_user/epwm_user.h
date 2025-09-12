/*
 * epwm_user.h
 *
 * 创建于: 2025年6月22日
 * 作者: galaxy kono
 * 重构日期: 2025年7月30日
 *
 * @brief ePWM模块用户自定义配置的公共接口。
 * 此文件将PWM关键参数集中管理，以方便修改。
 */

#ifndef EPWM_USER_H_
#define EPWM_USER_H_

#include "DSP28x_Project.h"

//===========================================================================
// PWM 配置参数
//===========================================================================

#if (CPU_FRQ_150MHZ)
    // 系统时钟频率，单位: Hz。使用UL表示无符号长整型常量。
    #define SYSTEM_CLOCK_FREQ   150000000UL
#else
    #error "不支持的CPU频率。请为您的CPU频率定义相应的设置。"
#endif

// 期望的PWM开关频率，单位: Hz。
// 注意: 原始代码的注释提到频率为70kHz，但根据其代码计算
// (150MHz / (2 * (15000/2))) 的结果是10kHz。此配置将遵循计算结果，设为10kHz。
#define PWM_SWITCHING_FREQ  10000UL

// 对称模式(增减计数模式)下的时基周期寄存器(TBPRD)计算。
// PWM周期 = 2 * TBPRD * (1 / TBCLK)
// 因此, TBPRD = (SYSTEM_CLOCK_FREQ / PWM_SWITCHING_FREQ) / 2
#define EPWM_TIMER_TBPRD    (SYSTEM_CLOCK_FREQ / PWM_SWITCHING_FREQ / 2)

// 死区配置。
// 此值以TBCLK时钟周期数为单位。
// 原始代码的计算为 TB_PRD/50 => (150e6/10000)/50 = 300 个时钟周期。
#define EPWM_DEAD_BAND_CYCLES  300U

//===========================================================================
// 函数原型
//===========================================================================

/**
 * @brief 初始化并配置所有需要的ePWM模块 (ePWM2, ePWM3, ePWM4)。
 * @details
 * - 使用相同的频率、死区和动作限定逻辑来设置ePWM2, ePWM3, 和 ePWM4。
 * - 配置ePWM2以产生ADC的转换启动(SOC)信号。
 * - 配置ePWM3和ePWM4与ePWM2同步。
 */
void InitUserEpwmModules(void);


#endif /* EPWM_USER_H_ */
