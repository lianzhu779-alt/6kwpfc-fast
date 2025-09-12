/*
 * relay_user.c
 *
 *  Created on: 2025年7月31日
 *      Author: oyzy
 */

/*
 * relay_control.c
 *
 * @brief 继电器控制逻辑的实现
 *
 * Created on: 2025年7月30日
 * Author: Gemini AI
 */

#include "relay_user.h"

// 为了代码清晰，定义继电器对应的GPIO引脚号
#define RELAY_A_GPIO   8  // AC, AB相继电器
#define RELAY_B_GPIO   0  // N相继电器

/**
 * @brief 初始化控制继电器的GPIO引脚
 */
void InitRelays(void)
{
    EALLOW;

    // --- 配置 RELAYA (GPIO8) ---
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;   // 1. 设置为通用IO功能
    GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;    // 2. 设置为输出方向
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;    // 3. 使能内部上拉 (当引脚为输入时有效，对于输出引脚影响不大，但按惯例配置)

    // --- 配置 RELAYB (GPIO0) ---
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;   // 1. 设置为通用IO功能
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;    // 2. 设置为输出方向
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;    // 3. 使能内部上拉

    EDIS;

    // 初始化时，默认进入安全关闭模式
    SetOperatingMode(MODE_OFF);
}

/**
 * @brief 设置系统的工作模式
 * @param mode 要设置的目标工作模式
 */
void SetOperatingMode(OperatingMode mode)
{
    // 根据传入的模式，控制两个继电器的开关状态
    // GpioDataRegs.GPASET.bit.GPIOx = 1;   // 设置GPIOx为高电平 (打开继电器)
    // GpioDataRegs.GPACLEAR.bit.GPIOx = 1; // 设置GPIOx为低电平 (关闭继电器)

    switch(mode)
    {
        case MODE_PFC:
            // PFC模式: RELAYB 打开, RELAYA 关闭
            GpioDataRegs.GPASET.bit.GPIO0 = 1;      // 打开 RELAYB
            GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;    // 关闭 RELAYA
            break;

        case MODE_BOOST:
            // Boost模式: RELAYB 打开, RELAYA 打开
            GpioDataRegs.GPASET.bit.GPIO0 = 1;      // 打开 RELAYB
            GpioDataRegs.GPASET.bit.GPIO8 = 1;      // 打开 RELAYA
            break;

        case MODE_OFF:
        default:
            // 默认或指定关闭模式: 所有继电器都关闭，确保系统安全
            GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;    // 关闭 RELAYA
            GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;    // 关闭 RELAYB
            break;
    }
}


