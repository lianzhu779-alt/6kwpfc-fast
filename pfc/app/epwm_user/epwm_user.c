/*
 * epwm_user.c
 *
 * 创建于: 2025年6月22日
 * 作者: galaxy kono
 * @brief C28x系列DSP的ePWM模块用户自定义配置。
 * 提供了一个标准化的、可复用的函数来配置多个ePWM模块。
 */

#include "epwm_user.h"
#include <stdbool.h> // 引入此头文件以使用 bool 类型 (true/false)

// 本地辅助函数的静态前向声明
static void ConfigureEpwmModule(volatile struct EPWM_REGS *pEpwmRegs, bool enableSoc);

/**
 * @brief 初始化所有用户需要用到的ePWM外设。
 * @details 本函数为ePWM2, ePWM3, 和 ePWM4调用通用的配置函数。
 * ePWM2在此组中被配置为主模块，并负责产生SOC触发信号。
 */
void InitUserEpwmModules(void)
{

    InitEPwm2Gpio();
    InitEPwm3Gpio();
    InitEPwm4Gpio();

    // 步骤 1: 全局禁用ePWM时基时钟同步 (冻结所有ePWM时钟)
    // 这是实现同步最关键的第一步。将TBCLKSYNC位清零，会立即停止所有ePWM模块的
    // 时基时钟(TBCLK)，使它们的内部计数器(TBCTR)全部冻结。
    // 这就创造了一个安全的“配置窗口”，可以防止在配置过程中，各个模块
    // 因独立运行而导致时序错乱。好比赛跑前裁判喊“各就位，预备！”。
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=0;
    EDIS;

    // 步骤 2: 在时钟冻结期间，配置各个ePWM模块的基础参数
    // 在这个“静态”窗口内，我们可以安全地调用通用配置函数来设置每个模块的
    // 频率、占空比、死区、动作逻辑等。

    // 配置ePWM2，并使其能够产生ADC转换启动(SOC)脉冲。
    ConfigureEpwmModule(&EPwm2Regs, true);

    // 配置ePWM3和ePWM4，但不产生SOC脉冲。
    ConfigureEpwmModule(&EPwm3Regs, false);
    ConfigureEpwmModule(&EPwm4Regs, false);

    // 步骤 3: 设置模块间的主从同步关系
    EALLOW;
    // 配置ePWM2在计数器为0时，发出一个同步输出信号。
    // 这使得ePWM2成为本组模块的“主控”模块。
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;

    // 配置ePWM3和ePWM4作为“从属”模块，监听来自主控的同步信号。
    // 它们在接收到同步信号后，会复位自己的计数器。
    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;       // 使能相位加载
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;   // 设置同步信号来源为同步输入
    EPwm3Regs.TBPHS.half.TBPHS = 0;              // 相对于主控模块的相位偏移为0

    EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm4Regs.TBPHS.half.TBPHS = 0;
    EDIS;

    // 步骤 4: 全局使能ePWM时基时钟同步 (同时启动所有ePWM时钟)
    // 在所有模块都配置完毕、主从关系也设定好后，将TBCLKSYNC位置一。
    // 这会同时释放所有ePWM模块的时基时钟，让它们在同一时刻开始计数。
    // 好比裁判鸣响发令枪，所有选手同时起跑。
    // 这样，主模块发出的第一个同步信号就能被所有从模块精确捕获，从而保证完美的相位关系。
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=1;
    EDIS;
}

/**
 * @brief 使用通用设置来配置单个ePWM模块。
 * @param pEpwmRegs 指向易失性ePWM寄存器结构的指针 (例如: &EPwm2Regs)。
 * @param enableSoc 一个布尔标志，用于使能(true)或禁用(false)此模块的ADC SOC事件生成。
 */
static void ConfigureEpwmModule(volatile struct EPWM_REGS *pEpwmRegs, bool enableSoc)
{
    EALLOW;

    // --- 时基 (TB) 子模块 ---
    pEpwmRegs->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;      // 对称增减计数模式
    pEpwmRegs->TBPRD = EPWM_TIMER_TBPRD;                 // 设置定时器周期
    pEpwmRegs->TBCTR = 0x0000;                           // 清空计数器
    pEpwmRegs->TBCTL.bit.PHSEN = TB_DISABLE;             // 暂时禁用相位加载
    pEpwmRegs->TBPHS.half.TBPHS = 0x0000;                // 设置相位为0
    pEpwmRegs->TBCTL.bit.HSPCLKDIV = TB_DIV1;            // 高速时钟不分频
    pEpwmRegs->TBCTL.bit.CLKDIV = TB_DIV1;               // 时钟不分频

    // --- 计数器比较 (CC) 子模块 ---
    // 初始化为50%占空比。该值可以在应用程序中动态更新。
    pEpwmRegs->CMPA.half.CMPA = EPWM_TIMER_TBPRD / 2;

    // --- 动作限定 (AQ) 子模块 ---
    // 向上计数时，当 TBCTR == CMPA，设置输出高电平 (EPWMxA)
    pEpwmRegs->AQCTLA.bit.CAU = AQ_SET;
    // 向下计数时，当 TBCTR == CMPA，设置输出低电平 (EPWMxA)
    pEpwmRegs->AQCTLA.bit.CAD = AQ_CLEAR;

    // --- 死区 (DB) 子模块 ---
    pEpwmRegs->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;      // 使能A和B两路的死区功能
    pEpwmRegs->DBCTL.bit.POLSEL = DB_ACTV_HIC;          // 高有效互补模式 (EPWMxB的输出波形将被反向)
    pEpwmRegs->DBCTL.bit.IN_MODE = DBA_ALL;             // EPWMxA同时作为上升沿和下降沿延时的输入源
    pEpwmRegs->DBRED = EPWM_DEAD_BAND_CYCLES;           // 设置上升沿延时
    pEpwmRegs->DBFED = EPWM_DEAD_BAND_CYCLES;           // 设置下降沿延时

    // --- 事件触发 (ET) 子模块 (用于ADC转换启动) ---
    if (enableSoc)
    {
        pEpwmRegs->ETSEL.bit.SOCAEN = 1;                 // 使能SOCA信号
        pEpwmRegs->ETSEL.bit.SOCASEL = ET_CTR_ZERO;      // 在计数器等于0时触发
        pEpwmRegs->ETPS.bit.SOCAPRD = ET_1ST;            // 每1个事件触发一次
    }
    else
    {
        pEpwmRegs->ETSEL.bit.SOCAEN = 0;                 // 禁用此模块的SOCA信号
    }

    EDIS;
}
