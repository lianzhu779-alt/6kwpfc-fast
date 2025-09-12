/**
 ******************************************************************************
 * @file    main.c
 * @brief   主程序文件
 * @author  Kono
 * @date    2025.09.12
>>>>>>> a0008ecd9d6d73251bd0967cb1cb6244f9b4f7e2
 * @version V1.0
 *
 * @brief   该程序为DSP28x系列微控制器的初始化引导程序。
 * 主要任务包括：
 * 1. 初始化系统时钟和控制器。
 * 2. 将中断服务函数从Flash拷贝到RAM中以提高执行速度。
 * 3. 配置ADC模块，并设置由ePWM触发其转换。
 * 4. 配置ADC转换完成后的中断（SEQ1INT）。
 * 5. 初始化其他外设，如ePWM、继电器（Relay）、LED等。
 * 6. 系统进入一个无限循环，所有主要工作在ADC中断服务函数 adc_isr() 中执行。
 ******************************************************************************
 */

#include "DSP28x_Project.h"
#include "math.h"
#include "stddef.h"
#include "stdint.h"

// 包含用户自定义的模块头文件
#include "epwm_user.h"
#include "adc_user.h"
#include "led_user.h"
#include "relay_user.h"

// --- 全局变量声明 ---
// 这些变量用于将代码从Flash拷贝到RAM
#pragma CODE_SECTION(adc_isr, "ramfuncs");
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
extern Uint16 RamfuncsLoadSize;

// --- 函数原型声明 ---
interrupt void adc_isr(void);
void System_Init(void);
void MemCopy_FlashToRam(void);
void Interrupt_Init(void);
void Peripherals_Init(void);

// --- 主函数 ---
void main()
{
    // 1. 系统核心初始化
    System_Init();

    // 2. 将指定代码段从Flash拷贝到RAM
    MemCopy_FlashToRam();

    // 3. 初始化所有外设
    Peripherals_Init();

    // 4. 配置中断
    Interrupt_Init();

    // --- 主循环 ---
    // 嵌入式系统通常在初始化后进入一个无限循环。
    // 实际的控制逻辑由中断服务函数驱动。
    while(1)
    {
        // 主循环可以执行一些优先级不高的任务，
        // 例如：状态监控、通信处理、LED闪烁等。
    }
}


/**
 * @brief  系统核心初始化
 * @param  None
 * @retval None
 */
void System_Init(void)
{
    // 初始化系统控制寄存器，包括PLL, Watchdog, Clocks等
    InitSysCtrl();

    // 初始化Flash控制器（等待状态、流水线等）
    // 必须在将代码拷贝到RAM之前或之后进行，具体取决于应用
    InitFlash();
}


/**
 * @brief  将 .ramfuncs 代码段从Flash拷贝到RAM
 * @param  None
 * @retval None
 * @note   这是为了提高中断等关键代码的执行速度。
 */
void MemCopy_FlashToRam(void)
{
    // 使用C标准库的memcpy函数执行拷贝操作
    // RamfuncsLoadStart: 源地址（在Flash中）
    // RamfuncsRunStart:  目标地址（在RAM中）
    // RamfuncsLoadSize:  拷贝的数据大小
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
}


/**
 * @brief  初始化所有应用所需的外设
 * @param  None
 * @retval None
 * @note   注意：原始代码中有两次ADC初始化（InitAdc和ADC_Init），这是冗余的。
 * 这里将其合并为一次调用。
 */
void Peripherals_Init(void)
{
    // 初始化ADC模块
    ADC_Init();

    // 初始化继电器
    InitRelays();

    // 初始化用户定义的ePWM模块
    InitUserEpwmModules();

    // 初始化LED
    LED_init();

    // 初始化自定义的控制系统（例如，变量清零、状态机复位等）
    control_system_init();

    // 设置初始工作模式（例如，PFC模式）
    SetOperatingMode(MODE_PFC);
}


/**
 * @brief  配置中断
 * @param  None
 * @retval None
 */
void Interrupt_Init(void)
{
    // --- 中断配置流程 ---
    // 1. 关闭全局中断，防止在配置过程中发生意外中断
    DINT;

    // 2. 使能对受保护寄存器（如PIE）的写操作
    EALLOW;

    // 3. 初始化PIE (Peripheral Interrupt Expansion) 控制寄存器
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;

    // 4. 将中断服务函数地址加载到PIE向量表
    // SEQ1INT 对应的是 ADC 序列1 转换完成中断
    PieVectTable.SEQ1INT = &adc_isr;

    // 5. 使能ADC外设级别的中断
    // 允许ePWM的SOCA信号启动ADC的SEQ1转换序列
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;
    // 使能SEQ1序列转换完成后的中断请求
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;

    // 6. 使能PIE级别的中断
    // PIE第1组的第1个中断就是 SEQ1INT (ADC)
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    // 7. 使能CPU级别的中断
    // IER的第一位对应PIE的INT1中断组
    IER |= M_INT1; // 等效于 IER |= 0x0001;

    // 8. 禁止对受保护寄存器的写操作，防止误改
    EDIS;

    // 9. 开启全局中断，系统开始响应中断请求
    EINT;
}
