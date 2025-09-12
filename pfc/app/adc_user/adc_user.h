/*
 * adc_user.h
 * 三相电压源逆变器ADC采样与控制头文件 (已修正)
 *
 * 创建于: 2025年6月22日
 * 作者: galaxy kono
 */

#ifndef APP_ADC_USER_ADC_USER_H_
#define APP_ADC_USER_ADC_USER_H_

#include "DSP28x_Project.h"
#include "math.h"
#include "filter_user.h"
#include <string.h>

/* ========== 数学与物理常量定义 ========== */
#define PI                   3.14159265358979323846f
#define SQRT_3               1.73205080756887729353f  // √3
#define SQRT_3_DIV_2         0.86602540378443864676f  // √3/2
#define SQRT_3_DIV_3         0.57735026918962576451f  // √3/3
#define TWO_DIV_3            0.66666666666666666667f  // 2/3
#define ADC_TO_VOLTAGE       0.00073260073260073260f  // ADC转换系数 (3.0V / 4095)

/* ========== 系统参数定义 ========== */
#define SWITCHING_FREQ       10000.0f                 // 开关频率 10kHz
#define SAMPLING_PERIOD      0.0001f                  // 采样周期 Ts = 1/10kHz
#define PWM_PERIOD_NS        0.000000125f             // PWM时钟周期(ns), 根据实际PWM时钟调整
#define OUTPUT_VOLTAGE_REF   800.0f                   // 输出电压基准值 (用于PLL归一化)
#define SYSCLK_FREQ          150000000.0
/* ========== PI控制器参数 ========== */
#define KP_VOLTAGE           1.055f
#define KI_VOLTAGE           0.0002f
#define KP_CURRENT_D         2.44f
#define KI_CURRENT_D         0.0019f
#define KP_CURRENT_Q         2.44f
#define KI_CURRENT_Q         0.0019f

/* ========== 限幅参数 ========== */
#define VOLTAGE_LIMIT_MAX    2.5f * 1
#define VOLTAGE_LIMIT_MIN    1.7f * 1
#define CURRENT_D_LIMIT      80.0f
#define CURRENT_Q_LIMIT      80.0f

/* ========== 数据记录参数 ========== */
#define DATA_BUFFER_SIZE     200

/* ========== PWM时钟配置 ========== */
#define PWM_SYSCLK_FREQ_HZ         150000000UL      // 系统时钟频率 150MHz
#define PWM_SWITCHING_FREQ_HZ      10000UL          // 开关频率 10kHz
#define PWM_DEAD_TIME_PERCENT      2                // 死区时间百分比 2%

/* ========== PWM周期计算 ========== */
// 增减计数模式下：实际PWM频率 = SYSCLK / (2 * TBPRD)
// 因此：TBPRD = SYSCLK / (2 * 目标频率)
#define PWM_TBPRD_COUNT            (PWM_SYSCLK_FREQ_HZ / (2 * PWM_SWITCHING_FREQ_HZ))  // 7500
#define PWM_DEAD_TIME_COUNT        (PWM_TBPRD_COUNT / (100 / PWM_DEAD_TIME_PERCENT))    // 150

/* ========== 带通滤波器结构体定义  ========== */
typedef struct {
    float a1, a2; // 差分方程分母系数 y[n-1], y[n-2]
    float scaled; // 缩放因子
} FilterBandpassData_t;

typedef struct {
    float x_new;        // 当前输入
    float x_pre1;       // 前一拍输入
    float x_pre2;       // 前两拍输入
    float y_new;        // 当前输出
    float y_pre1;       // 前一拍输出
    float y_pre2;       // 前两拍输出
    float x_new_scaled; // 缩放后的当前输入
} FilterBandpassState_t;

/* ========== ADC原始数据结构体 ========== */
// 仅包含实际使用的通道
typedef struct {
    float output_voltage;     // 直流输出电压采样值
    float output_current;     // 直流输出电流采样值
    float ac_voltage_a;       // A相交流电压采样值
    float ac_voltage_b;       // B相交流电压采样值
    float ac_voltage_c;       // C相交流电压采样值
    float ac_current_a;       // A相交流电流采样值
    float ac_current_b;       // B相交流电流采样值
    float ac_current_c;       // C相交流电流采样值
    float ref_voltage_voltage_165; // 1.65V偏置电压采样值
    float ref_voltage_current_165; // 1.65A偏置电流采样

} AdcRawData_t;

/* ========== 三相坐标系结构体 ========== */
typedef struct {
    float phase_a;
    float phase_b;
    float phase_c;
} ThreePhase_t;

/* ========== Alpha-Beta静止坐标系结构体 ========== */
typedef struct {
    float alpha;
    float beta;
} AlphaBeta_t;

/* ========== D-Q旋转坐标系结构体 ========== */
typedef struct {
    float d;
    float q;
} DqAxis_t;

/* ========== 低通滤波器状态结构体 ========== */
typedef struct {
    float current;      // 当前输入
    float previous;     // 上一次的输出
    float filtered;     // 本次滤波输出
} FilterState_t;

/* ========== PI控制器结构体 ========== */
typedef struct {
    float kp, ki;           // PI系数
    float error, error_prev;// 误差与前一次误差
    float output;           // 控制器输出
    float max_limit, min_limit; // 输出限幅
} PIController_t;

/* ========== PLL锁相环结构体 ========== */
typedef struct {
    float theta;            // 锁定的相位角 (rad)
    float omega;            // 锁定的角频率 (rad/s)
    float sin_wt, cos_wt;   // 相位的正余弦值
    float theta_prev;// !!! 新增成员 !!!
    float sin_wt_prev;
    float cos_wt_prev;
    // PI控制器状态变量 (用于锁相)
//    float pi_state_1, pi_state_2, pi_prev;
    float pi_integrator_state; // 用于积分器累加
    float pi_output;           // PI控制器的总输出 (Δω)
    float error_prev;          // 用于增量式PI计算或调试
} PLL_t;

/* ========== SVPWM结构体 ========== */
typedef struct {
    float duty_a, duty_b,duty_c;        // 占空比
//    float t1, t2;           // 基础矢量作用时间
    float ta, tb, tc;       // 三相PWM波形比较时间
    float tcm1, tcm2, tcm3; // PWM比较寄存器值
    float ua,ub,uc;
    float vm_amplitude,modulation_index,phase_angle;
} SVPWM_t;

/* ========== 主控制系统结构体 ========== */
typedef struct {
    // 原始采样数据
    AdcRawData_t adc_raw;

    // 转换为物理单位的电压电流值
    ThreePhase_t voltage;
    ThreePhase_t current;

    // 经过带通滤波后的电压电流值 (用于控制)
    ThreePhase_t voltage_filtered;
    ThreePhase_t current_filtered;


    //采集的母线电压的转换结果
    AlphaBeta_t voltage_raw_ab;
    DqAxis_t voltage_raw_dq;

    // 坐标变换结果
    AlphaBeta_t voltage_ab;
    AlphaBeta_t current_ab;
    DqAxis_t voltage_dq;
    DqAxis_t current_dq;

    // 直流信号低通滤波器
    FilterState_t filter_vo;
    FilterState_t filter_io;
    FilterState_t filter_v165_current;
    FilterState_t filter_v165_voltage;

    float filter_VDC;

    // 交流信号带通滤波器
    FilterBandpassData_t  bp_coeffs;
    FilterBandpassState_t bp_va, bp_vb, bp_vc;
    FilterBandpassState_t bp_ia, bp_ib, bp_ic;

    // 锁相环内部使用的滤波器
    FilterState_t filter_vpd;
    FilterState_t filter_vpq;
    FilterState_t filter_vnd;
    FilterState_t filter_vnq;

    // PI控制器
    PIController_t pi_voltage;
    PIController_t pi_current_d;
    PIController_t pi_current_q;

    // 锁相环
    PLL_t pll;

    // SVPWM
    SVPWM_t svpwm;

    // 控制环路参考值
    float voltage_ref;
    float current_d_ref;
    float current_q_ref;

    // 控制器输出的指令电压
    DqAxis_t voltage_cmd;
    AlphaBeta_t voltage_ab_cmd;

    // --- 新增软启动相关变量 ---
    float voltage_ref_final;      // 最终的目标电压 (例如 800.0f)
    float voltage_ref_start;      // 软启动的起始电压 (例如 540.0f)
    float voltage_ramp_increment;   // 每个中断周期的电压增量
    int soft_start_complete;      // 软启动完成标志 (0: 进行中, 1: 已完成)

} ControlSystem_t;

/* ========== 数据记录结构体 (调试用) ========== */
typedef struct {
    float cos_wt[DATA_BUFFER_SIZE];
    float current_a[DATA_BUFFER_SIZE];
    float voltage_a[DATA_BUFFER_SIZE];
    float current_d[DATA_BUFFER_SIZE];
    float voltage_positive_q[DATA_BUFFER_SIZE];
    float current_dq_q[DATA_BUFFER_SIZE];
    float filter_VDC[DATA_BUFFER_SIZE];
    Uint16 index;
} DataLogger_t;

/* ========== 全局变量声明 ========== */
extern ControlSystem_t g_control_system;
extern DataLogger_t g_data_logger;

/* ========== 函数声明 ========== */
void ADC_Init(void);
interrupt void adc_isr(void);
void control_system_init(void);

/* ========== 内联函数 ========== */
// 限幅函数
static inline float saturate(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

// 角度归一化到 [0, 2*PI)
static inline float normalize_angle(float angle) {
    while (angle >= (2.0f * PI)) angle -= (2.0f * PI);
    while (angle < 0.0f) angle += (2.0f * PI);
    return angle;
}

#endif /* APP_ADC_USER_ADC_USER_H_ */
