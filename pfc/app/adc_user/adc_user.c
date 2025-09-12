/*
 * =====================================================================================
 *
 * 文件:  adc_user.c
 *
 * 说明:  三相电压源逆变器ADC采样与控制核心实现文件。
 * 本文件经过重构，以解决编译错误并优化信号处理流程。
 * 集成了代码一的二阶带通滤波器，以替代原有的低通滤波方案，
 * 从而更精确地提取基波信号，提高锁相环和控制系统的性能。
 *
 * 创建于:  2025年6月22日
 * 作者:  galaxy kono dsa
 *
 * =====================================================================================
 */

#include "adc_user.h"
#include "filter_user.h" // 确保包含了filter_1_x和filter_2_x的定义

/* ========== 全局变量定义 ========== */
// 定义全局的控制系统结构体实例，用于存储所有控制相关的变量
ControlSystem_t g_control_system = {0};
// 定义全局的数据记录结构体实例，用于调试和波形观测
DataLogger_t g_data_logger = {0};

/* ========== 静态函数（内部函数）声明 ========== */
// 硬件层与信号预处理
static void adc_read_channels(void);            // 从ADC寄存器读取各通道的原始值
static void apply_dc_filters(void);             // 对直流信号（如1.65V偏置）进行低通滤波
static void apply_offset_and_gain(void);        // 减去直流偏置并应用增益，转换为真实物理单位
static void apply_bandpass_filters(void);       // 对交流信号进行带通滤波，提取基波

// 控制算法核心
static void pll_process(void);                  // 锁相环（PLL）处理，用于同步电网相位和频率
static void clarke_transform(void);             // Clarke变换 (abc -> αβ)
static void park_transform(void);               // Park变换 (αβ -> dq)
static void pi_control_process(void);           // 双闭环PI控制器处理（电压外环，电流内环）
static void inverse_park_transform(void);       // 逆Park变换 (dq -> αβ)
static void svpwm_process(void);                // SVPWM调制算法，生成PWM占空比
static void update_pwm_compare(void);           // 将计算出的占空比更新到PWM硬件寄存器

// 辅助与初始化
static void data_logging(void);                 // 数据记录，用于调试
static void init_bandpass_filter_coeffs(FilterBandpassData_t *filter_data); // 初始化带通滤波器系数
static float filter_bandpass(float x_new, FilterBandpassState_t *filter, const FilterBandpassData_t *filter_data); // 带通滤波器实现
static Uint32 duty_to_pwm_register(float duty_ratio);
static void soft_start_process(void);

/*
 *--------------------------------------------------------------------------------------
 * 函数:  adc_config
 * 说明:  配置ADC模块。此函数与硬件紧密相关，根据实际电路连接配置ADC通道和采样模式。
 *--------------------------------------------------------------------------------------
 */
void ADC_Init(void)
{
    InitAdc();
    EALLOW; // 允许访问受保护的寄存器

    // 配置ADC时钟: ADCCLK = HSPCLK / 1 = 150MHz / 1 = 150MHz
    // 注意: 根据DSP型号和系统时钟，ADCCLK通常需要分频以满足ADC规格要求（如<=25MHz）
    // 这里假设SysCtrlRegs.HISPCP.all已在别处正确配置，例如为3，则ADCCLK=25MHz
    // SysCtrlRegs.HISPCP.all = 3;
    SysCtrlRegs.HISPCP.all= 3;                  //HSPCLK 3分频 150M/3=50M

    AdcRegs.ADCTRL1.bit.ACQ_PS=5;               //采样时间
    AdcRegs.ADCTRL1.bit.CPS=0;                  //外设时钟不分频
    AdcRegs.ADCTRL3.bit.ADCCLKPS=0;             //二分频 ADCLK=HSPCLK/2=25MHz

    AdcRegs.ADCTRL1.bit.CONT_RUN=0;//读取完转换序列后停止
    AdcRegs.ADCTRL3.bit.SMODE_SEL=0;//顺序采样模式
    AdcRegs.ADCTRL1.bit.SEQ_CASC=1;//级联排序器模式

    AdcRegs.ADCMAXCONV.bit.MAX_CONV1=12;//这个数字要比用到的采样通道多一个

    // --- 配置ADC采样通道 (根据实际硬件连接) ---
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0B; // 通道0采样ADCINB3 -> VO (直流输出电压)
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x0A; // 通道1采样ADCINB3 -> IO (直流输出电压)
    AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0xC;  //通道2作为ADCINB4 -> 1.65V (偏置电压)
    AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0xC;  //通道3采样ADCINB4 -> 1.65V (偏置电压)

    AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0x00; // 通道6采样ADCINA0 -> VA (A相交流电压)
    AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 0x08; // 通道7采样ADCINB0 -> VB (B相交流电压)
    AdcRegs.ADCCHSELSEQ3.bit.CONV08 = 0x09; // 通道8采样ADCINB1 -> VC (C相交流电压)

    AdcRegs.ADCCHSELSEQ3.bit.CONV09 = 0x03; // 通道09采样ADCINA9 -> IA (A相交流电流)
    AdcRegs.ADCCHSELSEQ3.bit.CONV10 = 0x0F; // 通道10采样ADCINB10 -> IB (B相交流电流)
    AdcRegs.ADCCHSELSEQ3.bit.CONV11 = 0x0E; // 通道11采样ADCINB11 -> IC (C相交流电流)

    EDIS; // 禁止访问受保护的寄存器采样ADCINB4 -> 1.65V (偏置电压)
}

/*
 *--------------------------------------------------------------------------------------
 * 函数:  adc_isr
 * 说明:  ADC中断服务程序。这是整个控制系统的核心，以固定的开关频率被调用。
 * 它定义了所有计算和控制任务的执行顺序。
 *--------------------------------------------------------------------------------------
 */
interrupt void adc_isr(void)
{
    // 新增：在所有控制计算之前，先执行软启动逻辑
    soft_start_process();

    // 步骤 1: 读取所有相关ADC通道的原始转换结果
    adc_read_channels();

    // 步骤 2: 对需要慢速响应的直流信号进行低通滤波，以获得稳定的值
    // 例如1.65V偏置电压，它理论上应该是恒定的
    apply_dc_filters();

    // 步骤 3: 利用滤波后的偏置电压，对原始AC采样值进行去偏置处理，并应用增益系数
    // 将其从0-3.6V的采样电压转换为真实的电压(V)和电流(A)值
    apply_offset_and_gain();

    // 步骤 4: 对已转换为物理单位的交流信号应用带通滤波器
    // 这是关键一步，目的是精确提取50Hz的基波分量，滤除高频噪声和直流残余
    apply_bandpass_filters();

    // 步骤 5: 执行锁相环算法，使用滤波后的电压信号来精确跟踪电网的相位和频率
    pll_process();

    // 步骤 6: 执行Clarke变换，将三相静止坐标系(abc)下的电流量转换到两相静止坐标系(αβ)
    clarke_transform();

    // 步骤 7: 执行Park变换，利用锁相环提供的角度，将两相静止(αβ)下的电压和电流量
    // 转换到两相同步旋转坐标系(dq)
    park_transform();

    // 步骤 8: 执行PI控制器，在dq坐标系下实现电压和电流的双闭环控制
    pi_control_process();

    // 步骤 9: 执行逆Park变换，将PI控制器输出的dq指令电压转换回两相静止坐标系(αβ)
    inverse_park_transform();

    // 步骤 10: 执行SVPWM算法，根据αβ指令电压计算出三相PWM的开关时间和占空比
    svpwm_process();

    // 步骤 11: 将计算出的SVPWM比较值写入ePWM模块的硬件寄存器
    update_pwm_compare();

    // 步骤 12: (调试用) 将关键变量存入缓冲区，用于通过CCS或示波器观察波形
    data_logging();

    // --- 中断结束前的清理工作 ---
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;      // 复位ADC序列发生器
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;    // 清除ADC序列1的中断标志位
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // 向PIE控制器应答中断，允许响应同组的其他中断
}

/*
 *--------------------------------------------------------------------------------------
 * 函数:  adc_read_channels
 * 说明:  从ADC结果寄存器中读取数据，并转换为0-3.0V范围内的浮点电压值。
 *--------------------------------------------------------------------------------------
 */
static void adc_read_channels(void)
{
    ControlSystem_t *sys = &g_control_system;

    // 右移4位是为了将12位ADC结果对齐。ADCRESULT寄存器是16位的，结果左对齐存放。

    // 根据 adc_config 中的 CONV00 配置，读取直流输出电压
    sys->adc_raw.output_voltage      = (AdcRegs.ADCRESULT0 >> 4) * ADC_TO_VOLTAGE;

    // 根据 adc_config 中的 CONV01 配置，读取直流输出电流
    sys->adc_raw.output_current      = (AdcRegs.ADCRESULT1 >> 4) * ADC_TO_VOLTAGE; // 假设结构体中有此成员

    // 根据 adc_config 中的 CONV02 和 CONV03 配置，读取偏置电压
    // 您可以根据需要选择存储一个或两个值
    sys->adc_raw.ref_voltage_current_165 = (AdcRegs.ADCRESULT2 >> 4) * ADC_TO_VOLTAGE; // 假设为电流环的偏置
    sys->adc_raw.ref_voltage_voltage_165 = (AdcRegs.ADCRESULT3 >> 4) * ADC_TO_VOLTAGE; // 假设为电压环的偏置

    // 根据 adc_config 中的 CONV06 配置，读取A相交流电压
    sys->adc_raw.ac_voltage_a        = (AdcRegs.ADCRESULT6 >> 4) * ADC_TO_VOLTAGE;

    // 根据 adc_config 中的 CONV07 配置，读取B相交流电压
    sys->adc_raw.ac_voltage_b        = (AdcRegs.ADCRESULT7 >> 4) * ADC_TO_VOLTAGE;

    // 根据 adc_config 中的 CONV08 配置，读取C相交流电压
    sys->adc_raw.ac_voltage_c        = (AdcRegs.ADCRESULT8 >> 4) * ADC_TO_VOLTAGE;

    // 根据 adc_config 中的 CONV09 配置，读取A相交流电流
    sys->adc_raw.ac_current_a        = (AdcRegs.ADCRESULT9 >> 4) * ADC_TO_VOLTAGE;

    // 根据 adc_config 中的 CONV10 配置，读取B相交流电流
    sys->adc_raw.ac_current_b        = (AdcRegs.ADCRESULT10 >> 4) * ADC_TO_VOLTAGE;

    // 根据 adc_config 中的 CONV11 配置，读取C相交流电流
    sys->adc_raw.ac_current_c        = (AdcRegs.ADCRESULT11 >> 4) * ADC_TO_VOLTAGE;
}

/*
 *--------------------------------------------------------------------------------------
 * 函数:  apply_dc_filters
 * 说明:  应用一阶低通滤波器来平滑直流信号。
 *--------------------------------------------------------------------------------------
 */
static void apply_dc_filters(void)
{
    ControlSystem_t *sys = &g_control_system;
    const float32 filter_coeff = 0.001f; // 统一的慢速滤波系数

    // 对直流母线电压 (Vo) 进行低通滤波
    sys->filter_vo.current = sys->adc_raw.output_voltage;
    // 使用滤波系数 0.001f
    sys->filter_vo.filtered = filter_2_x(sys->filter_vo.current, sys->filter_vo.previous, filter_coeff);
    sys->filter_vo.previous = sys->filter_vo.filtered;
    sys->filter_VDC = (sys->filter_vo.filtered - 0.011) * OUTPUT_VOLTAGE_REF/2;

    // 对直流输出电流 (Io) 进行低通滤波
    sys->filter_io.current = sys->adc_raw.output_current;
    sys->filter_io.filtered = filter_2_x(sys->filter_io.current, sys->filter_io.previous, filter_coeff);
    sys->filter_io.previous = sys->filter_io.filtered;

    // 对电流环的1.65V偏置电压进行低通滤波
    sys->filter_v165_current.current = sys->adc_raw.ref_voltage_current_165;
    sys->filter_v165_current.filtered = filter_2_x(sys->filter_v165_current.current, sys->filter_v165_current.previous, filter_coeff);
    sys->filter_v165_current.previous = sys->filter_v165_current.filtered;

    // 对电压环的1.65V偏置电压进行低通滤波
    sys->filter_v165_voltage.current = sys->adc_raw.ref_voltage_voltage_165;
    sys->filter_v165_voltage.filtered = filter_2_x(sys->filter_v165_voltage.current, sys->filter_v165_voltage.previous, filter_coeff);
    sys->filter_v165_voltage.previous = sys->filter_v165_voltage.filtered;


}

/*
 *--------------------------------------------------------------------------------------
 * 函数:  apply_offset_and_gain
 * 说明:  进行偏移校正和增益转换，得到真实的物理值。
 *--------------------------------------------------------------------------------------
 */


static void apply_offset_and_gain(void)
{
    ControlSystem_t *sys = &g_control_system;
    float v165_filtered = sys->filter_v165_voltage.filtered;
//    float A165_filtered = sys->filter_v165_current.filtered;

    // 1. 从AC信号的采样值中减去滤波后得到的稳定直流偏置
    float va_offset = sys->adc_raw.ac_voltage_a - v165_filtered;
    float vb_offset = sys->adc_raw.ac_voltage_b - v165_filtered;
    float vc_offset = sys->adc_raw.ac_voltage_c - v165_filtered;

    float ia_offset = sys->adc_raw.ac_current_a - v165_filtered;
    float ib_offset = sys->adc_raw.ac_current_b - v165_filtered;
    float ic_offset = sys->adc_raw.ac_current_c - v165_filtered;

    // 2. 应用硬件电路的增益系数，将校正后的电压信号转换为真实的物理单位
    sys->voltage.phase_a = va_offset * 487.8f;
    sys->voltage.phase_b = vb_offset * 487.8f;
    sys->voltage.phase_c = vc_offset * 487.8f;

    // 根据代码一的逻辑，电流信号需要取反。这通常与电流传感器的方向或接法有关。
    sys->current.phase_a = -ia_offset * 40.0f;
    sys->current.phase_b = -ib_offset * 40.0f;
    sys->current.phase_c = -ic_offset * 40.0f;
}

/*
 *--------------------------------------------------------------------------------------
 * 函数:  apply_bandpass_filters
 * 说明:  对三相交流电压和电流应用带通滤波器。
 *--------------------------------------------------------------------------------------
 */
static void apply_bandpass_filters(void)
{
    ControlSystem_t *sys = &g_control_system;
    // 将经过增益转换后的物理值输入带通滤波器
    sys->voltage_filtered.phase_a = filter_bandpass(sys->voltage.phase_a, &sys->bp_va, &sys->bp_coeffs);
    sys->voltage_filtered.phase_b = filter_bandpass(sys->voltage.phase_b, &sys->bp_vb, &sys->bp_coeffs);
    sys->voltage_filtered.phase_c = filter_bandpass(sys->voltage.phase_c, &sys->bp_vc, &sys->bp_coeffs);

    sys->current_filtered.phase_a = filter_bandpass(sys->current.phase_a, &sys->bp_ia, &sys->bp_coeffs);
    sys->current_filtered.phase_b = filter_bandpass(sys->current.phase_b, &sys->bp_ib, &sys->bp_coeffs);
    sys->current_filtered.phase_c = filter_bandpass(sys->current.phase_c, &sys->bp_ic, &sys->bp_coeffs);
}


/**
 * @brief 双同步坐标系锁相环(DDSOGI-PLL)核心处理函数
 * @details
 * 它使用在 adc_user.h 中定义的全局结构体 g_control_system 进行所有计算和状态更新。
 * 流程: Clarke变换 -> 正负序分离 -> 交叉解耦 -> 低通滤波 -> PI调节 -> 频率/相位更新
 */
static void pll_process(void)
{
    // 从全局结构体中获取工作所需的子结构体指针，方便读写
    ControlSystem_t *sys = &g_control_system;
    PLL_t *pll = &sys->pll;

    // !!! 关键步骤: 在所有计算开始前，保存上一拍的角度快照,该pi参数建立在一拍延迟上!!!
    pll->theta_prev  = pll->theta;
    pll->cos_wt_prev = pll->cos_wt;
    pll->sin_wt_prev = pll->sin_wt;
    /* ================================================================================= */
    /* 步骤 1: Clarke 变换 (abc -> αβ)                                                 */
    /* 注意：此处进行了归一化处理，不可作为电流环输出的参考电压                                */
    /* ================================================================================= */
    // 从滤波后的三相电压进行变换
    float va = sys->voltage_filtered.phase_a;
    float vb = sys->voltage_filtered.phase_b;
    float vc = sys->voltage_filtered.phase_c;

    // 计算归一化后的 Alpha-Beta 坐标
    sys->voltage_ab.alpha = TWO_DIV_3 * (va / OUTPUT_VOLTAGE_REF - 0.5f * vb / OUTPUT_VOLTAGE_REF - 0.5f * vc / OUTPUT_VOLTAGE_REF);
    sys->voltage_ab.beta  = SQRT_3_DIV_3 * (vb / OUTPUT_VOLTAGE_REF - vc / OUTPUT_VOLTAGE_REF);


    /* ================================================================================= */
    /* 步骤 2: 计算原始的正序和负序d-q分量               */
    /* ================================================================================= */
    float v_alpha = sys->voltage_ab.alpha;
    float v_beta  = sys->voltage_ab.beta;
    float cos_wt  = pll->cos_wt_prev;
    float sin_wt  = pll->sin_wt_prev;

    float vpd_raw =  v_alpha * cos_wt + v_beta * sin_wt;
    float vpq_raw = -v_alpha * sin_wt + v_beta * cos_wt;
    float vnd_raw =  v_alpha * cos_wt - v_beta * sin_wt;
    float vnq_raw =  v_alpha * sin_wt + v_beta * cos_wt;


    /* ================================================================================= */
    /* 步骤 3: 正负序交叉解耦与低通滤波 (Decoupling)                                  */
    /* ================================================================================= */
    float sin_2wt = sinf(2.0f * pll->theta_prev);
    float cos_2wt = cosf(2.0f * pll->theta_prev);

    // 解耦正序d轴电压 (Vpd)
    float vpd_decoupled = vpd_raw - (cos_2wt * sys->filter_vnd.filtered + sin_2wt * sys->filter_vnq.filtered);
    sys->filter_vpd.filtered = filter_1_x(vpd_decoupled, sys->filter_vpd.previous, 0.062832f);
    sys->filter_vpd.previous = sys->filter_vpd.filtered;

    // 解耦正序q轴电压 (Vpq)
    float vpq_decoupled = vpq_raw - (-sin_2wt * sys->filter_vnd.filtered + cos_2wt * sys->filter_vnq.filtered);
    sys->filter_vpq.filtered = filter_1_x(vpq_decoupled, sys->filter_vpq.previous, 0.062832f);
    sys->filter_vpq.previous = sys->filter_vpq.filtered;

    // 解耦负序d轴电压 (Vnd)
    float vnd_decoupled = vnd_raw - (cos_2wt * sys->filter_vpd.filtered - sin_2wt * sys->filter_vpq.filtered);
    sys->filter_vnd.filtered = filter_1_x(vnd_decoupled, sys->filter_vnd.previous, 0.062832f);
    sys->filter_vnd.previous = sys->filter_vnd.filtered;

    // 解耦负序q轴电压 (Vnq)
    float vnq_decoupled = vnq_raw - (sin_2wt * sys->filter_vpd.filtered + cos_2wt * sys->filter_vpq.filtered);
    sys->filter_vnq.filtered = filter_1_x(vnq_decoupled, sys->filter_vnq.previous, 0.062832f);
    sys->filter_vnq.previous = sys->filter_vnq.filtered;


    /*
    * =================================================================================
    * 步骤 4: PI (比例-积分) 控制器
    * ---------------------------------------------------------------------------------
    * 本部分是锁相环频率回路的核心。它通过一个PI控制器，处理代表相位误差的
    * q轴电压分量(vq_error)，目标是将其驱动至零。控制器的输出是频率补偿量(Δω)，
    * 用于修正对电网频率的估计。
    * =================================================================================
    */

    // 4.1 定义PI控制器系数
    // 为了提高计算效率，这里使用了PI控制器的差分方程预计算系数形式:
    // u(k) = u(k-1) + Kp*(e(k) - e(k-1)) + Ki*T*e(k)
    // 整理后可得: u(k) = u(k-1) + (Kp + Ki*T)*e(k) - Kp*e(k-1)
    // 其中: u(k)是当前输出, u(k-1)是上次输出, e(k)是当前误差, e(k-1)是上次误差。
    // PI_COEFF_A 对应 (Kp + Ki*T)，是比例和积分项组合系数
    // PI_COEFF_B 对应 Kp，是纯比例项系数
    const float PI_COEFF_A = 166.9743385f;
    const float PI_COEFF_B = 166.2661165f;

    // 4.2 获取误差信号并进行输入限幅
    // 从滤波器获取解耦后的q轴电压，它在锁相成功时应接近于0，作为PI控制器的误差输入。
    float vq_error = sys->filter_vpq.filtered;
    // 对输入误差进行限幅，可以防止在电网电压发生剧烈扰动时，过大的误差冲击导致PI控制器输出剧变。
    vq_error = saturate(vq_error, -0.1f, 0.1f);

    // 4.3 执行PI控制器计算
    // 此行代码实现了上述的PI差分方程。
    // pll->pi_integrator_state 存储了上一次的输出 u(k-1)。
    // pll->error_prev 存储了上一次的误差 e(k-1)。
    pll->pi_output = pll->pi_integrator_state + PI_COEFF_A * vq_error - PI_COEFF_B * pll->error_prev;

    // 4.4 更新历史误差值
    // 保存本次的误差值，以便在下一次迭代中作为 e(k-1) 使用。
    pll->error_prev = vq_error;

    // 4.5 对PI输出进行限幅 (抗积分饱和)
    // 限制PI控制器的最大输出，即限制PLL允许的最大频率偏移量 (此处为 +/- 5Hz)。
    // 这可以防止PLL在启动或同步过程中请求一个不切实际的频率。
    const float FREQ_DEV_LIMIT = 5.0f * 2.0f * PI; // 5Hz 对应的角频率偏移量
    pll->pi_output = saturate(pll->pi_output, -FREQ_DEV_LIMIT, FREQ_DEV_LIMIT);

    // 4.6 更新积分器状态 (抗积分饱和的关键步骤)
    // 这是“积分钳位”或“返回计算”抗饱和策略。通过将饱和后的输出值回传给积分状态变量，
    // 可以防止在输出被限幅时积分项继续累加，从而避免积分饱和。这使得PLL在脱离饱和状态时能更快地恢复。
    pll->pi_integrator_state = pll->pi_output;


    /*
    * =================================================================================
    * 步骤 5: 更新角频率和相位
    * ---------------------------------------------------------------------------------
    * 本部分利用PI控制器的输出结果，更新锁相环的内部频率和相位角，
    * 以便在下一周期生成正确的正余弦参考信号。
    * =================================================================================
    */

    // PI控制器的输出 (pll->pi_output) 即为计算出的角频率补偿量 Δω。
    float freq_deviation = pll->pi_output;

    // 将补偿量与电网的标称角频率 (50Hz对应) 相加，得到最终估算的电网角频率 ω。
    pll->omega = (2.0f * PI * 50.0f) + freq_deviation;

    // 通过对角频率进行积分来更新相位角 θ。
    // 这是一个离散时间积分 (前向欧拉法): θ_new = θ_old + ω * Δt
    pll->theta += pll->omega * SAMPLING_PERIOD;

    // 相位角归一化 (wrapping)，确保其值保持在 [0, 2π) 区间内。
    if (pll->theta >= (2.0f * PI))
    {
        pll->theta -= (2.0f * PI);
    }
    // 注意: 此处的负值处理方式比较特殊，直接置零而不是加上2π。
    // 这种处理方式可能是为了应对特定的瞬态情况或简化逻辑。
    else if (pll->theta < 0.0f)
    {
        pll->theta = 0.0f;
    }

    /* ================================================================================= */
    /* 步骤 6: 更新正余弦值，供下一次迭代使用                                            */
    /* ================================================================================= */
    pll->sin_wt = sinf(pll->theta);
    pll->cos_wt = cosf(pll->theta);


    /* ================================================================================= */
    /* 步骤 7: 将最终解耦后的d-q电压存入主结构体，供其他控制环路使用                     */
    /* ================================================================================= */
    sys->voltage_dq.d = sys->filter_vpd.filtered; // 最终的正序d轴电压
    sys->voltage_dq.q = sys->filter_vpq.filtered; // 最终的正序q轴电压 (理论上应趋近于0)
}
/*
 *--------------------------------------------------------------------------------------
 * 函数:  clarke_transform
 * 说明:  执行电流的Clarke变换。
 *--------------------------------------------------------------------------------------
 */
static void clarke_transform(void)
{
    ControlSystem_t *sys = &g_control_system;

    // 直接从滤波后的原始三相电流获取输入
    float ia = sys->current_filtered.phase_a;
    float ib = sys->current_filtered.phase_b;
    float ic = sys->current_filtered.phase_c;

    // 直接从滤波后的原始三相电压获取输入
    float va = sys->voltage_filtered.phase_a;
    float vb = sys->voltage_filtered.phase_b;
    float vc = sys->voltage_filtered.phase_c;

    // 幅值不变Clarke变换
    sys->current_ab.alpha = TWO_DIV_3 * (ia - 0.5f * ib - 0.5f * ic);
    sys->current_ab.beta = SQRT_3_DIV_3 * (ib - ic);

    // 幅值不变Clarke变换
    sys->voltage_raw_ab.alpha = TWO_DIV_3 * (va - 0.5f * vb - 0.5f * vc);
    sys->voltage_raw_ab.beta = SQRT_3_DIV_3 * (vb - vc);
}

/*
 *--------------------------------------------------------------------------------------
 * 函数:  park_transform
 * 说明:  执行电压和电流的Park变换。
 *--------------------------------------------------------------------------------------
 */
static void park_transform(void)
{
    ControlSystem_t *sys = &g_control_system;
    PLL_t *pll = &sys->pll;

    // 从Clarke变换结果中获取电流alpha, beta
    float Ialpha = sys->current_ab.alpha;
    float Ibeta = sys->current_ab.beta;

    // 从PLL的中间计算结果中获取电压alpha, beta
    float Valpha = sys->voltage_raw_ab.alpha;
    float Vbeta = sys->voltage_raw_ab.beta;

    // 执行电流的Park变换
    sys->current_dq.d =  Ialpha * pll->cos_wt_prev + Ibeta  * pll->sin_wt_prev;
    sys->current_dq.q = -Ialpha * pll->sin_wt_prev + Ibeta  * pll->cos_wt_prev;

    sys->voltage_raw_dq.d =  Valpha * pll->cos_wt_prev + Vbeta  * pll->sin_wt_prev;
    sys->voltage_raw_dq.q = -Valpha * pll->sin_wt_prev + Vbeta  * pll->cos_wt_prev;

}

/*
 *--------------------------------------------------------------------------------------
 * 函数:  pi_control_process
 * 说明:  PI控制器实现，包含电压外环和d,q轴电流内环。
 * (此部分逻辑来自原始代码二)
 *--------------------------------------------------------------------------------------
 */
static void pi_control_process(void)
{
    ControlSystem_t *sys = &g_control_system;

//     --- 电压外环 ---
//    sys->voltage_ref = 800.0f; // 参考值可以由软启动或外部给定
    sys->pi_voltage.error = sys->voltage_ref - sys->filter_VDC;
    sys->pi_voltage.output += KP_VOLTAGE * (sys->pi_voltage.error - sys->pi_voltage.error_prev) + KI_VOLTAGE * sys->pi_voltage.error;
    sys->pi_voltage.error_prev = sys->pi_voltage.error;
    sys->pi_voltage.output = saturate(sys->pi_voltage.output, -VOLTAGE_LIMIT_MIN, VOLTAGE_LIMIT_MAX);
    sys->current_d_ref = sys->pi_voltage.output; // 电压环的输出作为d轴电流的参考值

//    sys->current_d_ref = 2.143f; // 调试：可暂时设为定值

    // --- d轴电流内环 ---
    sys->pi_current_d.error = sys->current_d_ref - sys->current_dq.d;
    sys->pi_current_d.output += KP_CURRENT_D * (sys->pi_current_d.error - sys->pi_current_d.error_prev) + KI_CURRENT_D * sys->pi_current_d.error;
    sys->pi_current_d.error_prev = sys->pi_current_d.error;
    sys->pi_current_d.output = saturate(sys->pi_current_d.output, -CURRENT_D_LIMIT, CURRENT_D_LIMIT);

    // --- q轴电流内环 ---
    sys->current_q_ref = 0.0f; // q轴电流参考通常为0，以实现单位功率因数
    sys->pi_current_q.error = sys->current_q_ref - sys->current_dq.q;
    sys->pi_current_q.output += KP_CURRENT_Q * (sys->pi_current_q.error - sys->pi_current_q.error_prev) + KI_CURRENT_Q * sys->pi_current_q.error;
    sys->pi_current_q.error_prev = sys->pi_current_q.error;
    sys->pi_current_q.output = saturate(sys->pi_current_q.output, -CURRENT_Q_LIMIT, CURRENT_Q_LIMIT);

    // --- 计算指令电压 (包含前馈解耦) ---
    sys->voltage_cmd.d = sys->voltage_raw_dq.d - sys->pi_current_d.output; // 电压前馈 - PI输出
    if(sys->voltage_cmd.d < 0)  sys->voltage_cmd.d = 0.1;
    sys->voltage_cmd.q = sys->voltage_raw_dq.q - sys->pi_current_q.output; // 电压前馈 - PI输出
    sys->voltage_cmd.q = saturate(sys->voltage_cmd.q,-0.1 * sys->voltage_cmd.d,0.1 * sys->voltage_cmd.d);

}


/*
 *--------------------------------------------------------------------------------------
 * 函数:  inverse_park_transform, svpwm_process, update_pwm_compare
 * 说明:  这些函数是标准矢量控制的后续部分，逻辑与原始代码一致，
 * 将PI控制器输出的指令电压转换为最终的PWM波。
 *--------------------------------------------------------------------------------------
 */
static void inverse_park_transform(void)
{
    ControlSystem_t *sys = &g_control_system;
    PLL_t *pll = &sys->pll;
    sys->voltage_ab_cmd.alpha = pll->cos_wt * sys->voltage_cmd.d - pll->sin_wt * sys->voltage_cmd.q;
    sys->voltage_ab_cmd.beta  = pll->sin_wt * sys->voltage_cmd.d + pll->cos_wt * sys->voltage_cmd.q;
}

static void svpwm_process(void)
{
    ControlSystem_t *sys = &g_control_system;
    SVPWM_t *svpwm = &sys->svpwm;
//    PLL_t *pll = &sys->pll;

    // 计算参考电压幅值Vm
    float vm_amplitude = sqrt(sys->voltage_cmd.d * sys->voltage_cmd.d +
                             sys->voltage_cmd.q * sys->voltage_cmd.q);

    // 计算相位角
    float phase_angle = atan2(sys->voltage_ab_cmd.beta, sys->voltage_ab_cmd.alpha);

//    pll->theta = 2 * PI * 11/12;

    // 或者直接使用PLL的相位信息
//     float phase_angle = pll->theta;

    // 归一化幅值到直流母线电压
    float vm_normalized = vm_amplitude / OUTPUT_VOLTAGE_REF;  // OUTPUT_VOLTAGE_REF为直流母线电压
//    float vm_normalized = 0.5;

    // 限制调制度到线性调制范围 (0 ~ 1.0)
    vm_normalized = saturate(vm_normalized, 0.0, 1.0);

    // 计算三相参考电压瞬时值
    float va_ref = vm_normalized * cos(phase_angle);
    float vb_ref = vm_normalized * cos(phase_angle - 2.0944);  // 2π/3 = 2.0944
    float vc_ref = vm_normalized * cos(phase_angle + 2.0944);  // -2π/3 = +4π/3

    // 最大最小值算法
    float v_max = va_ref;
    float v_min = va_ref;

    if (vb_ref > v_max) v_max = vb_ref;
    if (vc_ref > v_max) v_max = vc_ref;

    if (vb_ref < v_min) v_min = vb_ref;
    if (vc_ref < v_min) v_min = vc_ref;

    // 计算零序偏移量
    float v_offset = -(v_max + v_min) * 0.5;

    // 添加零序偏移量得到调制波
    float va_mod = va_ref + v_offset;
    float vb_mod = vb_ref + v_offset;
    float vc_mod = vc_ref + v_offset;

    // 转换为占空比 (0~1)
    float duty_a = va_mod + 0.5;
    float duty_b = vb_mod + 0.5;
    float duty_c = vc_mod + 0.5;

    // 占空比限幅
    duty_a = saturate(duty_a, 0.0, 1.0);
    duty_b = saturate(duty_b, 0.0, 1.0);
    duty_c = saturate(duty_c, 0.0, 1.0);

    // 存储占空比
    svpwm->duty_a = duty_a;
    svpwm->duty_b = duty_b;
    svpwm->duty_c = duty_c;

//    floating_pin_protection();

    // 转换为计数值（150MHz时钟）
    svpwm->tcm1 = (Uint32)duty_to_pwm_register(svpwm->duty_a);
    svpwm->tcm2 = (Uint32)duty_to_pwm_register(svpwm->duty_b);
    svpwm->tcm3 = (Uint32)duty_to_pwm_register(svpwm->duty_c);
//
//    Uint32 result = duty_to_pwm_register(1);

    // 存储调制信息
    svpwm->vm_amplitude = vm_amplitude;
    svpwm->modulation_index = vm_normalized;
    svpwm->phase_angle = phase_angle;
}

static void update_pwm_compare(void)
{
    ControlSystem_t *sys = &g_control_system;
    SVPWM_t *svpwm = &sys->svpwm;

    // 更新ePWM2的比较值 (A相)
    EPwm2Regs.CMPA.half.CMPA = (Uint16)svpwm->tcm1;
    EPwm2Regs.CMPB           = (Uint16)svpwm->tcm1;

    // 更新ePWM3的比较值 (B相)
    EPwm3Regs.CMPA.half.CMPA = (Uint16)svpwm->tcm2;
    EPwm3Regs.CMPB           = (Uint16)svpwm->tcm2;

    // 更新ePWM4的比较值 (C相)
    EPwm4Regs.CMPA.half.CMPA = (Uint16)svpwm->tcm3;
    EPwm4Regs.CMPB           = (Uint16)svpwm->tcm3;
}

static void data_logging(void)
{
   ControlSystem_t *sys = &g_control_system;
    DataLogger_t *logger = &g_data_logger;

    // 记录数据到循环缓冲区
    logger->cos_wt[logger->index] = sys->pll.cos_wt;
    logger->current_a[logger->index] = sys->current_filtered.phase_a;
    logger->voltage_a[logger->index] = sys->voltage_filtered.phase_a;
//    logger->current_d[logger->index] = sys->current_dq.d;
    logger->voltage_positive_q[logger->index]       = sys->filter_vpq.filtered;
    logger->current_dq_q[logger->index] = sys->current_dq.d;
    logger->filter_VDC[logger->index] = sys->filter_VDC;
    // 更新索引（循环缓冲）
    logger->index++;
    if (logger->index >= DATA_BUFFER_SIZE) {
        logger->index = 0;
    }
}

/*
 *--------------------------------------------------------------------------------------
 * 函数:  control_system_init
 * 说明:  初始化所有控制相关的变量和滤波器状态。
 *--------------------------------------------------------------------------------------
 */
void control_system_init(void)
{
    ControlSystem_t *sys = &g_control_system;
    memset(sys, 0, sizeof(ControlSystem_t));

    // 初始化带通滤波器系数
    init_bandpass_filter_coeffs(&sys->bp_coeffs);

    // --- 初始化软启动参数 ---
    sys->voltage_ref_final = 800.0f; // 设定最终目标为800V
    sys->voltage_ref_start = 400.0f; // 设定启动电压为540V (根据预充电情况调整)
    sys->voltage_ref = sys->voltage_ref_start; // 让当前目标电压从起始电压开始
    sys->soft_start_complete = 0; // 启动时，软启动未完成

    // 计算电压增量。决定了软启动的快慢。
    // 假设开关频率为20kHz (T_s = 50us), 期望软启动时间为 2 秒
    // 总步数 = 2s / 50us = 40000步
    // 总电压增量 = 800V - 540V = 260V
    // 每步增量 = 260V / 40000 = 0.0065f
//    sys->voltage_ramp_increment = 0.0065f;

    // 如果不希望计算，可以先给一个非常小的值，例如 0.001f，然后慢慢调整
     sys->voltage_ramp_increment = 0.001f;

    // 初始化PI控制器参数
    sys->pi_voltage.kp = KP_VOLTAGE;
    sys->pi_voltage.ki = KI_VOLTAGE;

    sys->pi_current_d.kp = KP_CURRENT_D;
    sys->pi_current_d.ki = KI_CURRENT_D;

    sys->pi_current_q.kp = KP_CURRENT_Q;
    sys->pi_current_q.ki = KI_CURRENT_Q;

    // 初始化锁相环
    sys->pll.omega = 2.0f * PI * 50.0f; // 初始频率设为50Hz
    sys->pll.theta = 0.0f;
}

/*
 *--------------------------------------------------------------------------------------
 * 函数:  init_bandpass_filter_coeffs, filter_bandpass
 * 说明:  从代码一移植而来的带通滤波器核心函数。
 *--------------------------------------------------------------------------------------
 */
static void init_bandpass_filter_coeffs(FilterBandpassData_t *filter_data)
{
    filter_data->a1 = -1.99276f;
    filter_data->a2 = 0.99374f;
    filter_data->scaled = 0.00313176422918917f;
}

static float filter_bandpass(float x_new, FilterBandpassState_t *filter, const FilterBandpassData_t *filter_data)
{
    // 对输入进行缩放
    filter->x_new_scaled = x_new;
    filter->x_new = filter->x_new_scaled * filter_data->scaled;

    // 执行IIR差分方程: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    // 隐式地定义了 b0=1, b1=0, b2=-1
    filter->y_new = filter->x_new - filter->x_pre2 - (filter_data->a1 * filter->y_pre1) - (filter_data->a2 * filter->y_pre2);

    // 更新历史状态值，为下一次计算做准备
    filter->y_pre2 = filter->y_pre1;
    filter->y_pre1 = filter->y_new;

    filter->x_pre2 = filter->x_pre1;
    filter->x_pre1 = filter->x_new;

    return filter->y_new;
}

/*
 *--------------------------------------------------------------------------------------
 * 函数:  soft_start_process
 * 说明:  执行电压斜坡软启动逻辑。在每次中断时被调用。
 *--------------------------------------------------------------------------------------
 */
static void soft_start_process(void)
{
    ControlSystem_t *sys = &g_control_system;

    // 如果软启动尚未完成
    if (!sys->soft_start_complete)
    {
        // 增加当前的目标电压
        sys->voltage_ref += sys->voltage_ramp_increment;

        // 判断是否已达到或超过最终目标电压
        if (sys->voltage_ref >= sys->voltage_ref_final)
        {
            // 如果是，则将目标电压固定在最终值，并标记软启动完成
            sys->voltage_ref = sys->voltage_ref_final;
            sys->soft_start_complete = 1;
        }
    }
    // 如果软启动已完成，则无需任何操作，sys->voltage_ref 将保持在最终值
}

/* ========== 转换函数 ========== */
static Uint32 duty_to_pwm_register(float duty_ratio)
{
    // 1. 限制占空比范围
    if (duty_ratio < 0.0f) duty_ratio = 0.0f;
    if (duty_ratio > 1.0f) duty_ratio = 1.0f;

    // 2. 计算PWM比较值
    Uint32 compare_value = (Uint32)(duty_ratio * PWM_TBPRD_COUNT);

    // 3. 确保不超过周期值
    if (compare_value > PWM_TBPRD_COUNT) {
        compare_value = PWM_TBPRD_COUNT;
    }

    return compare_value;
}
