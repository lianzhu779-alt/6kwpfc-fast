/*
 * =====================================================================================
 *
 * �ļ�:  adc_user.c
 *
 * ˵��:  �����ѹԴ�����ADC��������ƺ���ʵ���ļ���
 * ���ļ������ع����Խ����������Ż��źŴ������̡�
 * �����˴���һ�Ķ��״�ͨ�˲����������ԭ�еĵ�ͨ�˲�������
 * �Ӷ�����ȷ����ȡ�����źţ�������໷�Ϳ���ϵͳ�����ܡ�
 *
 * ������:  2025��6��22��
 * ����:  galaxy kono
 * �޸���:  Gemini AI
 *
 * =====================================================================================
 */

#include "adc_user.h"
#include "filter_user.h" // ȷ��������filter_1_x��filter_2_x�Ķ���

/* ========== ȫ�ֱ������� ========== */
// ����ȫ�ֵĿ���ϵͳ�ṹ��ʵ�������ڴ洢���п�����صı���
ControlSystem_t g_control_system = {0};
// ����ȫ�ֵ����ݼ�¼�ṹ��ʵ�������ڵ��ԺͲ��ι۲�
DataLogger_t g_data_logger = {0};

/* ========== ��̬�������ڲ����������� ========== */
// Ӳ�������ź�Ԥ����
static void adc_read_channels(void);            // ��ADC�Ĵ�����ȡ��ͨ����ԭʼֵ
static void apply_dc_filters(void);             // ��ֱ���źţ���1.65Vƫ�ã����е�ͨ�˲�
static void apply_offset_and_gain(void);        // ��ȥֱ��ƫ�ò�Ӧ�����棬ת��Ϊ��ʵ����λ
static void apply_bandpass_filters(void);       // �Խ����źŽ��д�ͨ�˲�����ȡ����

// �����㷨����
static void pll_process(void);                  // ���໷��PLL����������ͬ��������λ��Ƶ��
static void clarke_transform(void);             // Clarke�任 (abc -> ����)
static void park_transform(void);               // Park�任 (���� -> dq)
static void pi_control_process(void);           // ˫�ջ�PI������������ѹ�⻷�������ڻ���
static void inverse_park_transform(void);       // ��Park�任 (dq -> ����)
static void svpwm_process(void);                // SVPWM�����㷨������PWMռ�ձ�
static void update_pwm_compare(void);           // ���������ռ�ձȸ��µ�PWMӲ���Ĵ���

// �������ʼ��
static void data_logging(void);                 // ���ݼ�¼�����ڵ���
static void init_bandpass_filter_coeffs(FilterBandpassData_t *filter_data); // ��ʼ����ͨ�˲���ϵ��
static float filter_bandpass(float x_new, FilterBandpassState_t *filter, const FilterBandpassData_t *filter_data); // ��ͨ�˲���ʵ��
static void floating_pin_protection(void);
static Uint32 duty_to_pwm_register(float duty_ratio);

/*
 *--------------------------------------------------------------------------------------
 * ����:  adc_config
 * ˵��:  ����ADCģ�顣�˺�����Ӳ��������أ�����ʵ�ʵ�·��������ADCͨ���Ͳ���ģʽ��
 *--------------------------------------------------------------------------------------
 */
void adc_config(void)
{
    EALLOW; // ��������ܱ����ļĴ���

    // ����ADCʱ��: ADCCLK = HSPCLK / 1 = 150MHz / 1 = 150MHz
    // ע��: ����DSP�ͺź�ϵͳʱ�ӣ�ADCCLKͨ����Ҫ��Ƶ������ADC���Ҫ����<=25MHz��
    // �������SysCtrlRegs.HISPCP.all���ڱ���ȷ���ã�����Ϊ3����ADCCLK=25MHz
    // SysCtrlRegs.HISPCP.all = 3;
    SysCtrlRegs.HISPCP.all= 3;                  //HSPCLK 3��Ƶ 150M/3=50M

    AdcRegs.ADCTRL1.bit.ACQ_PS=5;               //����ʱ��
    AdcRegs.ADCTRL1.bit.CPS=0;                  //����ʱ�Ӳ���Ƶ
    AdcRegs.ADCTRL3.bit.ADCCLKPS=0;             //����Ƶ ADCLK=HSPCLK/2=25MHz

    AdcRegs.ADCTRL1.bit.CONT_RUN=0;//��ȡ��ת�����к�ֹͣ
    AdcRegs.ADCTRL3.bit.SMODE_SEL=0;//˳�����ģʽ
    AdcRegs.ADCTRL1.bit.SEQ_CASC=1;//����������ģʽ

    AdcRegs.ADCMAXCONV.bit.MAX_CONV1=12;//�������Ҫ���õ��Ĳ���ͨ����һ��

    // --- ����ADC����ͨ�� (����ʵ��Ӳ������) ---
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0B; // ͨ��0����ADCINB3 -> VO (ֱ�������ѹ)
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x0A; // ͨ��1����ADCINB3 -> IO (ֱ�������ѹ)
    AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0xC;  //����ͨ��2��Ϊ��3���任--1.65V����ѹ1.65
    AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0xC;  //����ͨ��3��Ϊ��4���任--1.65A������1.65

    AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0x00; // ͨ��6����ADCINA0 -> VA (A�ཻ����ѹ)
    AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 0x08; // ͨ��7����ADCINB0 -> VB (B�ཻ����ѹ)
    AdcRegs.ADCCHSELSEQ3.bit.CONV08 = 0x09; // ͨ��8����ADCINB1 -> VC (C�ཻ����ѹ)

    AdcRegs.ADCCHSELSEQ3.bit.CONV09 = 0x03; // ͨ��09����ADCINA9 -> IA (A�ཻ������)
    AdcRegs.ADCCHSELSEQ3.bit.CONV10 = 0x0F; // ͨ��10����ADCINB10 -> IB (B�ཻ������)
    AdcRegs.ADCCHSELSEQ3.bit.CONV11 = 0x0E; // ͨ��11����ADCINB11 -> IC (C�ཻ������)

    EDIS; // ��ֹ�����ܱ����ļĴ�������ADCINB4 -> 1.65V (ƫ�õ�ѹ)
}

/*
 *--------------------------------------------------------------------------------------
 * ����:  adc_isr
 * ˵��:  ADC�жϷ������������������ϵͳ�ĺ��ģ��Թ̶��Ŀ���Ƶ�ʱ����á�
 * �����������м���Ϳ��������ִ��˳��
 *--------------------------------------------------------------------------------------
 */
interrupt void adc_isr(void)
{
    // ���� 1: ��ȡ�������ADCͨ����ԭʼת�����
    adc_read_channels();

    // ���� 2: ����Ҫ������Ӧ��ֱ���źŽ��е�ͨ�˲����Ի���ȶ���ֵ
    // ����1.65Vƫ�õ�ѹ����������Ӧ���Ǻ㶨��
    apply_dc_filters();

    // ���� 3: �����˲����ƫ�õ�ѹ����ԭʼAC����ֵ����ȥƫ�ô�����Ӧ������ϵ��
    // �����0-3V�Ĳ�����ѹת��Ϊ��ʵ�ĵ�ѹ(V)�͵���(A)ֵ
    apply_offset_and_gain();

    // ���� 4: ����ת��Ϊ����λ�Ľ����ź�Ӧ�ô�ͨ�˲���
    // ���ǹؼ�һ����Ŀ���Ǿ�ȷ��ȡ50Hz�Ļ����������˳���Ƶ������ֱ������
    apply_bandpass_filters();

    // ���� 5: ִ�����໷�㷨��ʹ���˲���ĵ�ѹ�ź�����ȷ���ٵ�������λ��Ƶ��
    pll_process();

    // ���� 6: ִ��Clarke�任�������ྲֹ����ϵ(abc)�µĵ�����ת�������ྲֹ����ϵ(����)
    clarke_transform();

    // ���� 7: ִ��Park�任���������໷�ṩ�ĽǶȣ������ྲֹ(����)�µĵ�ѹ�͵�����
    // ת��������ͬ����ת����ϵ(dq)
    park_transform();

    // ���� 8: ִ��PI����������dq����ϵ��ʵ�ֵ�ѹ�͵�����˫�ջ�����
    pi_control_process();

    // ���� 9: ִ����Park�任����PI�����������dqָ���ѹת�������ྲֹ����ϵ(����)
    inverse_park_transform();

    // ���� 10: ִ��SVPWM�㷨�����ݦ���ָ���ѹ���������PWM�Ŀ���ʱ���ռ�ձ�
    svpwm_process();

    // ���� 11: ���������SVPWM�Ƚ�ֵд��ePWMģ���Ӳ���Ĵ���
    update_pwm_compare();

    // ���� 12: (������) ���ؼ��������뻺����������ͨ��CCS��ʾ�����۲첨��
    data_logging();

    // --- �жϽ���ǰ�������� ---
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;      // ��λADC���з�����
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;    // ���ADC����1���жϱ�־λ
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // ��PIE������Ӧ���жϣ�������Ӧͬ��������ж�
}

/*
 *--------------------------------------------------------------------------------------
 * ����:  adc_read_channels
 * ˵��:  ��ADC����Ĵ����ж�ȡ���ݣ���ת��Ϊ0-3.0V��Χ�ڵĸ����ѹֵ��
 *--------------------------------------------------------------------------------------
 */
static void adc_read_channels(void)
{
    ControlSystem_t *sys = &g_control_system;

    // ����4λ��Ϊ�˽�12λADC������롣ADCRESULT�Ĵ�����16λ�ģ����������š�

    // ���� adc_config �е� CONV00 ���ã���ȡֱ�������ѹ
    sys->adc_raw.output_voltage      = (AdcRegs.ADCRESULT0 >> 4) * ADC_TO_VOLTAGE;

    // ���� adc_config �е� CONV01 ���ã���ȡֱ���������
    sys->adc_raw.output_current      = (AdcRegs.ADCRESULT1 >> 4) * ADC_TO_VOLTAGE; // ����ṹ�����д˳�Ա

    // ���� adc_config �е� CONV02 �� CONV03 ���ã���ȡƫ�õ�ѹ
    // �����Ը�����Ҫѡ��洢һ��������ֵ
    sys->adc_raw.ref_voltage_current_165 = (AdcRegs.ADCRESULT2 >> 4) * ADC_TO_VOLTAGE; // ����Ϊ��������ƫ��
    sys->adc_raw.ref_voltage_voltage_165 = (AdcRegs.ADCRESULT3 >> 4) * ADC_TO_VOLTAGE; // ����Ϊ��ѹ����ƫ��

    // ���� adc_config �е� CONV06 ���ã���ȡA�ཻ����ѹ
    sys->adc_raw.ac_voltage_a        = (AdcRegs.ADCRESULT6 >> 4) * ADC_TO_VOLTAGE;

    // ���� adc_config �е� CONV07 ���ã���ȡB�ཻ����ѹ
    sys->adc_raw.ac_voltage_b        = (AdcRegs.ADCRESULT7 >> 4) * ADC_TO_VOLTAGE;

    // ���� adc_config �е� CONV08 ���ã���ȡC�ཻ����ѹ
    sys->adc_raw.ac_voltage_c        = (AdcRegs.ADCRESULT8 >> 4) * ADC_TO_VOLTAGE;

    // ���� adc_config �е� CONV09 ���ã���ȡA�ཻ������
    sys->adc_raw.ac_current_a        = (AdcRegs.ADCRESULT9 >> 4) * ADC_TO_VOLTAGE;

    // ���� adc_config �е� CONV10 ���ã���ȡB�ཻ������
    sys->adc_raw.ac_current_b        = (AdcRegs.ADCRESULT10 >> 4) * ADC_TO_VOLTAGE;

    // ���� adc_config �е� CONV11 ���ã���ȡC�ཻ������
    sys->adc_raw.ac_current_c        = (AdcRegs.ADCRESULT11 >> 4) * ADC_TO_VOLTAGE;
}

/*
 *--------------------------------------------------------------------------------------
 * ����:  apply_dc_filters
 * ˵��:  Ӧ��һ�׵�ͨ�˲�����ƽ��ֱ���źš�
 *--------------------------------------------------------------------------------------
 */
static void apply_dc_filters(void)
{
    ControlSystem_t *sys = &g_control_system;
    const float32 filter_coeff = 0.001f; // ͳһ�������˲�ϵ������ AdcVIHan() ����һ��

    // ��ֱ��ĸ�ߵ�ѹ (Vo) ���е�ͨ�˲�
    sys->filter_vo.current = sys->adc_raw.output_voltage;
    // ʹ�� AdcVIHan �е��˲�ϵ�� 0.001f
    sys->filter_vo.filtered = filter_2_x(sys->filter_vo.current, sys->filter_vo.previous, filter_coeff);
    sys->filter_vo.previous = sys->filter_vo.filtered;
    sys->filter_VDC = sys->filter_vo.filtered * OUTPUT_VOLTAGE_REF/2;

    // ��������ֱ��������� (Io) ���е�ͨ�˲�
    sys->filter_io.current = sys->adc_raw.output_current;
    sys->filter_io.filtered = filter_2_x(sys->filter_io.current, sys->filter_io.previous, filter_coeff);
    sys->filter_io.previous = sys->filter_io.filtered;

    // �Ե�������1.65Vƫ�õ�ѹ���е�ͨ�˲�
    sys->filter_v165_current.current = sys->adc_raw.ref_voltage_current_165;
    sys->filter_v165_current.filtered = filter_2_x(sys->filter_v165_current.current, sys->filter_v165_current.previous, filter_coeff);
    sys->filter_v165_current.previous = sys->filter_v165_current.filtered;

    // �Ե�ѹ����1.65Vƫ�õ�ѹ���е�ͨ�˲�
    sys->filter_v165_voltage.current = sys->adc_raw.ref_voltage_voltage_165;
    sys->filter_v165_voltage.filtered = filter_2_x(sys->filter_v165_voltage.current, sys->filter_v165_voltage.previous, filter_coeff);
    sys->filter_v165_voltage.previous = sys->filter_v165_voltage.filtered;


}

/*
 *--------------------------------------------------------------------------------------
 * ����:  apply_offset_and_gain
 * ˵��:  ����ƫ��У��������ת�����õ���ʵ������ֵ��
 *--------------------------------------------------------------------------------------
 */


static void apply_offset_and_gain(void)
{
    ControlSystem_t *sys = &g_control_system;
    float v165_filtered = sys->filter_v165_voltage.filtered;
    float A165_filtered = sys->filter_v165_current.filtered;

    // 1. ��AC�źŵĲ���ֵ�м�ȥ�˲���õ����ȶ�ֱ��ƫ��
    float va_offset = sys->adc_raw.ac_voltage_a - v165_filtered;
    float vb_offset = sys->adc_raw.ac_voltage_b - v165_filtered;
    float vc_offset = sys->adc_raw.ac_voltage_c - v165_filtered;

    float ia_offset = sys->adc_raw.ac_current_a - A165_filtered;
    float ib_offset = sys->adc_raw.ac_current_b - A165_filtered;
    float ic_offset = sys->adc_raw.ac_current_c - A165_filtered;

    // 2. Ӧ��Ӳ����·������ϵ������У����ĵ�ѹ�ź�ת��Ϊ��ʵ������λ
    sys->voltage.phase_a = va_offset * 487.8f;
    sys->voltage.phase_b = vb_offset * 487.8f;
    sys->voltage.phase_c = vc_offset * 487.8f;

    // ���ݴ���һ���߼��������ź���Ҫȡ������ͨ��������������ķ����ӷ��йء�
    sys->current.phase_a = -ia_offset * 40.0f;
    sys->current.phase_b = -ib_offset * 40.0f;
    sys->current.phase_c = -ic_offset * 40.0f;
}

/*
 *--------------------------------------------------------------------------------------
 * ����:  apply_bandpass_filters
 * ˵��:  �����ཻ����ѹ�͵���Ӧ�ô�ͨ�˲�����
 *--------------------------------------------------------------------------------------
 */
int test= 0;
#define TEST 200
float va[TEST] = {0};
static void apply_bandpass_filters(void)
{
    ControlSystem_t *sys = &g_control_system;
    // ����������ת���������ֵ�����ͨ�˲���
    sys->voltage_filtered.phase_a = filter_bandpass(sys->voltage.phase_a, &sys->bp_va, &sys->bp_coeffs);
    sys->voltage_filtered.phase_b = filter_bandpass(sys->voltage.phase_b, &sys->bp_vb, &sys->bp_coeffs);
    sys->voltage_filtered.phase_c = filter_bandpass(sys->voltage.phase_c, &sys->bp_vc, &sys->bp_coeffs);

    sys->current_filtered.phase_a = filter_bandpass(sys->current.phase_a, &sys->bp_ia, &sys->bp_coeffs);
    sys->current_filtered.phase_b = filter_bandpass(sys->current.phase_b, &sys->bp_ib, &sys->bp_coeffs);
    sys->current_filtered.phase_c = filter_bandpass(sys->current.phase_c, &sys->bp_ic, &sys->bp_coeffs);

    va[test] = sys->voltage_filtered.phase_a;
    test++;
    test%=TEST;
}


/**
 * @brief ˫ͬ������ϵ���໷(DDSOGI-PLL)���Ĵ�����
 * @details
 * �������ϸ���ղ������� LockPhase() �����ļ����߼�����ʵ�ֹ����ϵ�1:1ӳ�䡣
 * ��ʹ���� adc_user.h �ж����ȫ�ֽṹ�� g_control_system �������м����״̬���¡�
 * ����: Clarke�任 -> ��������� -> ������� -> ��ͨ�˲� -> PI���� -> Ƶ��/��λ����
 */
static void pll_process(void)
{
    // ��ȫ�ֽṹ���л�ȡ����������ӽṹ��ָ�룬�����д
    ControlSystem_t *sys = &g_control_system;
    PLL_t *pll = &sys->pll;

    /* ================================================================================= */
    /* ���� 1: Clarke �任 (abc -> ����)                                                 */
    /* ע�⣺ԭʼ��������� Uo (OUTPUT_VOLTAGE_REF) ���й�һ����                       */
    /* ================================================================================= */
    // ���˲���������ѹ���б任
    float va = sys->voltage_filtered.phase_a;
    float vb = sys->voltage_filtered.phase_b;
    float vc = sys->voltage_filtered.phase_c;

    // �����һ����� Alpha-Beta ����
    sys->voltage_ab.alpha = TWO_DIV_3 * (va / OUTPUT_VOLTAGE_REF - 0.5f * vb / OUTPUT_VOLTAGE_REF - 0.5f * vc / OUTPUT_VOLTAGE_REF);
    sys->voltage_ab.beta  = SQRT_3_DIV_3 * (vb / OUTPUT_VOLTAGE_REF - vc / OUTPUT_VOLTAGE_REF);


    /* ================================================================================= */
    /* ���� 2: ����ԭʼ������͸���d-q���� (VPosD, VPosQ, VNegD, VNegQ)              */
    /* ================================================================================= */
    float v_alpha = sys->voltage_ab.alpha;
    float v_beta  = sys->voltage_ab.beta;
    float cos_wt  = pll->cos_wt;
    float sin_wt  = pll->sin_wt;

    float vpd_raw =  v_alpha * cos_wt + v_beta * sin_wt;
    float vpq_raw = -v_alpha * sin_wt + v_beta * cos_wt;
    float vnd_raw =  v_alpha * cos_wt - v_beta * sin_wt;
    float vnq_raw =  v_alpha * sin_wt + v_beta * cos_wt;


    /* ================================================================================= */
    /* ���� 3: �����򽻲�������ͨ�˲� (Decoupling)                                  */
    /* ================================================================================= */
    float sin_2wt = sinf(2.0f * pll->theta);
    float cos_2wt = cosf(2.0f * pll->theta);

    // ��������d���ѹ (Vpd)
    float vpd_decoupled = vpd_raw - (cos_2wt * sys->filter_vnd.filtered + sin_2wt * sys->filter_vnq.filtered);
    sys->filter_vpd.filtered = filter_1_x(vpd_decoupled, sys->filter_vpd.previous, 0.062832f);
    sys->filter_vpd.previous = sys->filter_vpd.filtered;

    // ��������q���ѹ (Vpq)
    // ע�⣺ԭʼ�����д˴�Ϊ Pll.VPosQ - (sin*Vnd - cos*Vnq)��������ѭ���߼�
    float vpq_decoupled = vpq_raw - (-sin_2wt * sys->filter_vnd.filtered + cos_2wt * sys->filter_vnq.filtered);
    sys->filter_vpq.filtered = filter_1_x(vpq_decoupled, sys->filter_vpq.previous, 0.062832f);
    sys->filter_vpq.previous = sys->filter_vpq.filtered;

    // �����d���ѹ (Vnd), ��Ӧ LockPhase �е� Pll.DVND �� Pll.FDVND
    float vnd_decoupled = vnd_raw - (cos_2wt * sys->filter_vpd.filtered - sin_2wt * sys->filter_vpq.filtered);
    sys->filter_vnd.filtered = filter_1_x(vnd_decoupled, sys->filter_vnd.previous, 0.062832f);
    sys->filter_vnd.previous = sys->filter_vnd.filtered;

    // �����q���ѹ (Vnq), ��Ӧ LockPhase �е� Pll.DVNQ �� Pll.FDVNQ
    float vnq_decoupled = vnq_raw - (sin_2wt * sys->filter_vpd.filtered + cos_2wt * sys->filter_vpq.filtered);
    sys->filter_vnq.filtered = filter_1_x(vnq_decoupled, sys->filter_vnq.previous, 0.062832f);
    sys->filter_vnq.previous = sys->filter_vnq.filtered;


    /* ================================================================================= */
    /* ���� 4: PI ������ (��ȫ���� LockPhase ���߼�)                                      */
    /* ע��: Ϊ��ȷ����, PLL_t �ṹ������Ҫ����״̬��������Ӧ Pll.DVPQ_preD2           */
    /* ���ǽ� pll->pi_integrator_state ���� Pll.DVPQ_preD2                           */
    /* pll->error_prev ���� Pll.DVPQ_preD                                            */
    /* pll->pi_output ���� Pll.DVPQ_preD1                                            */
    /* ================================================================================= */
    // 4.1 ����PIϵ�� (�� LockPhase �е�Ӳ������ֵ��ȫһ��)
    //PI_COEFF_A��KP+KI��PI_COEFF_B��KP
    const float PI_COEFF_A = 166.9743385f;
    const float PI_COEFF_B = 166.2661165f;

    // 4.2 ��ȡ����ź� (������Vpq) �����������޷�
    float vq_error = sys->filter_vpq.filtered; // ��Ӧ Pll.FDVPQ
    vq_error = saturate(vq_error, -0.1f, 0.1f);

    // 4.3 ִ��PI���㣬��ȫ��Ч�� Pll.DVPQ_preD1 = Pll.DVPQ_preD2 + A*err(k) - B*err(k-1)
    pll->pi_output = pll->pi_integrator_state + PI_COEFF_A * vq_error - PI_COEFF_B * pll->error_prev;

    // 4.4 ����������һ�μ�������ֵ
    pll->error_prev = vq_error; // ��Ӧ Pll.DVPQ_preD = Pll.FDVPQ;

    // 4.5 PI����޷� (������), ��Ӧ +/- 5Hz
    const float FREQ_DEV_LIMIT = 5.0f * 2.0f * PI;
    pll->pi_output = saturate(pll->pi_output, -FREQ_DEV_LIMIT, FREQ_DEV_LIMIT);

    // 4.6 ���»�����״ֵ̬�������´��ۼ�
    pll->pi_integrator_state = pll->pi_output; // ��Ӧ Pll.DVPQ_preD2 = Pll.DVPQ_preD1;


    /* ================================================================================= */
    /* ���� 5: ���½�Ƶ�ʺ���λ (��ȫ���� LockPhase ���߼�)                               */
    /* ================================================================================= */
    float freq_deviation = pll->pi_output;         // PI�������Ƶ�ʵ����� ����, ��Ӧ Pll.w_cmp
    pll->omega = (2.0f * PI * 50.0f) + freq_deviation; // �õ����ս�Ƶ��, ��Ӧ Pll.w

    pll->theta += pll->omega * SAMPLING_PERIOD;    // ���ֵõ��µ���λ��, ��Ӧ Pll.wt += Pll.dwt

    // ����λ�ǽ���ȡģ���� LockPhase �߼�����һ��
    // (ע�⣺ԭʼ������С��0ֱ����0���˴���ʵ��ԭ)
    if (pll->theta >= (2.0f * PI)) {
        pll->theta -= (2.0f * PI);
    } else if (pll->theta < 0.0f) {
        pll->theta = 0.0f;
    }


    /* ================================================================================= */
    /* ���� 6: ����������ֵ������һ�ε���ʹ��                                            */
    /* ================================================================================= */
    pll->sin_wt = sinf(pll->theta);
    pll->cos_wt = cosf(pll->theta);


    /* ================================================================================= */
    /* ���� 7: �����ս�����d-q��ѹ�������ṹ�壬���������ƻ�·ʹ��                     */
    /* ================================================================================= */
    sys->voltage_dq.d = sys->filter_vpd.filtered; // ���յ�����d���ѹ
    sys->voltage_dq.q = sys->filter_vpq.filtered; // ���յ�����q���ѹ (������Ӧ������0)
}
/*
 *--------------------------------------------------------------------------------------
 * ����:  clarke_transform
 * ˵��:  ִ�е�����Clarke�任��
 *--------------------------------------------------------------------------------------
 */
static void clarke_transform(void)
{
    ControlSystem_t *sys = &g_control_system;
    float ia = sys->current_filtered.phase_a;
    float ib = sys->current_filtered.phase_b;
    float ic = sys->current_filtered.phase_c;

    // ��ֵ����Clarke�任
    sys->current_ab.alpha = TWO_DIV_3 * (ia - 0.5f * ib - 0.5f * ic);
    sys->current_ab.beta = SQRT_3_DIV_3 * (ib - ic);
}

/*
 *--------------------------------------------------------------------------------------
 * ����:  park_transform
 * ˵��:  ִ�е�ѹ�͵�����Park�任��
 *--------------------------------------------------------------------------------------
 */
static void park_transform(void)
{
    ControlSystem_t *sys = &g_control_system;
    PLL_t *pll = &sys->pll;

    // ��Clarke�任����л�ȡ����alpha, beta
    float Ialpha = sys->current_ab.alpha;
    float Ibeta = sys->current_ab.beta;

    // ��PLL���м�������л�ȡ��ѹalpha, beta
    float Valpha = sys->voltage_ab.alpha;
    float Vbeta = sys->voltage_ab.beta;

    // ִ�е�����Park�任
    sys->current_dq.d =  Ialpha * pll->cos_wt + Ibeta  * pll->sin_wt;
    sys->current_dq.q = -Ialpha * pll->sin_wt + Ibeta  * pll->cos_wt;

    // ִ�е�ѹ��Park�任
    sys->voltage_dq.d =  Valpha * pll->cos_wt + Vbeta  * pll->sin_wt;
    sys->voltage_dq.q = -Valpha * pll->sin_wt + Vbeta  * pll->cos_wt;
}

/*
 *--------------------------------------------------------------------------------------
 * ����:  pi_control_process
 * ˵��:  PI������ʵ�֣�������ѹ�⻷��d,q������ڻ���
 * (�˲����߼�����ԭʼ�����)
 *--------------------------------------------------------------------------------------
 */
static void pi_control_process(void)
{
    ControlSystem_t *sys = &g_control_system;

//     --- ��ѹ�⻷ ---
//    sys->voltage_ref = 100.0f; // �ο�ֵ���������������ⲿ����
//    sys->pi_voltage.error = sys->voltage_ref - sys->filter_VDC;
//    sys->pi_voltage.output += KP_VOLTAGE * (sys->pi_voltage.error - sys->pi_voltage.error_prev) + KI_VOLTAGE * sys->pi_voltage.error;
//    sys->pi_voltage.error_prev = sys->pi_voltage.error;
//    sys->pi_voltage.output = saturate(sys->pi_voltage.output, -VOLTAGE_LIMIT_MIN, VOLTAGE_LIMIT_MAX);
//    sys->current_d_ref = sys->pi_voltage.output; // ��ѹ���������Ϊd������Ĳο�ֵ

    sys->current_d_ref = 2.143f; // ���ԣ�����ʱ��Ϊ��ֵ

    // --- d������ڻ� ---
    sys->pi_current_d.error = sys->current_d_ref - sys->current_dq.d;
    sys->pi_current_d.output += KP_CURRENT_D * (sys->pi_current_d.error - sys->pi_current_d.error_prev) + KI_CURRENT_D * sys->pi_current_d.error;
    sys->pi_current_d.error_prev = sys->pi_current_d.error;
    sys->pi_current_d.output = saturate(sys->pi_current_d.output, -CURRENT_D_LIMIT, CURRENT_D_LIMIT);

    // --- q������ڻ� ---
    sys->current_q_ref = 0.0f; // q������ο�ͨ��Ϊ0����ʵ�ֵ�λ��������
    sys->pi_current_q.error = sys->current_q_ref - sys->current_dq.q;
    sys->pi_current_q.output += KP_CURRENT_Q * (sys->pi_current_q.error - sys->pi_current_q.error_prev) + KI_CURRENT_Q * sys->pi_current_q.error;
    sys->pi_current_q.error_prev = sys->pi_current_q.error;
    sys->pi_current_q.output = saturate(sys->pi_current_q.output, -CURRENT_Q_LIMIT, CURRENT_Q_LIMIT);

    // --- ����ָ���ѹ (����ǰ������) ---
    sys->voltage_cmd.d = sys->voltage_dq.d - sys->pi_current_d.output; // ��ѹǰ�� - PI���
    if(sys->voltage_cmd.d < 0)  sys->voltage_cmd.d = 0.1;
    sys->voltage_cmd.q = sys->voltage_dq.q - sys->pi_current_q.output; // ��ѹǰ�� - PI���
    sys->voltage_cmd.q = saturate(sys->voltage_cmd.q,-0.1 * sys->voltage_cmd.d,0.1 * sys->voltage_cmd.d);

}


/*
 *--------------------------------------------------------------------------------------
 * ����:  inverse_park_transform, svpwm_process, update_pwm_compare
 * ˵��:  ��Щ�����Ǳ�׼ʸ�����Ƶĺ������֣��߼���ԭʼ����һ�£�
 * ��PI�����������ָ���ѹת��Ϊ���յ�PWM����
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
    PLL_t *pll = &sys->pll;

    // ����ο���ѹ��ֵVm
    float vm_amplitude = sqrt(sys->voltage_cmd.d * sys->voltage_cmd.d +
                             sys->voltage_cmd.q * sys->voltage_cmd.q);

    // ������λ��
//    float phase_angle = atan2(sys->voltage_ab_cmd.beta, sys->voltage_ab_cmd.alpha);

//    pll->theta = 2 * PI * 11/12;

    // ����ֱ��ʹ��PLL����λ��Ϣ
     float phase_angle = pll->theta;

    // ��һ����ֵ��ֱ��ĸ�ߵ�ѹ
    float vm_normalized = vm_amplitude / OUTPUT_VOLTAGE_REF;  // ����OUTPUT_VOLTAGE_REFΪֱ��ĸ�ߵ�ѹ
//    float vm_normalized = 0.5;

    // ���Ƶ��ƶȵ����Ե��Ʒ�Χ (0 ~ 1.0)
    vm_normalized = saturate(vm_normalized, 0.0, 1.0);

    // ��������ο���ѹ˲ʱֵ
    float va_ref = vm_normalized * cos(phase_angle);
    float vb_ref = vm_normalized * cos(phase_angle - 2.0944);  // 2��/3 = 2.0944
    float vc_ref = vm_normalized * cos(phase_angle + 2.0944);  // -2��/3 = +4��/3

    // �����Сֵ�㷨
    float v_max = va_ref;
    float v_min = va_ref;

    if (vb_ref > v_max) v_max = vb_ref;
    if (vc_ref > v_max) v_max = vc_ref;

    if (vb_ref < v_min) v_min = vb_ref;
    if (vc_ref < v_min) v_min = vc_ref;

    // ��������ƫ����
    float v_offset = -(v_max + v_min) * 0.5;

    // �������ƫ�����õ����Ʋ�
    float va_mod = va_ref + v_offset;
    float vb_mod = vb_ref + v_offset;
    float vc_mod = vc_ref + v_offset;

    // ת��Ϊռ�ձ� (0~1)
    float duty_a = va_mod + 0.5;
    float duty_b = vb_mod + 0.5;
    float duty_c = vc_mod + 0.5;

    // ռ�ձ��޷�
    duty_a = saturate(duty_a, 0.0, 1.0);
    duty_b = saturate(duty_b, 0.0, 1.0);
    duty_c = saturate(duty_c, 0.0, 1.0);

    // �洢ռ�ձ�
    svpwm->duty_a = duty_a;
    svpwm->duty_b = duty_b;
    svpwm->duty_c = duty_c;

//    floating_pin_protection();

    // ת��Ϊ����ֵ��150MHzʱ�ӣ�
    svpwm->tcm1 = (Uint32)duty_to_pwm_register(svpwm->duty_a);
    svpwm->tcm2 = (Uint32)duty_to_pwm_register(svpwm->duty_b);
    svpwm->tcm3 = (Uint32)duty_to_pwm_register(svpwm->duty_c);

//    svpwm->tcm1 = 7500/2;
//    svpwm->tcm2 = 7500/2;
//    svpwm->tcm3 = 7500/2;
//
//    Uint32 result = duty_to_pwm_register(1);

    // �洢������Ϣ
    svpwm->vm_amplitude = vm_amplitude;
    svpwm->modulation_index = vm_normalized;
    svpwm->phase_angle = phase_angle;
}

static void update_pwm_compare(void)
{
    ControlSystem_t *sys = &g_control_system;
    SVPWM_t *svpwm = &sys->svpwm;

    // ����ePWM2�ıȽ�ֵ (A��)
    EPwm2Regs.CMPA.half.CMPA = (Uint16)svpwm->tcm1;
    EPwm2Regs.CMPB           = (Uint16)svpwm->tcm1;

    // ����ePWM3�ıȽ�ֵ (B��)
    EPwm3Regs.CMPA.half.CMPA = (Uint16)svpwm->tcm2;
    EPwm3Regs.CMPB           = (Uint16)svpwm->tcm2;

    // ����ePWM4�ıȽ�ֵ (C��)
    EPwm4Regs.CMPA.half.CMPA = (Uint16)svpwm->tcm3;
    EPwm4Regs.CMPB           = (Uint16)svpwm->tcm3;
}

static void data_logging(void)
{
   ControlSystem_t *sys = &g_control_system;
    DataLogger_t *logger = &g_data_logger;

    // ��¼���ݵ�ѭ��������
    logger->cos_wt[logger->index] = sys->pll.cos_wt;
    logger->current_a[logger->index] = sys->current_filtered.phase_a;
    logger->voltage_a[logger->index] = sys->voltage_filtered.phase_a;
//    logger->current_d[logger->index] = sys->current_dq.d;
    logger->voltage_positive_q[logger->index]       = sys->filter_vpq.filtered;

    // ����������ѭ�����壩
    logger->index++;
    if (logger->index >= DATA_BUFFER_SIZE) {
        logger->index = 0;
    }
}

/*
 *--------------------------------------------------------------------------------------
 * ����:  control_system_init
 * ˵��:  ��ʼ�����п�����صı������˲���״̬��
 *--------------------------------------------------------------------------------------
 */
void control_system_init(void)
{
    ControlSystem_t *sys = &g_control_system;
    memset(sys, 0, sizeof(ControlSystem_t));

    // ��ʼ����ͨ�˲���ϵ��
    init_bandpass_filter_coeffs(&sys->bp_coeffs);

    // ��ʼ��PI����������
    sys->pi_voltage.kp = KP_VOLTAGE;
    sys->pi_voltage.ki = KI_VOLTAGE;

    sys->pi_current_d.kp = KP_CURRENT_D;
    sys->pi_current_d.ki = KI_CURRENT_D;

    sys->pi_current_q.kp = KP_CURRENT_Q;
    sys->pi_current_q.ki = KI_CURRENT_Q;

    // ��ʼ�����໷
    sys->pll.omega = 2.0f * PI * 50.0f; // ��ʼƵ����Ϊ50Hz
    sys->pll.theta = 0.0f;
}

/*
 *--------------------------------------------------------------------------------------
 * ����:  init_bandpass_filter_coeffs, filter_bandpass
 * ˵��:  �Ӵ���һ��ֲ�����Ĵ�ͨ�˲������ĺ�����
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
    // �������������
    filter->x_new_scaled = x_new;
    filter->x_new = filter->x_new_scaled * filter_data->scaled;

    // ִ��IIR��ַ���: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    // ����һ�ķ�����ʽ�ض����� b0=1, b1=0, b2=-1
    filter->y_new = filter->x_new - filter->x_pre2 - (filter_data->a1 * filter->y_pre1) - (filter_data->a2 * filter->y_pre2);

    // ������ʷ״ֵ̬��Ϊ��һ�μ�����׼��
    filter->y_pre2 = filter->y_pre1;
    filter->y_pre1 = filter->y_new;

    filter->x_pre2 = filter->x_pre1;
    filter->x_pre1 = filter->x_new;

    return filter->y_new;
}


/* ========== ������ռ�Ᵽ�� ========== */
static void floating_pin_protection(void)
{
    ControlSystem_t *sys = &g_control_system;
    SVPWM_t *svpwm = &sys->svpwm;

    // ���ADC�����Ƿ��쳣
    if (abs(sys->voltage_filtered.phase_a / OUTPUT_VOLTAGE_REF) < 0.1 &&
        abs(sys->voltage_filtered.phase_b / OUTPUT_VOLTAGE_REF) < 0.1 &&
        abs(sys->voltage_filtered.phase_c / OUTPUT_VOLTAGE_REF) < 0.1) {

        // ����������״̬�����뱣��ģʽ
//        sys->protection_mode = FLOATING_PIN_PROTECTION;

        // ǿ��PWM���Ϊ��ȫֵ
        svpwm->duty_a = 0.5;  // 50%ռ�ձ�
        svpwm->duty_b = 0.5;
        svpwm->duty_c = 0.5;

        // ���߹ر�PWM���
        // disable_pwm_output();
    }
}


/* ========== ת������ ========== */
static Uint32 duty_to_pwm_register(float duty_ratio)
{
    // 1. ����ռ�ձȷ�Χ
    if (duty_ratio < 0.0f) duty_ratio = 0.0f;
    if (duty_ratio > 1.0f) duty_ratio = 1.0f;

    // 2. ����PWM�Ƚ�ֵ
    Uint32 compare_value = (Uint32)(duty_ratio * PWM_TBPRD_COUNT);

    // 3. ȷ������������ֵ
    if (compare_value > PWM_TBPRD_COUNT) {
        compare_value = PWM_TBPRD_COUNT;
    }

    return compare_value;
}
