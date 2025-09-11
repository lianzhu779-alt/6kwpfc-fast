/*
 * adc_user.h
 * �����ѹԴ�����ADC���������ͷ�ļ� (������)
 *
 * ������: 2025��6��22��
 * ����: galaxy kono
 * �޸���: Gemini AI
 */

#ifndef APP_ADC_USER_ADC_USER_H_
#define APP_ADC_USER_ADC_USER_H_

#include "DSP28x_Project.h"
#include "math.h"
#include "filter_user.h"
#include <string.h>

/* ========== ��ѧ������������ ========== */
#define PI                  3.14159265358979323846f
#define SQRT_3              1.73205080756887729353f  // ��3
#define SQRT_3_DIV_2        0.86602540378443864676f  // ��3/2
#define SQRT_3_DIV_3        0.57735026918962576451f  // ��3/3
#define TWO_DIV_3           0.66666666666666666667f  // 2/3
#define ADC_TO_VOLTAGE      0.00073260073260073260f  // ADCת��ϵ�� (3.0V / 4095)

/* ========== ϵͳ�������� ========== */
#define SWITCHING_FREQ      10000.0f                 // ����Ƶ�� 10kHz
#define SAMPLING_PERIOD     0.0001f                  // �������� Ts = 1/10kHz
#define PWM_PERIOD_NS       0.000000125f             // PWMʱ������(ns), ����ʵ��PWMʱ�ӵ���
#define OUTPUT_VOLTAGE_REF  800.0f                   // �����ѹ��׼ֵ (����PLL��һ��)

/* ========== PI���������� ========== */
#define KP_VOLTAGE          1.0f
#define KI_VOLTAGE          0.015f
#define KP_CURRENT_D        2.44f
#define KI_CURRENT_D        0.0019f
#define KP_CURRENT_Q        2.44f
#define KI_CURRENT_Q        0.0019f

/* ========== �޷����� ========== */
#define VOLTAGE_LIMIT_MAX   80.0f
#define VOLTAGE_LIMIT_MIN   50.0f
#define CURRENT_D_LIMIT     80.0f
#define CURRENT_Q_LIMIT     80.0f

/* ========== ���ݼ�¼���� ========== */
#define DATA_BUFFER_SIZE    200

/* ========== PWMʱ������ ========== */
#define PWM_SYSCLK_FREQ_HZ          150000000UL    // ϵͳʱ��Ƶ�� 150MHz
#define PWM_SWITCHING_FREQ_HZ       10000UL        // ����Ƶ�� 10kHz
#define PWM_DEAD_TIME_PERCENT       2              // ����ʱ��ٷֱ� 2%

/* ========== PWM���ڼ��� ========== */
// ��������ģʽ�£�ʵ��PWMƵ�� = SYSCLK / (2 * TBPRD)
// ��ˣ�TBPRD = SYSCLK / (2 * Ŀ��Ƶ��)
#define PWM_TBPRD_COUNT            (PWM_SYSCLK_FREQ_HZ / (2 * PWM_SWITCHING_FREQ_HZ))  // 7500
#define PWM_DEAD_TIME_COUNT        (PWM_TBPRD_COUNT / (100 / PWM_DEAD_TIME_PERCENT))   // 150

/* ========== ��ͨ�˲����ṹ�嶨�� (���Դ���һ) ========== */
typedef struct {
    float a1, a2; // ��ַ��̷�ĸϵ�� y[n-1], y[n-2]
    float scaled; // ��������
} FilterBandpassData_t;

typedef struct {
    float x_new;        // ��ǰ����
    float x_pre1;       // ǰһ������
    float x_pre2;       // ǰ��������
    float y_new;        // ��ǰ���
    float y_pre1;       // ǰһ�����
    float y_pre2;       // ǰ�������
    float x_new_scaled; // ���ź�ĵ�ǰ����
} FilterBandpassState_t;

/* ========== ADCԭʼ���ݽṹ�� ========== */
// ������ʵ��ʹ�õ�ͨ��
typedef struct {
    float output_voltage;   // ֱ�������ѹ����ֵ
    float output_current;   // ֱ�������������ֵ
    float ac_voltage_a;     // A�ཻ����ѹ����ֵ
    float ac_voltage_b;     // B�ཻ����ѹ����ֵ
    float ac_voltage_c;     // C�ཻ����ѹ����ֵ
    float ac_current_a;     // A�ཻ����������ֵ
    float ac_current_b;     // B�ཻ����������ֵ
    float ac_current_c;     // C�ཻ����������ֵ
    float ref_voltage_voltage_165; // 1.65Vƫ�õ�ѹ����ֵ
    float ref_voltage_current_165; // 1.65Aƫ�õ�������

} AdcRawData_t;

/* ========== ��������ϵ�ṹ�� ========== */
typedef struct {
    float phase_a;
    float phase_b;
    float phase_c;
} ThreePhase_t;

/* ========== Alpha-Beta��ֹ����ϵ�ṹ�� ========== */
typedef struct {
    float alpha;
    float beta;
} AlphaBeta_t;

/* ========== D-Q��ת����ϵ�ṹ�� ========== */
typedef struct {
    float d;
    float q;
} DqAxis_t;

/* ========== ��ͨ�˲���״̬�ṹ�� ========== */
typedef struct {
    float current;      // ��ǰ����
    float previous;     // ��һ�ε����
    float filtered;     // �����˲����
} FilterState_t;

/* ========== PI�������ṹ�� ========== */
typedef struct {
    float kp, ki;           // PIϵ��
    float error, error_prev;// �����ǰһ�����
    float output;           // ���������
    float max_limit, min_limit; // ����޷�
} PIController_t;

/* ========== PLL���໷�ṹ�� ========== */
typedef struct {
    float theta;            // ��������λ�� (rad)
    float omega;            // �����Ľ�Ƶ�� (rad/s)
    float sin_wt, cos_wt;   // ��λ��������ֵ
    // PI������״̬���� (��������)
//    float pi_state_1, pi_state_2, pi_prev;
    float pi_integrator_state; // ���ڻ������ۼ�
    float pi_output;           // PI������������� (����)
    float error_prev;          // ��������ʽPI��������
} PLL_t;

/* ========== SVPWM�ṹ�� ========== */
typedef struct {
    float duty_a, duty_b,duty_c;         // ռ�ձ�
//    float t1, t2;           // ����ʸ������ʱ��
    float ta, tb, tc;       // ����PWM���αȽ�ʱ��
    float tcm1, tcm2, tcm3; // PWM�ȽϼĴ���ֵ
    float ua,ub,uc;
    float vm_amplitude,modulation_index,phase_angle;
} SVPWM_t;

/* ========== ������ϵͳ�ṹ�� ========== */
typedef struct {
    // ԭʼ��������
    AdcRawData_t adc_raw;

    // ת��Ϊ����λ�ĵ�ѹ����ֵ
    ThreePhase_t voltage;
    ThreePhase_t current;

    // ������ͨ�˲���ĵ�ѹ����ֵ (���ڿ���)
    ThreePhase_t voltage_filtered;
    ThreePhase_t current_filtered;

    // ����任���
    AlphaBeta_t voltage_ab;
    AlphaBeta_t current_ab;
    DqAxis_t voltage_dq;
    DqAxis_t current_dq;

    // ֱ���źŵ�ͨ�˲���
    FilterState_t filter_vo;
    FilterState_t filter_io;
    FilterState_t filter_v165_current;
    FilterState_t filter_v165_voltage;

    float filter_VDC;

    // �����źŴ�ͨ�˲���
    FilterBandpassData_t  bp_coeffs;
    FilterBandpassState_t bp_va, bp_vb, bp_vc;
    FilterBandpassState_t bp_ia, bp_ib, bp_ic;

    // ���໷�ڲ�ʹ�õ��˲���
    FilterState_t filter_vpd;
    FilterState_t filter_vpq;
    FilterState_t filter_vnd;
    FilterState_t filter_vnq;

    // PI������
    PIController_t pi_voltage;
    PIController_t pi_current_d;
    PIController_t pi_current_q;

    // ���໷
    PLL_t pll;

    // SVPWM
    SVPWM_t svpwm;

    // ���ƻ�·�ο�ֵ
    float voltage_ref;
    float current_d_ref;
    float current_q_ref;

    // �����������ָ���ѹ
    DqAxis_t voltage_cmd;
    AlphaBeta_t voltage_ab_cmd;

} ControlSystem_t;

/* ========== ���ݼ�¼�ṹ�� (������) ========== */
typedef struct {
    float cos_wt[DATA_BUFFER_SIZE];
    float current_a[DATA_BUFFER_SIZE];
    float voltage_a[DATA_BUFFER_SIZE];
    float current_d[DATA_BUFFER_SIZE];
    float voltage_positive_q[DATA_BUFFER_SIZE];
    Uint16 index;
} DataLogger_t;

/* ========== ȫ�ֱ������� ========== */
extern ControlSystem_t g_control_system;
extern DataLogger_t g_data_logger;

/* ========== �������� ========== */
void adc_config(void);
interrupt void adc_isr(void);
void control_system_init(void);

/* ========== �������� ========== */
// �޷�����
static inline float saturate(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

// �Ƕȹ�һ���� [0, 2*PI)
static inline float normalize_angle(float angle) {
    while (angle >= (2.0f * PI)) angle -= (2.0f * PI);
    while (angle < 0.0f) angle += (2.0f * PI);
    return angle;
}

#endif /* APP_ADC_USER_ADC_USER_H_ */
