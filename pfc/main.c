#include "DSP28x_Project.h"
#include"math.h"
#include"stddef.h"
#include"stdint.h"
#include "epwm_user.h"
#include "adc_user.h"

interrupt void  adc_isr();

void adc_config();
//void RELAYA_config();
void RELAYB_config();

#pragma CODE_SECTION(adc_isr, "ramfuncs");
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
extern Uint16 RamfuncsLoadSize;



//N��̵���
void RELAYB_config(void)
{
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0=0;//io
    GpioCtrlRegs.GPADIR.bit.GPIO0=1;//���
    GpioCtrlRegs.GPAPUD.bit.GPIO0=0;//ʹ������
    GpioDataRegs.GPASET.bit.GPIO0=1;//����
    EDIS;
}
//void Delay()
//{
//    Uint16 i;
//    Uint32 j;
//    for(i=0;i<16;i++)
//         for(j=0;j<100000;j++);
//}


//�̵�������
//AC��AB��̵���
//void RELAYA_config(void)
//{
//    EALLOW;
//    GpioCtrlRegs.GPAMUX1.bit.GPIO8=0;//io
//    GpioCtrlRegs.GPADIR.bit.GPIO8=1;//���
//    GpioCtrlRegs.GPAPUD.bit.GPIO8=0;//ʹ������
//    GpioDataRegs.GPASET.bit.GPIO8=1;//����
//    EDIS;
//}



void main()
{

    InitSysCtrl();   //ϵͳ��ʼ��

    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);//�⼸���ǽ�FLASH�еĳ���COPY��RAM�����У�ͨ����Ŀ���Ǽӿ����������ٶȣ�֮��ĳ�������ʱ��
                                                                     //ֻҪ����FLASH��RamfuncsLoadStart��ַ��ʼ����غ�����ϵͳ�����Զ���ָ��RAM����Ӧ�ĺ�����ڵ�ַ���С�
    InitFlash();
    InitAdc();//adc��ʼ��
    DINT;          //�ر�ȫ���ж�
    EALLOW;        //�ж�ʹ��
    PieCtrlRegs.PIECTRL.bit.ENPIE=1;    //��Ǩ�ж�ʸ����
    PieCtrlRegs.PIEIER1.bit.INTx1= 1;     //PIE�е�n��С�ж�ʹ��
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1=1; //����ePWM�Ĵ����ź�����SEQ1
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1=1;             //����SEQ1�ж�(�����ж�)
    //  AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1=0;       //��ÿ��SEQ1ת������������ж�
    IER=0x0001;
    PieVectTable.SEQ1INT=&adc_isr;//��ȫ���ж�
    EDIS;
    EINT;

    control_system_init();

//    EALLOW;
//        GpioCtrlRegs.GPAMUX1.bit.GPIO1=0;//io
//        GpioCtrlRegs.GPADIR.bit.GPIO1=1;//���
//        GpioCtrlRegs.GPAPUD.bit.GPIO1=0;//ʹ������
//        GpioDataRegs.GPASET.bit.GPIO1=1;//����
//    EDIS;
    RELAYB_config();//�̵�������


    ADC_Init();//�ջ�

    InitEPwm2Gpio();
    InitEPwm3Gpio();
    InitEPwm4Gpio();

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=0;
    EDIS;

    Epwm2_config();
    Epwm3_config();
    Epwm4_config();

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=1;
    EDIS;

    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO14=0;//io
    GpioCtrlRegs.GPADIR.bit.GPIO14=1;//���
    GpioCtrlRegs.GPAPUD.bit.GPIO14=0;//����
    GpioDataRegs.GPASET.bit.GPIO14=1;//����͵�ƽ

    GpioCtrlRegs.GPAMUX1.bit.GPIO15=0;//io
    GpioCtrlRegs.GPADIR.bit.GPIO15=1;//���
    GpioCtrlRegs.GPAPUD.bit.GPIO15=0;//����
    GpioDataRegs.GPASET.bit.GPIO15=1;//����͵�ƽ

//    GpioCtrlRegs.GPAMUX1.bit.GPIO0=0;       //io
//    GpioCtrlRegs.GPADIR.bit.GPIO0=1;        //���
//    GpioCtrlRegs.GPAPUD.bit.GPIO0=0;        //����
//    GpioDataRegs.GPASET.bit.GPIO0=1;        //����

//    GpioCtrlRegs.GPAMUX1.bit.GPIO1=0;       //io
//    GpioCtrlRegs.GPADIR.bit.GPIO1=1;        //���
//    GpioCtrlRegs.GPAPUD.bit.GPIO1=0;        //����
//    GpioDataRegs.GPASET.bit.GPIO1=1;        //����
   EDIS;

    while(1)
    {

    }

}
