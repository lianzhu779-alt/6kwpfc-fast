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



//N相继电器
void RELAYB_config(void)
{
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0=0;//io
    GpioCtrlRegs.GPADIR.bit.GPIO0=1;//输出
    GpioCtrlRegs.GPAPUD.bit.GPIO0=0;//使能上拉
    GpioDataRegs.GPASET.bit.GPIO0=1;//上拉
    EDIS;
}
//void Delay()
//{
//    Uint16 i;
//    Uint32 j;
//    for(i=0;i<16;i++)
//         for(j=0;j<100000;j++);
//}


//继电器配置
//AC，AB相继电器
//void RELAYA_config(void)
//{
//    EALLOW;
//    GpioCtrlRegs.GPAMUX1.bit.GPIO8=0;//io
//    GpioCtrlRegs.GPADIR.bit.GPIO8=1;//输出
//    GpioCtrlRegs.GPAPUD.bit.GPIO8=0;//使能上拉
//    GpioDataRegs.GPASET.bit.GPIO8=1;//上拉
//    EDIS;
//}



void main()
{

    InitSysCtrl();   //系统初始化

    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);//这几句是将FLASH中的程序COPY到RAM中运行，通常的目的是加快程序的运行速度，之后的程序运行时，
                                                                     //只要调用FLASH中RamfuncsLoadStart地址开始的相关函数，系统都会自动地指向RAM中相应的函数入口地址运行。
    InitFlash();
    InitAdc();//adc初始化
    DINT;          //关闭全局中断
    EALLOW;        //中断使能
    PieCtrlRegs.PIECTRL.bit.ENPIE=1;    //搬迁中断矢量表
    PieCtrlRegs.PIEIER1.bit.INTx1= 1;     //PIE中第n个小中段使能
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1=1; //允许ePWM的触发信号启动SEQ1
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1=1;             //启用SEQ1中断(外设中断)
    //  AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1=0;       //在每个SEQ1转换结束后产生中断
    IER=0x0001;
    PieVectTable.SEQ1INT=&adc_isr;//开全局中断
    EDIS;
    EINT;

    control_system_init();

//    EALLOW;
//        GpioCtrlRegs.GPAMUX1.bit.GPIO1=0;//io
//        GpioCtrlRegs.GPADIR.bit.GPIO1=1;//输出
//        GpioCtrlRegs.GPAPUD.bit.GPIO1=0;//使能上拉
//        GpioDataRegs.GPASET.bit.GPIO1=1;//上拉
//    EDIS;
    RELAYB_config();//继电器启动


    adc_config();//闭环

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
    GpioCtrlRegs.GPADIR.bit.GPIO14=1;//输出
    GpioCtrlRegs.GPAPUD.bit.GPIO14=0;//上拉
    GpioDataRegs.GPASET.bit.GPIO14=1;//输出低电平

    GpioCtrlRegs.GPAMUX1.bit.GPIO15=0;//io
    GpioCtrlRegs.GPADIR.bit.GPIO15=1;//输出
    GpioCtrlRegs.GPAPUD.bit.GPIO15=0;//上拉
    GpioDataRegs.GPASET.bit.GPIO15=1;//输出低电平

//    GpioCtrlRegs.GPAMUX1.bit.GPIO0=0;       //io
//    GpioCtrlRegs.GPADIR.bit.GPIO0=1;        //输出
//    GpioCtrlRegs.GPAPUD.bit.GPIO0=0;        //上拉
//    GpioDataRegs.GPASET.bit.GPIO0=1;        //上拉

//    GpioCtrlRegs.GPAMUX1.bit.GPIO1=0;       //io
//    GpioCtrlRegs.GPADIR.bit.GPIO1=1;        //输出
//    GpioCtrlRegs.GPAPUD.bit.GPIO1=0;        //上拉
//    GpioDataRegs.GPASET.bit.GPIO1=1;        //上拉
   EDIS;

    while(1)
    {

    }

}
