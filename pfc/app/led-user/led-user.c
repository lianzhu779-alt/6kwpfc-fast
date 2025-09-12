#include"led_user.h"


void LED_init(void)
{

    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO14=0;//io
    GpioCtrlRegs.GPADIR.bit.GPIO14=1;//输出
    GpioCtrlRegs.GPAPUD.bit.GPIO14=0;//上拉
    GpioDataRegs.GPASET.bit.GPIO14=1;//输出低电平

    GpioCtrlRegs.GPAMUX1.bit.GPIO15=0;//io
    GpioCtrlRegs.GPADIR.bit.GPIO15=1;//输出
    GpioCtrlRegs.GPAPUD.bit.GPIO15=0;//上拉
    GpioDataRegs.GPASET.bit.GPIO15=1;//输出低电平

    EDIS;
}
