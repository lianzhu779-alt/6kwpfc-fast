/*
 * epwm_user.c
 *
 *  Created on: 2025年6月22日
 *      Author: galaxy kono
 */
#include "epwm_user.h"

//EPWM配置
void Epwm2_config(void)//目前不优化，成宏定义
{
    EALLOW;
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;         //不分频 TBCLK=150Mhz基准时钟
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;      //不分频：0高速基准时钟
    EPwm2Regs.TBCTR = 0;                    //计数寄存器；计数初始值从0开始计
    EPwm2Regs.TBPRD = TB_PRD/2;                 //计数周期值，时间14us，频率70k
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;       //EPWM比较方式:增减模式
 //   EPwm1Regs.CMPA.half.CMPA = TB_PRD/2;        //比较寄存值初始化
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;           //递增模式到达比较值后高电平有效
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;           //递减模式到达比较值后低电平有效
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;        //死区输入选择模式控制
    EPwm2Regs.DBCTL.bit.POLSEL = 0x2;//DB_ACTV_HIC;          //输出极性选择
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;       //死区输出模式控制，启用死区模块
    EPwm2Regs.DBFED = TB_PRD/50;                   //下降沿延时时间，死区设计2%=1/50
    EPwm2Regs.DBRED = TB_PRD/50;                   //上升沿延时时间，死区设计2%=1/50
    EPwm2Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;        //在0处进行采样
    EPwm2Regs.ETPS.bit.SOCAPRD = ET_1ST;         //每隔多少周期采样一次
    EPwm2Regs.ETSEL.bit.SOCAEN = 1;         //使能
    EDIS;
}

void Epwm3_config(void)
{
    EALLOW;
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;  //不分频 TBCLK=150Mhz
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   //不分频：0
    EPwm3Regs.TBCTR = 0;                   //计数初始值
    EPwm3Regs.TBPRD = TB_PRD/2;          //因为增减模块要把总周期/2，2142.85714285714/2=1071.428571428571429
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;    //EPWM比较方式:增减模式
//    EPwm2Regs.CMPA.half.CMPA = TB_PRD/2;      //比较寄存值
    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;   //递增模式到达比较值后高电平有效
    EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;   //递减模式到达比较值后低电平有效
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;     //死区输入选择模式控制
    EPwm3Regs.DBCTL.bit.POLSEL = 0x2;//DB_ACTV_HIC;     //输出极性选择
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  //死区输出模式控制，启用死区模块
    EPwm3Regs.DBFED = TB_PRD/50;  //下降沿延时时间，死区设计2%=1/50
    EPwm3Regs.DBRED = TB_PRD/50;   //上升沿延时时间，死区设计2%=1/50
    EDIS;
}

void Epwm4_config(void)
{
    EALLOW;
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;  //不分频 TBCLK=150Mhz
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   //不分频：0
    EPwm4Regs.TBCTR = 0;
    EPwm4Regs.TBPRD = TB_PRD/2;            //因为增减模块要把总周期/2
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
//    EPwm3Regs.CMPA.half.CMPA = TB_PRD/2;
    EPwm4Regs.AQCTLA.bit.CAU = AQ_SET;
    EPwm4Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm4Regs.DBCTL.bit.POLSEL = 0x2;//DB_ACTV_HIC;
    EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm4Regs.DBFED = TB_PRD/50;
    EPwm4Regs.DBRED = TB_PRD/50;
    EDIS;
}

//void Epwm5_config(void)
//{
//    EALLOW;
//    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;  //不分频 TBCLK=150Mhz
//    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   //不分频：0
//    EPwm5Regs.TBCTR = 0;
//    EPwm5Regs.TBPRD = TB_PRD/2;            //因为增减模块要把总周期/2
//    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
////    EPwm3Regs.CMPA.half.CMPA = TB_PRD/2;
//    EPwm5Regs.AQCTLA.bit.CAU = AQ_SET;
//    EPwm5Regs.AQCTLA.bit.CAD = AQ_CLEAR;
//    EPwm5Regs.DBCTL.bit.IN_MODE = DBA_ALL;
//    EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
//    EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
//    EPwm5Regs.DBFED = TB_PRD/50;
//    EPwm5Regs.DBRED = TB_PRD/50;
//    EDIS;
//
//}



