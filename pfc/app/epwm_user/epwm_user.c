/*
 * epwm_user.c
 *
 *  Created on: 2025��6��22��
 *      Author: galaxy kono
 */
#include "epwm_user.h"

//EPWM����
void Epwm2_config(void)//Ŀǰ���Ż����ɺ궨��
{
    EALLOW;
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;         //����Ƶ TBCLK=150Mhz��׼ʱ��
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;      //����Ƶ��0���ٻ�׼ʱ��
    EPwm2Regs.TBCTR = 0;                    //�����Ĵ�����������ʼֵ��0��ʼ��
    EPwm2Regs.TBPRD = TB_PRD/2;                 //��������ֵ��ʱ��14us��Ƶ��70k
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;       //EPWM�ȽϷ�ʽ:����ģʽ
 //   EPwm1Regs.CMPA.half.CMPA = TB_PRD/2;        //�ȽϼĴ�ֵ��ʼ��
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;           //����ģʽ����Ƚ�ֵ��ߵ�ƽ��Ч
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;           //�ݼ�ģʽ����Ƚ�ֵ��͵�ƽ��Ч
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;        //��������ѡ��ģʽ����
    EPwm2Regs.DBCTL.bit.POLSEL = 0x2;//DB_ACTV_HIC;          //�������ѡ��
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;       //�������ģʽ���ƣ���������ģ��
    EPwm2Regs.DBFED = TB_PRD/50;                   //�½�����ʱʱ�䣬�������2%=1/50
    EPwm2Regs.DBRED = TB_PRD/50;                   //��������ʱʱ�䣬�������2%=1/50
    EPwm2Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;        //��0�����в���
    EPwm2Regs.ETPS.bit.SOCAPRD = ET_1ST;         //ÿ���������ڲ���һ��
    EPwm2Regs.ETSEL.bit.SOCAEN = 1;         //ʹ��
    EDIS;
}

void Epwm3_config(void)
{
    EALLOW;
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;  //����Ƶ TBCLK=150Mhz
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   //����Ƶ��0
    EPwm3Regs.TBCTR = 0;                   //������ʼֵ
    EPwm3Regs.TBPRD = TB_PRD/2;          //��Ϊ����ģ��Ҫ��������/2��2142.85714285714/2=1071.428571428571429
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;    //EPWM�ȽϷ�ʽ:����ģʽ
//    EPwm2Regs.CMPA.half.CMPA = TB_PRD/2;      //�ȽϼĴ�ֵ
    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;   //����ģʽ����Ƚ�ֵ��ߵ�ƽ��Ч
    EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;   //�ݼ�ģʽ����Ƚ�ֵ��͵�ƽ��Ч
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;     //��������ѡ��ģʽ����
    EPwm3Regs.DBCTL.bit.POLSEL = 0x2;//DB_ACTV_HIC;     //�������ѡ��
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  //�������ģʽ���ƣ���������ģ��
    EPwm3Regs.DBFED = TB_PRD/50;  //�½�����ʱʱ�䣬�������2%=1/50
    EPwm3Regs.DBRED = TB_PRD/50;   //��������ʱʱ�䣬�������2%=1/50
    EDIS;
}

void Epwm4_config(void)
{
    EALLOW;
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;  //����Ƶ TBCLK=150Mhz
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   //����Ƶ��0
    EPwm4Regs.TBCTR = 0;
    EPwm4Regs.TBPRD = TB_PRD/2;            //��Ϊ����ģ��Ҫ��������/2
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
//    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;  //����Ƶ TBCLK=150Mhz
//    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   //����Ƶ��0
//    EPwm5Regs.TBCTR = 0;
//    EPwm5Regs.TBPRD = TB_PRD/2;            //��Ϊ����ģ��Ҫ��������/2
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



