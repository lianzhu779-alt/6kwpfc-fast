/*
 * filter_user.c
 *
 *  Created on: 2025��6��22��
 *      Author: galaxy kono
 */
#include "filter_user.h"

float filter_1_x(float NEW_DATA,float LAST_OUT,float K_x)
{
    //ʵ�������λƫ�ƣ�Ŀǰ��������
    float32 NOW_OUT;
    NOW_OUT=(1-K_x)*LAST_OUT + K_x * NEW_DATA;
    return NOW_OUT;
}


float filter_2_x(float NEW_DATA,float OLD_DATA,float K_x)
{
    //һ�׵�ͨ�˲����������ݣ���ͽ����ppt
    OLD_DATA=(1-K_x)*OLD_DATA + K_x * NEW_DATA;
    return OLD_DATA;
}



