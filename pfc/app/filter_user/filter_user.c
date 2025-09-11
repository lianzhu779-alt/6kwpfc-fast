/*
 * filter_user.c
 *
 *  Created on: 2025年6月22日
 *      Author: galaxy kono
 */
#include "filter_user.h"

float filter_1_x(float NEW_DATA,float LAST_OUT,float K_x)
{
    //实验出有相位偏移，目前用于锁相
    float32 NOW_OUT;
    NOW_OUT=(1-K_x)*LAST_OUT + K_x * NEW_DATA;
    return NOW_OUT;
}


float filter_2_x(float NEW_DATA,float OLD_DATA,float K_x)
{
    //一阶低通滤波器，把数据，留徒弟做ppt
    OLD_DATA=(1-K_x)*OLD_DATA + K_x * NEW_DATA;
    return OLD_DATA;
}



