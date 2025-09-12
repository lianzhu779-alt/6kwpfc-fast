/*
 * filter_user.h
 *
 *  Created on: 2025年6月22日
 *      Author: galaxy kono
 */

#ifndef APP_FILTER_USER_FILTER_USER_H_
#define APP_FILTER_USER_FILTER_USER_H_
#include "DSP2833x_Device.h"     // DSP2833x 头文件
#include "DSP2833x_Examples.h"   // DSP2833x 示例头文件


float filter_1_x(float NEW_DATA,float LAST_OUT,float K_x);
float filter_2_x(float NEW_DATA,float OLD_DATA,float K_x);


#endif 
