/*
 * epwm_user.h
 *
 *  Created on: 2025Äê6ÔÂ22ÈÕ
 *      Author: galaxy kono
 */

#ifndef APP_EPWM_USER_EPWM_USER_H_
#define APP_EPWM_USER_EPWM_USER_H_

#include "DSP28x_Project.h"

#if (CPU_FRQ_150MHZ)
  #define CPU_CLK   150e6
#endif

#define PWM_CLK   10000
#define TB_PRD    CPU_CLK/PWM_CLK//150e6/10000=15000

void Epwm2_config(void);
void Epwm3_config(void);
void Epwm4_config(void);
void Epwm5_config(void);


#endif /* APP_EPWM_USER_EPWM_USER_H_ */
