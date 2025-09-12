/*
 * relay_user.h
 *
 *  Created on: 2025年7月31日
 *      Author: oyzy
 */

#ifndef APP_RELAY_USER_RELAY_USER_H_
#define APP_RELAY_USER_RELAY_USER_H_

#include "DSP28x_Project.h" // 引入您项目的设备头文件

/**
 * @brief 定义系统的工作模式
 */
typedef enum
{
    MODE_OFF,       // 关闭模式：所有继电器都关闭 (安全状态)
    MODE_PFC,       // PFC模式：RELAYB打开, RELAYA关闭
    MODE_BOOST      // Boost模式：RELAYB打开, RELAYA打开
} OperatingMode;


/**
 * @brief 初始化控制继电器的GPIO引脚
 * @details 将GPIO0和GPIO8配置为推挽输出，并默认设置为关闭状态。
 */
void InitRelays(void);

/**
 * @brief 设置系统的工作模式
 * @param mode 要设置的目标工作模式 (MODE_OFF, MODE_PFC, 或 MODE_BOOST)
 */
void SetOperatingMode(OperatingMode mode);

#endif /* APP_RELAY_USER_RELAY_USER_H_ */
