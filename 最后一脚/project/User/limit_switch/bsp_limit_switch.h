/**
 ******************************************************************************
 * 限位开关驱动
 *
 * ============================================================================
 * 使用示例：
 *
 * // 1. 初始化
 * LimitSwitch_Init();                            // 初始化所有限位开关
 *
 * // 2. 读取单个限位开关状态
 * if (LimitSwitch_IsTriggered(LIMIT_M1_ORIGIN)) {  // 电机1原点限位触发
 *     // 处理
 * }
 *
 * // 3. 读取所有限位开关状态
 * uint8_t status = LimitSwitch_ReadAll();        // 返回7位状态
 * if (status & (1 << LIMIT_M1_ORIGIN)) {         // 检查电机1原点
 *     // ...
 * }
 *
 * // 4. 清除状态
 * LimitSwitch_ClearState(LIMIT_M1_ORIGIN);       // 清除单个限位状态
 * LimitSwitch_ClearAll();                         // 清除所有限位状态
 *
 * // 5. 急停检测
 * if (LimitSwitch_EmergencyPressed()) {           // 急停被按下
 *     M1_Stop(); M2_Stop(); M3_Stop();           // 停止所有电机
 * }
 *
 * // 6. 获取触发计数
 * uint32_t count = LimitSwitch_GetTriggerCount(LIMIT_M1_ORIGIN);
 *
 * // 7. 便捷函数
 * // 检查电机1是否回到原点
 * if (LimitSwitch_HomeReached(1)) { }
 * // 检查电机2是否到达正限位
 * if (LimitSwitch_ForwardLimitReached(2)) { }
 *
 * ============================================================================
 * LimitSwitch_ID_TypeDef 类型说明：
 *   LIMIT_M1_ORIGIN = 0    - 电机1原点限位，PE0
 *   LIMIT_M1_FORWARD = 1    - 电机1正限位，PE1
 *   LIMIT_M2_ORIGIN = 2    - 电机2原点限位，PE2
 *   LIMIT_M2_FORWARD = 3    - 电机2正限位，PE3
 *   LIMIT_M3_ORIGIN = 4    - 电机3原点限位，PE4
 *   LIMIT_M3_FORWARD = 5    - 电机3正限位，PE5
 *   LIMIT_EMERGENCY = 6    - 急停开关，PE6
 *   LIMIT_MAX = 7           - 限位总数
 *
 * ============================================================================
 * LimitSwitch_State_TypeDef 类型说明：
 *   LIMIT_NOT_TRIGGERED = 0  - 未触发
 *   LIMIT_TRIGGERED = 1      - 已触发
 *
 * ============================================================================
 * 参数说明：
 * - 所有限位开关使用上拉输入模式
 * - 触发时引脚被拉低（低电平触发）
 * - active_level = 0 表示低电平触发
 *
 * ============================================================================
 * 引脚分配（PE端口，避免与PC冲突）：
 * - PE0: 电机1原点限位
 * - PE1: 电机1正限位
 * - PE2: 电机2原点限位
 * - PE3: 电机2正限位
 * - PE4: 电机3原点限位
 * - PE5: 电机3正限位
 * - PE6: 急停开关
 *
 * ============================================================================
 * 作者：工程训练中心116电控组
 * 日期：2026-03-24
 ******************************************************************************
 */

#ifndef __BSP_LIMIT_SWITCH_H__
#define __BSP_LIMIT_SWITCH_H__

#include "stm32f10x.h"

/* 限位开关编号
 *   LIMIT_M1_ORIGIN = 0    - 电机1原点限位，PE0
 *   LIMIT_M1_FORWARD = 1    - 电机1正限位，PE1
 *   LIMIT_M2_ORIGIN = 2    - 电机2原点限位，PE2
 *   LIMIT_M2_FORWARD = 3    - 电机2正限位，PE3
 *   LIMIT_M3_ORIGIN = 4    - 电机3原点限位，PE4
 *   LIMIT_M3_FORWARD = 5    - 电机3正限位，PE5
 *   LIMIT_EMERGENCY = 6    - 急停开关，PE6
 *   LIMIT_MAX = 7           - 限位总数
 */
typedef enum {
    LIMIT_M1_ORIGIN = 0,
    LIMIT_M1_FORWARD = 1,
    LIMIT_M2_ORIGIN = 2,
    LIMIT_M2_FORWARD = 3,
    LIMIT_M3_ORIGIN = 4,
    LIMIT_M3_FORWARD = 5,
    LIMIT_EMERGENCY = 6,
    LIMIT_MAX = 7
} LimitSwitch_ID_TypeDef;

/* 限位开关状态
 *   LIMIT_NOT_TRIGGERED = 0  - 未触发
 *   LIMIT_TRIGGERED = 1      - 已触发
 */
typedef enum {
    LIMIT_NOT_TRIGGERED = 0,
    LIMIT_TRIGGERED = 1
} LimitSwitch_State_TypeDef;

/* 限位开关配置结构体 */
typedef struct {
    LimitSwitch_ID_TypeDef id;           /* 编号 */
    GPIO_TypeDef* port;                  /* GPIO端口 */
    uint16_t pin;                       /* GPIO引脚 */
    uint8_t active_level;               /* 触发有效电平：0=低电平触发 */
    LimitSwitch_State_TypeDef state;     /* 当前状态 */
} LimitSwitch_Config_TypeDef;

/* 初始化所有限位开关 */
void LimitSwitch_Init(void);

/* 读取单个限位开关状态 */
LimitSwitch_State_TypeDef LimitSwitch_Read(LimitSwitch_ID_TypeDef switch_id);

/* 读取所有限位开关状态 */
uint8_t LimitSwitch_ReadAll(void);

/* 检查是否触发 */
uint8_t LimitSwitch_IsTriggered(LimitSwitch_ID_TypeDef switch_id);

/* 获取限位配置 */
LimitSwitch_Config_TypeDef* LimitSwitch_GetConfig(LimitSwitch_ID_TypeDef switch_id);

/* 清除限位状态 */
void LimitSwitch_ClearState(LimitSwitch_ID_TypeDef switch_id);

/* 清除所有限位状态 */
void LimitSwitch_ClearAll(void);

/* 急停检测 */
uint8_t LimitSwitch_EmergencyPressed(void);

/* 获取触发计数 */
uint32_t LimitSwitch_GetTriggerCount(LimitSwitch_ID_TypeDef switch_id);

/* 便捷函数：检查是否到达原点 */
uint8_t LimitSwitch_HomeReached(uint8_t motor_id);

/* 便捷函数：检查是否到达正限位 */
uint8_t LimitSwitch_ForwardLimitReached(uint8_t motor_id);

#endif /* __BSP_LIMIT_SWITCH_H__ */
