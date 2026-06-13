/**
 ******************************************************************************
 * 继电器驱动
 *
 * ============================================================================
 * 继电器引脚定义（STM32F103VET6）：
 *   K_AIR  - PB12  - GPIO输出，推挽（喷气继电器）
 *   K_PUMP - PB13  - GPIO输出，推挽（水泵继电器）
 *   K_S    - PA15  - GPIO输出，推挽（预留继电器，关闭JTAG仅保留SWD）
 *
 * 注意：PA15 默认是 JTAG 的 JTDI，K_S 使用前须关闭 JTAG 复用。
 * ============================================================================
 *
 * 作者：工程训练中心116电控组
 * 日期：2026-04-19
 ******************************************************************************
 */

#ifndef __BSP_RELAY_H__
#define __BSP_RELAY_H__

#include "stm32f10x.h"

/* ============================================================================
 * 继电器编号枚举
 * ============================================================================ */
typedef enum {
    RELAY_K_AIR  = 0,   /* 喷气继电器 */
    RELAY_K_PUMP = 1,   /* 水泵继电器 */
    RELAY_K_S    = 2,   /* 预留继电器 */
    RELAY_FAN_E6 = 3,   /* 轴流风机继电器（E6/PE6，固态继电器） */
    RELAY_MAX    = 4
} Relay_ID_TypeDef;

/* ============================================================================
 * 函数声明
 * ============================================================================ */

/* 初始化所有继电器（推挽输出，初始关闭） */
void Relay_InitAll(void);

/* 打开指定继电器 */
void Relay_On(Relay_ID_TypeDef relay);

/* 关闭指定继电器 */
void Relay_Off(Relay_ID_TypeDef relay);

/* 切换指定继电器状态 */
void Relay_Toggle(Relay_ID_TypeDef relay);

/* 获取指定继电器当前状态（1=打开，0=关闭） */
uint8_t Relay_GetState(Relay_ID_TypeDef relay);

/* ============================================================================
 * 快捷宏（兼容设备层命名）
 * ============================================================================ */
#define K_AIR_On()   Relay_On(RELAY_K_AIR)
#define K_AIR_Off()  Relay_Off(RELAY_K_AIR)
#define K_AIR_Toggle() Relay_Toggle(RELAY_K_AIR)

#define K_PUMP_On()  Relay_On(RELAY_K_PUMP)
#define K_PUMP_Off() Relay_Off(RELAY_K_PUMP)
#define K_PUMP_Toggle() Relay_Toggle(RELAY_K_PUMP)

#define K_S_On()     Relay_On(RELAY_K_S)
#define K_S_Off()    Relay_Off(RELAY_K_S)
#define K_S_Toggle()  Relay_Toggle(RELAY_K_S)

#define FAN_E6_On()   Relay_On(RELAY_FAN_E6)
#define FAN_E6_Off()  Relay_Off(RELAY_FAN_E6)

#endif /* __BSP_RELAY_H__ */

/******************* (C) COPYRIGHT 2026 *****END OF FILE*****/
