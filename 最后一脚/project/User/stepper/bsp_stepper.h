/**
 ******************************************************************************
 * 步进电机底层驱动（VET6版本）
 *
 * ============================================================================
 * STM32F103VE LQFP100 硬件资源（根据ST官方参考手册RM0008核实）：
 *   - TIM1: FullRemap 后 CH1(PE9), CH4(PE14), CH3(PE13) — 高级定时器（有RCR）
 *   - TIM2: CH1(PA0), CH2(PB3), CH3(PA2), CH4(PA3) — 通用定时器
 *   - TIM3: CH1(PA6), CH2(PA7), CH3(PB0), CH4(PB1) — 通用定时器
 *   - TIM4: CH1(PB6), CH2(PB7), CH3(PB8), CH4(PB9) — 通用定时器
 *   - TIM5: CH1(PA0), CH2(PA1), CH3(PA2), CH4(PA3) — 通用定时器
 *
 * ============================================================================
 * 电机引脚分配（VET6 LQFP100修正版）：
 *   步进1: PE9(PUL/TIM1_CH1, FullRemap) + PC0(DIR) + ENA(不接) - 86mm同步带
 *   步进2: PA0(PUL/TIM2_CH1) + PC2(DIR) + ENA(不接) - 86mm同步带
 *   步进3: PA6(PUL/TIM3_CH1) + PC4(DIR) + ENA(不接) - 42mm传送带
 *   步进4: PA1(PUL/TIM5_CH2) + PC12(DIR) + ENA(不接) - 42mm打包
 *   步进5: PE14(PUL/TIM1_CH4) + PD2(DIR) + ENA(不接) - 57mm切割丝杆
 *   步进6: PE13(PUL/TIM1_CH3) + PD10(DIR) + ENA(不接) - 57mm切割丝杆
 *
 * 修正说明（2026-04-13 / 2026-04-14）：
 *   - TIM1 GPIO_FullRemap：PE13/PE14 才有 TIM1_CH3/CH4；步进1 CH1 迁至 PE9（非 PA8）
 *   - 步进3 PUL: TIM3_CH1(PA6)，PB0 作超声 TRIG
 *   - 步进4 PUL: 原PC12(非TIM5!) → 改为PA1(TIM5_CH2)
 *   - 步进5 PUL: PE14(TIM1_CH4)；步进6 PUL: PE13(TIM1_CH3)
 *   - DIR引脚: 步进4 DIR 原PD8 → 改为PC12，步进5 DIR 原PD2 → 保持不变
 *
 * 设计说明：
 *   - 同步带电机(步进1+2)联动，方向相反
 *   - 切割丝杆电机(步进5+6)联动，同方向
 *   - ENA不接，驱动器默认使能
 *   - TIM1为高级定时器，支持RCR重复计数器
 *
 * 作者：工程训练中心116电控组
 * 日期：2026-04-13（VET6 LQFP100修正版）
 ******************************************************************************
 */

#ifndef __BSP_STEPPER_H__
#define __BSP_STEPPER_H__

#include "stm32f10x.h"

/* 步进电机编号
 *   STEPPER_SYNC_A  = 0  - 同步带步进A，TIM1_CH1(PE9 FullRemap)，DIR=PC0
 *   STEPPER_SYNC_B  = 1  - 同步带步进B，TIM2_CH1(PA0)，DIR=PC2（与A方向相反）
 *   STEPPER_CONVEY  = 2  - 传送带步进，TIM3_CH1(PA6)，DIR=PC4
 *   STEPPER_PACK    = 3  - 打包步进，TIM5_CH2(PA1)，DIR=PC12
 *   STEPPER_SCREW_A = 4  - 切割丝杆A，TIM1_CH4(PE14)，DIR=PD2
 *   STEPPER_SCREW_B = 5  - 切割丝杆B，TIM1_CH3(PE13)，DIR=PD10（与A同方向）
 *   STEPPER_MAX     = 6  - 电机总数
 */
typedef enum {
    STEPPER_SYNC_A = 0,    /* 同步带A - 86mm/DMA860H */
    STEPPER_SYNC_B = 1,    /* 同步带B - 86mm/DMA860H */
    STEPPER_CONVEY = 2,    /* 传送带 - 42mm/DM422 */
    STEPPER_PACK   = 3,    /* 打包 - 42mm/DM422 */
    STEPPER_SCREW_A = 4,   /* 切割丝杆A - 57mm/DM542 */
    STEPPER_SCREW_B = 5,   /* 切割丝杆B - 57mm/DM542 */
    STEPPER_MAX    = 6
} Stepper_ID_TypeDef;

/* 向后兼容宏（旧项目代码使用 STEPPER_1/2/3/4）
 * M3=传送带(STEPPER_CONVEY)、M4=打包(STEPPER_PACK)
 */
#define STEPPER_1  STEPPER_SYNC_A
#define STEPPER_2  STEPPER_SYNC_B
#define STEPPER_3  STEPPER_CONVEY
#define STEPPER_4  STEPPER_PACK
#define STEPPER_5  STEPPER_SCREW_A
#define STEPPER_6  STEPPER_SCREW_B

/* 继续保留旧别名 */
#define STEPPER_86A    STEPPER_SYNC_A
#define STEPPER_86B    STEPPER_SYNC_B
#define STEPPER_42A    STEPPER_CONVEY
#define STEPPER_42B    STEPPER_PACK
#define STEPPER_SCREW  STEPPER_SCREW_A

/* ============================================================================
 * 兼容 bsp_motor_obj.h 的宏定义（便于旧代码迁移）
 * ============================================================================ */

/* 方向定义（必须在使用前定义） */
typedef enum {
    STEPPER_DIR_CW  = 0,
    STEPPER_DIR_CCW = 1
} Stepper_Dir_TypeDef;

/* 方向定义兼容 */
#define DIR_FORWARD   STEPPER_DIR_CW
#define DIR_BACKWARD  STEPPER_DIR_CCW
typedef Stepper_Dir_TypeDef Motor_Direction_TypeDef;

/* 电机状态兼容 */
typedef enum {
    MOTOR_STATE_IDLE = 0,
    MOTOR_STATE_RUNNING = 1,
    MOTOR_STATE_BRAKE = 2
} Motor_State_TypeDef;

/* 兼容宏 */
#define MOTOR_1  STEPPER_1
#define MOTOR_2  STEPPER_2
#define MOTOR_3  STEPPER_3
#define MOTOR_4  STEPPER_4
#define MOTOR_5  STEPPER_5
typedef Stepper_ID_TypeDef Motor_ID_TypeDef;

/* ============================================================================
 * 兼容层：MOTOR 数组和 M1/M2/M3/M4 快捷宏
 * ============================================================================ */

/* MOTOR 状态结构体 */
typedef struct {
    uint8_t state;           /* 0=停止, 1=运行, 2=制动 */
    uint16_t current_speed;  /* 当前速度 */
    int32_t position;        /* 当前位置 */
} Motor_State_Data_TypeDef;

/* 外部 MOTOR 数组声明（定义在 bsp_stepper.c） */
extern Motor_State_Data_TypeDef MOTOR[STEPPER_MAX];

/* 初始化所有电机（兼容 MOTOR_InitAll） */
void MOTOR_InitAll(void);

/* 电机控制宏（调用底层步进驱动） */
#define M1_Init()       Stepper_Init(); MOTOR[STEPPER_1].state = 0
#define M1_Run(s,d)     Stepper_RunAtSpeed(STEPPER_1, s, d); MOTOR[STEPPER_1].state = 1; MOTOR[STEPPER_1].current_speed = s
#define M1_Stop()       Stepper_Stop(STEPPER_1); MOTOR[STEPPER_1].state = 0; MOTOR[STEPPER_1].current_speed = 0
#define M1_GetPos()      Stepper_GetPosition(STEPPER_1)
#define M1_ResetPos()    Stepper_ResetPosition(STEPPER_1); MOTOR[STEPPER_1].position = 0
#define M1_Enable(e)     Stepper_Enable(STEPPER_1, e)
#define M1_Brake()       Stepper_Stop(STEPPER_1); MOTOR[STEPPER_1].state = 2
#define M1_SetSpeed(s)   Stepper_RunAtSpeed(STEPPER_1, s, STEPPER_DIR_CW)
#define M1_GetState()    (MOTOR_STATE_IDLE + MOTOR[STEPPER_1].state)

#define M2_Init()       /* 无单独初始化 */
#define M2_Run(s,d)     Stepper_RunAtSpeed(STEPPER_2, s, d); MOTOR[STEPPER_2].state = 1; MOTOR[STEPPER_2].current_speed = s
#define M2_Stop()       Stepper_Stop(STEPPER_2); MOTOR[STEPPER_2].state = 0; MOTOR[STEPPER_2].current_speed = 0
#define M2_GetPos()      Stepper_GetPosition(STEPPER_2)
#define M2_ResetPos()    Stepper_ResetPosition(STEPPER_2); MOTOR[STEPPER_2].position = 0
#define M2_Enable(e)     Stepper_Enable(STEPPER_2, e)
#define M2_Brake()       Stepper_Stop(STEPPER_2); MOTOR[STEPPER_2].state = 2
#define M2_SetSpeed(s)   Stepper_RunAtSpeed(STEPPER_2, s, STEPPER_DIR_CW)
#define M2_GetState()    (MOTOR_STATE_IDLE + MOTOR[STEPPER_2].state)

#define M3_Init()       /* 无单独初始化 */
#define M3_Run(s,d)     Stepper_RunAtSpeed(STEPPER_3, s, d); MOTOR[STEPPER_3].state = 1; MOTOR[STEPPER_3].current_speed = s
#define M3_Stop()       Stepper_Stop(STEPPER_3); MOTOR[STEPPER_3].state = 0; MOTOR[STEPPER_3].current_speed = 0
#define M3_GetPos()      Stepper_GetPosition(STEPPER_3)
#define M3_ResetPos()    Stepper_ResetPosition(STEPPER_3); MOTOR[STEPPER_3].position = 0
#define M3_Enable(e)     Stepper_Enable(STEPPER_3, e)
#define M3_Brake()       Stepper_Stop(STEPPER_3); MOTOR[STEPPER_3].state = 2
#define M3_SetSpeed(s)   Stepper_RunAtSpeed(STEPPER_3, s, STEPPER_DIR_CW)
#define M3_GetState()    (MOTOR_STATE_IDLE + MOTOR[STEPPER_3].state)

#define M4_Init()       /* 无单独初始化 */
#define M4_Run(s,d)     Stepper_RunAtSpeed(STEPPER_4, s, d); MOTOR[STEPPER_4].state = 1; MOTOR[STEPPER_4].current_speed = s
#define M4_Stop()       Stepper_Stop(STEPPER_4); MOTOR[STEPPER_4].state = 0; MOTOR[STEPPER_4].current_speed = 0
#define M4_GetPos()      Stepper_GetPosition(STEPPER_4)
#define M4_ResetPos()    Stepper_ResetPosition(STEPPER_4); MOTOR[STEPPER_4].position = 0
#define M4_Enable(e)     Stepper_Enable(STEPPER_4, e)
#define M4_Brake()       Stepper_Stop(STEPPER_4); MOTOR[STEPPER_4].state = 2
#define M4_SetSpeed(s)   Stepper_RunAtSpeed(STEPPER_4, s, STEPPER_DIR_CW)
#define M4_GetState()    (MOTOR_STATE_IDLE + MOTOR[STEPPER_4].state)

#define M5_Init()       /* 无单独初始化 */
#define M5_Run(s,d)     Stepper_RunAtSpeed(STEPPER_5, s, d); MOTOR[STEPPER_5].state = 1; MOTOR[STEPPER_5].current_speed = s
#define M5_Stop()       Stepper_Stop(STEPPER_5); MOTOR[STEPPER_5].state = 0; MOTOR[STEPPER_5].current_speed = 0
#define M5_GetPos()      Stepper_GetPosition(STEPPER_5)
#define M5_ResetPos()   Stepper_ResetPosition(STEPPER_5)
#define M5_Enable(e)    Stepper_Enable(STEPPER_5, e)
#define M5_Brake()      Stepper_Stop(STEPPER_5); MOTOR[STEPPER_5].state = 2
#define M5_SetSpeed(s)  Stepper_RunAtSpeed(STEPPER_5, s, STEPPER_DIR_CW)
#define M5_GetState()   (MOTOR_STATE_IDLE + MOTOR[STEPPER_5].state)

#define M6_Init()       /* 无单独初始化 */
#define M6_Run(s,d)     Stepper_RunAtSpeed(STEPPER_6, s, d); MOTOR[STEPPER_6].state = 1; MOTOR[STEPPER_6].current_speed = s
#define M6_Stop()       Stepper_Stop(STEPPER_6); MOTOR[STEPPER_6].state = 0; MOTOR[STEPPER_6].current_speed = 0
#define M6_GetPos()      Stepper_GetPosition(STEPPER_6)
#define M6_ResetPos()   Stepper_ResetPosition(STEPPER_6)
#define M6_Enable(e)    Stepper_Enable(STEPPER_6, e)
#define M6_Brake()      Stepper_Stop(STEPPER_6); MOTOR[STEPPER_6].state = 2
#define M6_SetSpeed(s)  Stepper_RunAtSpeed(STEPPER_6, s, STEPPER_DIR_CW)
#define M6_GetState()   (MOTOR_STATE_IDLE + MOTOR[STEPPER_6].state)

/* 兼容函数 */
#define Motor_Stop(id)           Stepper_Stop((Stepper_ID_TypeDef)(id))
#define Motor_ResetPosition(id)   Stepper_ResetPosition((Stepper_ID_TypeDef)(id))

/* 初始化所有步进电机 */
void Stepper_Init(void);

/* 在可能修改 AFIO 映射的初始化之后调用，恢复 TIM1 FullRemap 与 PE9/13/14 PUL 引脚 */
void Stepper_ReapplyTim1RemapAndPulGpio(void);

/* 关闭 TIM1 三路 PUL 的捕获/比较输出使能（定时器仍由 Run/Stop 控制） */
void Stepper_TIM1_ClearPulOutputs(void);

/* 六路步进 PUL 同时输出连续 PWM（关闭步进位置更新中断，用于接线/示波器测试） */
void Stepper_StartContinuousPwmTest(uint16_t hz);

/* 使能/禁用驱动器输出 */
void Stepper_Enable(Stepper_ID_TypeDef id, uint8_t ena);

/* 设置方向 */
void Stepper_SetDirLevel(Stepper_ID_TypeDef id, Stepper_Dir_TypeDef forward);

/* 连续旋转（速度模式） */
void Stepper_RunAtSpeed(Stepper_ID_TypeDef id, uint16_t steps_per_sec, Stepper_Dir_TypeDef dir);

/* 定量走步（指定脉冲数后自动停止） */
void Stepper_MoveSteps(Stepper_ID_TypeDef id, uint32_t total_steps, Stepper_Dir_TypeDef dir, uint16_t steps_per_sec);

/* 停止电机 */
void Stepper_Stop(Stepper_ID_TypeDef id);

/* 获取当前位置（累计脉冲数） */
int32_t Stepper_GetPosition(Stepper_ID_TypeDef id);

/* 重置位置为0 */
void Stepper_ResetPosition(Stepper_ID_TypeDef id);

/* 获取当前速度（Hz） */
uint16_t Stepper_GetCurrentSpeed(Stepper_ID_TypeDef id);

/* 定时器更新中断回调（供 stm32f10x_it.c 中断服务程序调用） */
void Stepper_OnTimUpdate(Stepper_ID_TypeDef id);

/* 同步带双电机控制（同时启动/停止，保证同步） */
void Stepper_SyncRun(Stepper_ID_TypeDef id_a, Stepper_ID_TypeDef id_b,
                     uint16_t steps_per_sec, Stepper_Dir_TypeDef dir);
/* 主流程一二同步带：电机1 DIR=取反，电机2 DIR=原方向 → 两皮带物理反向 */
void Stepper_SyncRun_Motor1Reversed(uint16_t steps_per_sec, Stepper_Dir_TypeDef dir);
void Stepper_SyncStop(void);

/* 丝杆双电机控制（同时启动/停止，保证同步） */
void Stepper_ScrewRun(Stepper_ID_TypeDef id_a, Stepper_ID_TypeDef id_b,
                      uint16_t steps_per_sec, Stepper_Dir_TypeDef dir);
void Stepper_ScrewStop(void);

/* 丝杆正转（向上，朝E0方向），内部保护PE0限位触发时停止 */
void Stepper_ScrewUp(uint16_t steps_per_sec);

/* 丝杆反转（向下，朝E1方向），内部保护PE1限位触发时停止 */
void Stepper_ScrewDown(uint16_t steps_per_sec);

/* 查询丝杆是否正在运行 */
uint8_t Stepper_ScrewIsRunning(void);

#endif /* __BSP_STEPPER_H__ */

/******************* (C) COPYRIGHT 2026 *****END OF FILE****/
