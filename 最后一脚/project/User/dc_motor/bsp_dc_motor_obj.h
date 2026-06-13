/**
 ******************************************************************************
 * 直流减速电机对象化封装
 *
 * ============================================================================
 * 使用示例：
 *
 * // 1. 初始化
 * DCM_InitAll();                          // 初始化所有直流电机
 *
 * // 2. 使用快捷宏（推荐）
 * DCM1_Run(80, DIR_FORWARD);             // 电机1正转，80%占空比
 * DCM2_Run(60, DIR_BACKWARD);            // 电机2反转，60%占空比
 * DCM3_Run(100, DIR_FORWARD);            // 电机3正转，100%占空比
 * DCM4_Run(50, DIR_FORWARD);              // 电机4正转，50%占空比
 *
 * DCM1_Stop();                            // 停止电机1
 * DCM2_Stop();                            // 停止电机2
 *
 * // 3. 按圈数运行（需先标定）
 * DCM1_RunRev(2.5, 80, DIR_FORWARD);     // 电机1转2.5圈，80%占空比
 *
 * // 4. 状态查询
 * // 检查电机是否忙碌: DCM1_IsBusy() 返回1表示运行中
 *
 * // 5. 使用对象方法
 * DCM[DCM_1].Run(&DCM[DCM_1], 80, DIR_FORWARD);
 * DCM[DCM_1].Stop(&DCM[DCM_1]);
 *
 * // 6. 主循环必须调用
 * while (1) {
 *     DCM_ProcessAll();  // 处理定时运行的电机
 * }
 *
 * // 7. 标定（实测1圈需要多少毫秒）
 * DCM_CalibrateMotor(DCM_1, 80, 1.0f, 1200);  // 80%占空比转1圈实测1200ms
 *
 * ============================================================================
 * DCM_ID_TypeDef 类型说明：
 *   DCM_1 = 0   - 直流电机1，PWM=PC9(CH4)，IN1/IN2 硬接 3.3V/GND 方向固定
 *   DCM_2 = 1   - 直流电机2，PWM=PC7(CH2)，方向=PD8/PD9
 *   DCM_3 = 2   - 直流电机3，PWM=PC8(CH3)，方向=PD4/PD5
 *   DCM_4 = 3   - 直流电机4，PWM=PC9(CH4)，方向=PD6/PD7
 *   DCM_MAX = 4  - 电机总数
 *
 * ============================================================================
 * DCM_State_TypeDef 类型说明：
 *   DCM_STATE_IDLE = 0    - 空闲/已停止
 *   DCM_STATE_RUNNING = 1 - 运行中
 *   DCM_STATE_TIMED = 2  - 定时运行中（按圈数）
 *
 * ============================================================================
 * 参数说明：
 * - duty：PWM占空比，uint8_t类型，范围0~100（百分比）
 * - revolutions：转动的圈数，float类型，如2.5表示转2.5圈
 * - dir：方向，使用DIR_FORWARD(0)或DIR_BACKWARD(1)
 *
 * ============================================================================
 * 作者：工程训练中心116电控组
 * 日期：2026-03-24
 ******************************************************************************
 */

#ifndef __BSP_DC_MOTOR_OBJ_H__
#define __BSP_DC_MOTOR_OBJ_H__

#include "stm32f10x.h"
#include "bsp_gpio.h"
#include "bsp_stepper.h"

/* 前向声明 - 统一struct标签与typedef名 */
typedef struct _DCM_t DCM_t;

/* 直流电机编号
 *   DCM_1 = 0   - 直流电机1，PWM=PC9(CH4)，IN1/IN2 硬接 3.3V/GND 方向固定
 *   DCM_2 = 1   - 切膜电机，PWM=PC7(CH2)，方向=PD8/PD9（避让 TIM1 FullRemap）
 *   DCM_3 = 2   - 直流电机3，PWM=PC8(CH3)，方向=PD4/PD5
 *   DCM_4 = 3   - 直流电机4，PWM=PC6(CH1)，方向=PD6/PD7
 *   DCM_MAX = 4  - 电机总数
 */
typedef enum {
    DCM_1 = 0,
    DCM_2 = 1,
    DCM_3 = 2,
    DCM_4 = 3,
    DCM_MAX = 4
} DCM_ID_TypeDef;

/* 方向定义（复用bsp_motor_obj.h中的定义）
 *   DIR_FORWARD  = 0   - 正转（IN1高,IN2低）
 *   DIR_BACKWARD = 1   - 反转（IN1低,IN2高）
 */

/* 电机状态
 *   DCM_STATE_IDLE = 0    - 空闲/已停止
 *   DCM_STATE_RUNNING = 1 - 正在运行
 *   DCM_STATE_TIMED = 2   - 定时运行中
 */
typedef enum {
    DCM_STATE_IDLE = 0,
    DCM_STATE_RUNNING = 1,
    DCM_STATE_TIMED = 2
} DCM_State_TypeDef;

/* 直流电机引脚配置 */
typedef struct {
    GPIO_PortPin_TypeDef pwm_pin;   /* PWM引脚（TIM8输出） */
    GPIO_PortPin_TypeDef in1_pin;    /* 方向引脚1 */
    GPIO_PortPin_TypeDef in2_pin;    /* 方向引脚2 */
    TIM_TypeDef* tim;               /* 关联定时器（TIM8） */
    uint8_t tim_channel;           /* 定时器通道 */
} DCM_Pins_TypeDef;

/* 直流电机对象 */
struct _DCM_t {
    DCM_ID_TypeDef id;              /* 电机编号 */
    DCM_State_TypeDef state;       /* 当前状态 */
    Motor_Direction_TypeDef direction;  /* 当前方向 */
    uint8_t duty_percent;          /* 当前占空比（0~100） */

    /* 标定参数 */
    float ms_per_revolution;        /* 标定占空比下转1圈需要的毫秒数 */
    uint8_t calib_duty;           /* 标定占空比 */

    /* 定时运行参数 */
    uint32_t end_tick;            /* 结束时刻的SysTick值 */

    /* 引脚配置 */
    DCM_Pins_TypeDef pins;

    /* 函数指针 */
    void (*Init)(DCM_t* self);
    void (*Run)(DCM_t* self, uint8_t duty, Motor_Direction_TypeDef dir);
    void (*RunRevolutions)(DCM_t* self, float revolutions, uint8_t duty, Motor_Direction_TypeDef dir);
    void (*Stop)(DCM_t* self);
    void (*SetCalibration)(DCM_t* self, float ms_per_rev, uint8_t calib_duty);
    uint8_t (*IsBusy)(DCM_t* self);
    void (*Process)(DCM_t* self);
};

/* 外部声明 */
extern DCM_t DCM[DCM_MAX];

/* 初始化所有直流电机 */
void DCM_InitAll(void);

/* 停止所有电机 */
void DCM_StopAll(void);

/* 处理所有电机（主循环调用） */
void DCM_ProcessAll(void);

/* 标定电机 */
void DCM_CalibrateMotor(DCM_ID_TypeDef id, uint8_t test_duty, float revolutions, uint32_t actual_ms);

/* =============================================================================
 * 快捷宏定义
 * ============================================================================= */

/* 直流电机1快捷宏 */
#define DCM1_Run(d, dir)         DCM[DCM_1].Run(&DCM[DCM_1], d, dir)
#define DCM1_RunRev(r, d, dir)  DCM[DCM_1].RunRevolutions(&DCM[DCM_1], r, d, dir)
#define DCM1_Stop()              DCM[DCM_1].Stop(&DCM[DCM_1])
#define DCM1_IsBusy()            DCM[DCM_1].IsBusy(&DCM[DCM_1])

/* 切膜电机快捷宏（DCM_2） */
#define CUT2_Run(d, dir)         DCM[DCM_2].Run(&DCM[DCM_2], d, dir)
#define CUT2_RunRev(r, d, dir)  DCM[DCM_2].RunRevolutions(&DCM[DCM_2], r, d, dir)
#define CUT2_Stop()              DCM[DCM_2].Stop(&DCM[DCM_2])
#define CUT2_IsBusy()            DCM[DCM_2].IsBusy(&DCM[DCM_2])

/* 直流电机2快捷宏（别名，等同于切膜电机） */
#define DCM2_Run(d, dir)         CUT2_Run(d, dir)
#define DCM2_RunRev(r, d, dir)  CUT2_RunRev(r, d, dir)
#define DCM2_Stop()              CUT2_Stop()
#define DCM2_IsBusy()            CUT2_IsBusy()

/* 直流电机3快捷宏 */
#define DCM3_Run(d, dir)         DCM[DCM_3].Run(&DCM[DCM_3], d, dir)
#define DCM3_RunRev(r, d, dir)  DCM[DCM_3].RunRevolutions(&DCM[DCM_3], r, d, dir)
#define DCM3_Stop()              DCM[DCM_3].Stop(&DCM[DCM_3])
#define DCM3_IsBusy()            DCM[DCM_3].IsBusy(&DCM[DCM_3])

/* 直流电机4快捷宏 */
#define DCM4_Run(d, dir)         DCM[DCM_4].Run(&DCM[DCM_4], d, dir)
#define DCM4_RunRev(r, d, dir)  DCM[DCM_4].RunRevolutions(&DCM[DCM_4], r, d, dir)
#define DCM4_Stop()              DCM[DCM_4].Stop(&DCM[DCM_4])
#define DCM4_IsBusy()            DCM[DCM_4].IsBusy(&DCM[DCM_4])

#endif /* __BSP_DC_MOTOR_OBJ_H__ */
