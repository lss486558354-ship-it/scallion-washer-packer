/**
 ******************************************************************************
 * 舵机对象化封装
 *
 * ============================================================================
 * 使用示例：
 *
 * // 1. 初始化
 * SERVO_InitAll();                       // 初始化所有舵机
 *
 * // 2. 使用快捷宏（推荐）
 * SV1_SetAngle(90);                     // 舵机1转到90度
 * SV2_SetAngle(45);                    // 舵机2转到45度
 * SV3_SetAngle(0);                     // 舵机3转到0度
 * SV4_SetAngle(180);                   // 舵机4转到180度
 *
 * float angle = SV1_GetAngle();         // 获取舵机1当前角度
 *
 * SV1_Enable(1);                        // 使能舵机1输出
 * SV1_Enable(0);                        // 禁用舵机1输出
 *
 * // 3. 使用对象方法（与快捷宏效果相同）
 * SERVO[SERVO_1].SetAngle(&SERVO[SERVO_1], 90.0f);
 * SERVO[SERVO_2].SetAngle(&SERVO[SERVO_2], 45.0f);
 * angle = SERVO[SERVO_1].GetAngle(&SERVO[SERVO_1]);
 *
 * // 4. 角度范围
 * // 有效角度范围：0度 ~ 180度
 * // 常用角度：0度（左极限）、90度（中间）、180度（右极限）
 *
 * ============================================================================
 * Servo_ID_TypeDef 类型说明：
 *   SERVO_1 = 0   - 舵机1，PB6，TIM4_CH1
 *   SERVO_2 = 1   - 舵机2，PB7，TIM4_CH2
 *   SERVO_3 = 2   - 舵机3，PA11，TIM1_CH2
 *   SERVO_4 = 3   - 舵机4，PA12，TIM1_CH3
 *   SERVO_MAX = 4  - 舵机总数
 *
 * ============================================================================
 * PWM参数：
 * - 频率：50Hz（周期20ms）
 * - 脉宽范围：0.5ms ~ 2.5ms
 * - 角度映射：0度=0.5ms，90度=1.5ms，180度=2.5ms
 *
 * ============================================================================
 * 参数说明：
 * - angle：角度值，float类型，范围0.0度 ~ 180.0度
 * - en：使能标志，1=使能输出，0=禁用输出
 *
 * ============================================================================
 * 作者：工程训练中心116电控组
 * 日期：2026-03-24
 ******************************************************************************
 */

#ifndef __BSP_SERVO_OBJ_H__
#define __BSP_SERVO_OBJ_H__

#include "stm32f10x.h"
#include "bsp_gpio.h"

/* 前向声明 - 统一struct标签与typedef名 */
typedef struct _SERVO_t SERVO_t;

/* 舵机编号
 *   SERVO_1 = 0   - 舵机1，PB6，TIM4_CH1
 *   SERVO_2 = 1   - 舵机2，PB7，TIM4_CH2
 *   SERVO_3 = 2   - 舵机3，PA11，TIM1_CH2
 *   SERVO_4 = 3   - 舵机4，PA12，TIM1_CH3
 *   SERVO_MAX = 4  - 舵机总数
 */
typedef enum {
    SERVO_1 = 0,
    SERVO_2 = 1,
    SERVO_3 = 2,
    SERVO_4 = 3,
    SERVO_MAX = 4
} Servo_ID_TypeDef;

/* 舵机引脚配置 */
typedef struct {
    GPIO_PortPin_TypeDef pin;   /* 控制引脚 */
    TIM_TypeDef* tim;           /* 关联定时器 */
    uint8_t tim_channel;        /* 定时器通道 */
} Servo_Pins_TypeDef;

/* 舵机对象 */
struct _SERVO_t {
    Servo_ID_TypeDef id;            /* 舵机编号 */
    float current_angle;           /* 当前角度（度） */
    float target_angle;            /* 目标角度（度） */
    float min_angle;               /* 最小角度，默认0度 */
    float max_angle;               /* 最大角度，默认180度 */

    /* 引脚配置 */
    Servo_Pins_TypeDef pins;

    /* PWM参数 */
    uint16_t pwm_min;             /* 0度对应的CCR值（默认500） */
    uint16_t pwm_max;             /* 180度对应的CCR值（默认2500） */
    uint16_t pwm_center;          /* 90度对应的CCR值（默认1500） */

    /* 函数指针 */
    void (*Init)(SERVO_t* self);
    void (*SetAngle)(SERVO_t* self, float angle);
    void (*SetAngleAbsolute)(SERVO_t* self, float angle);
    float (*GetAngle)(SERVO_t* self);
    void (*Enable)(SERVO_t* self, uint8_t en);
};

/* 外部声明 */
extern SERVO_t SERVO[SERVO_MAX];

/* 初始化所有舵机 */
void SERVO_InitAll(void);

/* =============================================================================
 * 快捷宏定义
 * ============================================================================= */

/* 舵机1快捷宏 */
#define SV1_SetAngle(a)    SERVO[SERVO_1].SetAngle(&SERVO[SERVO_1], a)
#define SV1_GetAngle()     SERVO[SERVO_1].GetAngle(&SERVO[SERVO_1])
#define SV1_Enable(e)      SERVO[SERVO_1].Enable(&SERVO[SERVO_1], e)

/* 舵机2快捷宏 */
#define SV2_SetAngle(a)    SERVO[SERVO_2].SetAngle(&SERVO[SERVO_2], a)
#define SV2_GetAngle()     SERVO[SERVO_2].GetAngle(&SERVO[SERVO_2])
#define SV2_Enable(e)      SERVO[SERVO_2].Enable(&SERVO[SERVO_2], e)

/* 舵机3快捷宏 */
#define SV3_SetAngle(a)    SERVO[SERVO_3].SetAngle(&SERVO[SERVO_3], a)
#define SV3_GetAngle()     SERVO[SERVO_3].GetAngle(&SERVO[SERVO_3])
#define SV3_Enable(e)      SERVO[SERVO_3].Enable(&SERVO[SERVO_3], e)

/* 舵机4快捷宏 */
#define SV4_SetAngle(a)    SERVO[SERVO_4].SetAngle(&SERVO[SERVO_4], a)
#define SV4_GetAngle()     SERVO[SERVO_4].GetAngle(&SERVO[SERVO_4])
#define SV4_Enable(e)      SERVO[SERVO_4].Enable(&SERVO[SERVO_4], e)

#endif /* __BSP_SERVO_OBJ_H__ */
